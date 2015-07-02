#include "Crazyradio.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <libusb-1.0/libusb.h>

enum
{
    SET_RADIO_CHANNEL   = 0x01,
    SET_RADIO_ADDRESS   = 0x02,
    SET_DATA_RATE       = 0x03,
    SET_RADIO_POWER     = 0x04,
    SET_RADIO_ARD       = 0x05,
    SET_RADIO_ARC       = 0x06,
    ACK_ENABLE          = 0x10,
    SET_CONT_CARRIER    = 0x20,
    SCANN_CHANNELS      = 0x21,
    LAUNCH_BOOTLOADER   = 0xFF,
};

Crazyradio::Crazyradio(uint32_t devid)
    : m_ctx(NULL)
    , m_handle(NULL)
    , m_version(0.0)
    , m_channel(0)
    , m_address(0)
    , m_datarate(Datarate_250KPS)
{
    int result = libusb_init(&m_ctx);
    if (result != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(result));
    }
    if (!open(devid)) {
        libusb_exit(m_ctx);
        throw std::runtime_error("Could not find Crazyradio with given devId!");
    }
}

Crazyradio::~Crazyradio()
{
    int result;



    // result = libusb_reset_device(m_handle);
    // if (result != LIBUSB_SUCCESS) {
    //     std::cerr << libusb_error_name(result) << std::endl;
    // }

    result = libusb_release_interface(m_handle, 0);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << libusb_error_name(result) << std::endl;
    }

    // function returns void => no error checking
    libusb_close(m_handle);

    // function returns void => no error checking
    libusb_exit(m_ctx);
}

bool Crazyradio::open(uint32_t devid)
{
    // discover devices
    libusb_device **list;
    libusb_device *found = NULL;
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    ssize_t i = 0;
    uint32_t foundid = 0;
    int err = 0;
    if (cnt < 0) {
        std::cerr << "Error during get_device_list" << std::endl;
        return false;
    }
    for (i = 0; i < cnt; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor deviceDescriptor;
        err = libusb_get_device_descriptor(device, &deviceDescriptor);
        if (err != LIBUSB_SUCCESS) {
            std::cerr << libusb_error_name(err) << std::endl;
            libusb_free_device_list(list, 1);
            return false;
        }
        else if (deviceDescriptor.idVendor == 0x1915 &&
                 deviceDescriptor.idProduct == 0x7777) {
            if (foundid == devid) {
                found = device;
                break;
            }
            ++foundid;
        }
    }
    if (found) {
        err = libusb_open(found, &m_handle);
        if (err != LIBUSB_SUCCESS) {
            std::cerr << libusb_error_name(err) << std::endl;
            libusb_free_device_list(list, 1);
            return false;
        }
        libusb_device_descriptor deviceDescriptor;
        err = libusb_get_device_descriptor(found, &deviceDescriptor);
        if (err != LIBUSB_SUCCESS) {
            std::cerr << libusb_error_name(err) << std::endl;
            return false;
        }
        std::stringstream sstr;
        sstr << std::hex << (deviceDescriptor.bcdDevice >> 8) << "." << (deviceDescriptor.bcdDevice & 0xFF);
        sstr >> m_version;
    }
    libusb_free_device_list(list, 1);

    // configure
    if (m_handle)
    {
        err = libusb_set_configuration(m_handle, 1);
        if (err != LIBUSB_SUCCESS) {
            std::cerr << libusb_error_name(err) << std::endl;
            return false;
        }

        err = libusb_claim_interface(m_handle, 0);
        if (err != LIBUSB_SUCCESS) {
            std::cerr << libusb_error_name(err) << std::endl;
            return false;
        }

        setDatarate(Datarate_2MPS);
        setChannel(2);
        setContCarrier(false);
        setAddress(0xE7E7E7E7E7);
        setPower(Power_0DBM);
        setArc(3);
        setArdBytes(32);

        std::cout << "Configured Dongle with version " << m_version << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

void Crazyradio::setChannel(uint8_t channel)
{
    sendVendorSetup(SET_RADIO_CHANNEL, channel, 0, NULL, 0);
    m_channel = channel;
}

void Crazyradio::setAddress(uint64_t address)
{
    unsigned char a[5];
    a[4] = (address >> 0) & 0xFF;
    a[3] = (address >> 8) & 0xFF;
    a[2] = (address >> 16) & 0xFF;
    a[1] = (address >> 24) & 0xFF;
    a[0] = (address >> 32) & 0xFF;

    // sendVendorSetup(SET_RADIO_ADDRESS, 0, 0, a, 5);
    // unsigned char a[] = {0xe7, 0xe7, 0xe7, 0xe7, 0x02};

    int status = libusb_control_transfer(
        m_handle,
        LIBUSB_REQUEST_TYPE_VENDOR,
        SET_RADIO_ADDRESS,
        0,
        0,
        a,
        5,
        /*timeout*/ 1000);
    // if (status != LIBUSB_SUCCESS) {
    //     std::cerr << "sendVendorSetup: " << libusb_error_name(status) << std::endl;
    // }
    m_address = address;
}

void Crazyradio::setDatarate(Datarate datarate)
{
    sendVendorSetup(SET_DATA_RATE, datarate, 0, NULL, 0);
    m_datarate = datarate;
}

void Crazyradio::setPower(Power power)
{
    sendVendorSetup(SET_RADIO_POWER, power, 0, NULL, 0);
}

void Crazyradio::setArc(uint8_t arc)
{
    sendVendorSetup(SET_RADIO_ARC, arc, 0, NULL, 0);
}

void Crazyradio::setArdTime(uint8_t us)
{
    // Auto Retransmit Delay:
    // 0000 - Wait 250uS
    // 0001 - Wait 500uS
    // 0010 - Wait 750uS
    // ........
    // 1111 - Wait 4000uS

    // Round down, to value representing a multiple of 250uS
    int t = (us / 250) - 1;
    if (t < 0) {
        t = 0;
    }
    if (t > 0xF) {
        t = 0xF;
    }
    sendVendorSetup(SET_RADIO_ARD, t, 0, NULL, 0);
}

void Crazyradio::setArdBytes(uint8_t nbytes)
{
    sendVendorSetup(SET_RADIO_ARD, 0x80 | nbytes, 0, NULL, 0);
}

void Crazyradio::setAckEnable(bool enable)
{
    sendVendorSetup(ACK_ENABLE, enable, 0, NULL, 0);
}

void Crazyradio::setContCarrier(bool active)
{
    sendVendorSetup(SET_CONT_CARRIER, active, 0, NULL, 0);
}

void Crazyradio::sendPacket(
    const uint8_t* data,
    uint32_t length,
    Ack& result)
{
    int status;
    int transferred;

    if (!m_handle) {
        std::cerr << "No valid Crazyradio handle!" << std::endl;
        return;
    }

    // Send data
    status = libusb_bulk_transfer(
        m_handle,
        /* endpoint*/ (0x01 | LIBUSB_ENDPOINT_OUT),
        (uint8_t*)data,
        length,
        &transferred,
        /*timeout*/ 1000);
    if (status != LIBUSB_SUCCESS) {
        std::cerr << "Send " << libusb_error_name(status) << std::endl;
    }
    if (length != transferred) {
        std::cerr << "Did transfer " << transferred << " but " << length << " was requested!" << std::endl;
    }

    // Read result
    result.ack = false;
    result.size = 0;
    status = libusb_bulk_transfer(
        m_handle,
        /* endpoint*/ (0x81 | LIBUSB_ENDPOINT_IN),
        (unsigned char*)&result,
        sizeof(result) - 1,
        &transferred,
        /*timeout*/ 1000);
    if (status != LIBUSB_SUCCESS) {
        std::cerr << "Receive " << libusb_error_name(status) << std::endl;
    }

    result.size = transferred - 1;
}

void Crazyradio::sendPacketNoAck(
    const uint8_t* data,
    uint32_t length)
{
    int status;
    int transferred;

    if (!m_handle) {
        std::cerr << "No valid Crazyradio handle!" << std::endl;
        return;
    }

    // Send data
    status = libusb_bulk_transfer(
        m_handle,
        /* endpoint*/ (0x01 | LIBUSB_ENDPOINT_OUT),
        (uint8_t*)data,
        length,
        &transferred,
        /*timeout*/ 1000);
    if (status != LIBUSB_SUCCESS) {
        std::cerr << "Send " << libusb_error_name(status) << std::endl;
    }
    if (length != transferred) {
        std::cerr << "Did transfer " << transferred << " but " << length << " was requested!" << std::endl;
    }
}

void Crazyradio::sendVendorSetup(
    uint8_t request,
    uint16_t value,
    uint16_t index,
    const unsigned char* data,
    uint16_t length)
{
    if (!m_handle) {
        std::cerr << "No valid Crazyradio handle!" << std::endl;
        return;
    }

    int status = libusb_control_transfer(
        m_handle,
        LIBUSB_REQUEST_TYPE_VENDOR,
        request,
        value,
        index,
        (unsigned char*)data,
        length,
        /*timeout*/ 1000);
    if (status != LIBUSB_SUCCESS) {
        std::cerr << "sendVendorSetup: " << libusb_error_name(status) << std::endl;
    }
}
