#include "USBDevice.h"

#include <sstream>
#include <stdexcept>

#include <libusb-1.0/libusb.h>

USBDevice::USBDevice(
    uint16_t idVendor,
    uint16_t idProduct)
    : m_ctx(NULL)
    , m_handle(NULL)
    , m_version(0.0)
    , m_idVendor(idVendor)
    , m_idProduct(idProduct)
{
    int result = libusb_init(&m_ctx);
    if (result != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(result));
    }
}

USBDevice::~USBDevice()
{
    if (m_handle) {
        // ignore the error (we don't want to throw in dtor)
        libusb_release_interface(m_handle, 0);

        // function returns void => no error checking
        libusb_close(m_handle);
    }

    // function returns void => no error checking
    libusb_exit(m_ctx);
}

uint32_t USBDevice::numDevices(
  uint16_t idVendor,
  uint16_t idProduct)
{
    libusb_context* ctx;
    int result = libusb_init(&ctx);
    if (result != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(result));
    }

    // discover devices
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    ssize_t i = 0;
    uint32_t num = 0;
    int err = 0;
    if (cnt < 0) {
        throw std::runtime_error("Error during get_device_list");
    }
    for (i = 0; i < cnt; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor deviceDescriptor;
        err = libusb_get_device_descriptor(device, &deviceDescriptor);
        if (err != LIBUSB_SUCCESS) {
            libusb_free_device_list(list, 1);
            throw std::runtime_error(libusb_error_name(err));
        }
        else if (deviceDescriptor.idVendor == idVendor &&
                 deviceDescriptor.idProduct == idProduct) {
            ++num;
        }
    }
    libusb_free_device_list(list, 1);

    // function returns void => no error checking
    libusb_exit(ctx);

    return num;
}

void USBDevice::open(
    uint32_t devid)
{
    // discover devices
    libusb_device **list;
    libusb_device *found = NULL;
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    ssize_t i = 0;
    uint32_t foundid = 0;
    int err = 0;
    if (cnt < 0) {
        throw std::runtime_error("Error during get_device_list");
    }
    for (i = 0; i < cnt; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor deviceDescriptor;
        err = libusb_get_device_descriptor(device, &deviceDescriptor);
        if (err != LIBUSB_SUCCESS) {
            libusb_free_device_list(list, 1);
            throw std::runtime_error(libusb_error_name(err));
        }
        else if (deviceDescriptor.idVendor == m_idVendor &&
                 deviceDescriptor.idProduct == m_idProduct) {
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
            libusb_free_device_list(list, 1);
            throw std::runtime_error(libusb_error_name(err));
        }
        libusb_device_descriptor deviceDescriptor;
        err = libusb_get_device_descriptor(found, &deviceDescriptor);
        if (err != LIBUSB_SUCCESS) {
            throw std::runtime_error(libusb_error_name(err));
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
            throw std::runtime_error(libusb_error_name(err));
        }

        err = libusb_claim_interface(m_handle, 0);
        if (err != LIBUSB_SUCCESS) {
            throw std::runtime_error(libusb_error_name(err));
        }
    }
    else
    {
        std::stringstream sstr;
        sstr << "No matching USB Device with devid = " << devid << " found!";
        throw std::runtime_error(sstr.str());
    }
}

void USBDevice::sendVendorSetup(
    uint8_t request,
    uint16_t value,
    uint16_t index,
    const unsigned char* data,
    uint16_t length)
{
    if (!m_handle) {
        throw std::runtime_error("No valid device handle!");
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
        throw std::runtime_error(libusb_error_name(status));
    }
}
