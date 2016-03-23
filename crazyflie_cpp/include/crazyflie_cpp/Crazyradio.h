#pragma once

#include <stdint.h>
#include <vector>

// forward declarations
struct libusb_context;
struct libusb_device_handle;

class Crazyradio
{
public:
    enum Datarate
    {
        Datarate_250KPS = 0,
        Datarate_1MPS   = 1,
        Datarate_2MPS   = 2,
    };

    enum Power
    {
        Power_M18DBM = 0,
        Power_M12DBM = 1,
        Power_M6DBM  = 2,
        Power_0DBM   = 3,
    };

    struct Ack
    {
        Ack()
          : ack(0)
          , size(0)
        {}

        uint8_t ack:1;
        uint8_t powerDet:1;
        uint8_t retry:4;
        uint8_t data[32];

        uint8_t size;
    }__attribute__((packed));

public:
    Crazyradio(uint32_t devid);

    ~Crazyradio();

    void setChannel(uint8_t channel);
    uint8_t getChannel() const {
        return m_channel;
    }
    void setAddress(uint64_t address);
    uint64_t getAddress() const {
        return m_address;
    }
    void setDatarate(Datarate datarate);
    Datarate getDatarate() const {
        return m_datarate;
    }
    void setPower(Power power);
    void setArc(uint8_t arc);
    void setArdTime(uint8_t us);
    void setArdBytes(uint8_t nbytes);
    void setAckEnable(bool enable);
    void setContCarrier(bool active);

    void sendPacket(
        const uint8_t* data,
        uint32_t length,
        Ack& result);

    void sendPacketNoAck(
        const uint8_t* data,
        uint32_t length);

private:
    bool open(uint32_t devid);
    void sendVendorSetup(
        uint8_t request,
        uint16_t value,
        uint16_t index,
        const unsigned char* data,
        uint16_t length);

private:
    libusb_context* m_ctx;
    libusb_device_handle *m_handle;

    float m_version;
    uint8_t m_channel;
    uint64_t m_address;
    Datarate m_datarate;
};
