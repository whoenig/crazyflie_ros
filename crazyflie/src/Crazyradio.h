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
        bool ack;
        bool powerDet;
        uint32_t retry;
        std::vector<uint8_t> data;
    };

public:
    Crazyradio(uint32_t devid);

    ~Crazyradio();

    void setChannel(uint8_t channel);
    void setAddress(uint64_t address);
    void setDatarate(Datarate datarate);
    void setPower(Power power);
    void setArc(uint8_t arc);
    void setArdTime(uint8_t us);
    void setArdBytes(uint8_t nbytes);
    void setContCarrier(bool active);

    void sendPacket(
        const uint8_t* data,
        uint32_t length,
        Ack& result);

private:
    void open(uint32_t devid);
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



};
