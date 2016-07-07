#pragma once

#include <stdint.h>
#include <vector>

#include "ITransport.h"
#include "USBDevice.h"

class Crazyradio
  : public ITransport
  , public USBDevice
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

public:
    Crazyradio(
        uint32_t devid);

    virtual ~Crazyradio();

    static uint32_t numDevices();

    float version() const {
        return m_version;
    }

    void setChannel(
        uint8_t channel);

    uint8_t getChannel() const {
        return m_channel;
    }

    void setAddress(
        uint64_t address);

    uint64_t getAddress() const {
        return m_address;
    }

    void setDatarate(
        Datarate datarate);

    Datarate getDatarate() const {
        return m_datarate;
    }

    void setPower(
        Power power);

    void setArc(
        uint8_t arc);

    void setArdTime(
        uint8_t us);

    void setArdBytes(
        uint8_t nbytes);

    void setAckEnable(
        bool enable);

    void setContCarrier(
        bool active);

    virtual void sendPacket(
        const uint8_t* data,
        uint32_t length,
        ITransport::Ack& result);

    virtual void sendPacketNoAck(
        const uint8_t* data,
        uint32_t length);

private:
    uint8_t m_channel;
    uint64_t m_address;
    Datarate m_datarate;
};
