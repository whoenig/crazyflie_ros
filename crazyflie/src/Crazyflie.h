#pragma once

#include "Crazyradio.h"

class Crazyflie
{
public:
  Crazyflie(
    const std::string& link_uri);

  void sendSetpoint(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust);

protected:
  void sendPacket(
    const uint8_t* data,
    uint32_t length);

  void handleAck(
    const Crazyradio::Ack& result);

private:
  Crazyradio* m_radio;
  int m_devId;

  uint8_t m_channel;
  uint64_t m_address;
  Crazyradio::Datarate m_datarate;
};
