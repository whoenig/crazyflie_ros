#pragma once

#include <stdint.h>
#include <vector>
#include <string>

#include "ITransport.h"
#include "USBDevice.h"

class CrazyflieUSB
  : public ITransport
  , public USBDevice
{
public:
  CrazyflieUSB(
      uint32_t devid);

  ~CrazyflieUSB();

  static uint32_t numDevices();

  std::string serial();

  float version() const;

  virtual void sendPacket(
    const uint8_t* data,
    uint32_t length,
    ITransport::Ack& result);

  virtual void sendPacketNoAck(
    const uint8_t* data,
    uint32_t length);

private:
  void setCrtpToUsb(bool crtpToUsb);

};
