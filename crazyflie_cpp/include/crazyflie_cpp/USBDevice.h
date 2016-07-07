#pragma once

#include <stdint.h>

// forward declarations
struct libusb_context;
struct libusb_device_handle;

class USBDevice
{
public:
  USBDevice(
    uint16_t idVendor,
    uint16_t idProduct);

  virtual ~USBDevice();

protected:
    static uint32_t numDevices(
      uint16_t idVendor,
      uint16_t idProduct);

    void open(uint32_t devid);

    void sendVendorSetup(
        uint8_t request,
        uint16_t value,
        uint16_t index,
        const unsigned char* data,
        uint16_t length);

protected:
    libusb_context* m_ctx;
    libusb_device_handle *m_handle;

    float m_version;

private:
  uint16_t m_idVendor;
  uint16_t m_idProduct;
};
