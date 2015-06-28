//#include <regex>
#include <mutex>

#include "Crazyflie.h"
#include "crtp.h"

#include "Crazyradio.h"

#include <iostream>

#define MAX_RADIOS 4

Crazyradio* g_crazyradios[MAX_RADIOS];
std::mutex g_mutex[MAX_RADIOS];



Crazyflie::Crazyflie(
  const std::string& link_uri)
  : m_radio(NULL)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) != EOF;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) != EOF;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (!g_crazyradios[m_devId]) {
      g_crazyradios[m_devId] = new Crazyradio(m_devId);
      // g_crazyradios[m_devId]->setAckEnable(false);
      g_crazyradios[m_devId]->setAckEnable(true);
      g_crazyradios[m_devId]->setArc(0);
    }

    m_radio = g_crazyradios[m_devId];
  }
}

void Crazyflie::sendSetpoint(
  float roll,
  float pitch,
  float yawrate,
  uint16_t thrust)
{
  crtpSetpointRequest request(roll, pitch, yawrate, thrust);
  sendPacket((const uint8_t*)&request, sizeof(crtpSetpointRequest));
}

void Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  Crazyradio::Ack ack;
  {
    std::unique_lock<std::mutex> mlock(g_mutex[m_devId]);
    if (m_radio->getAddress() != m_address) {
      m_radio->setAddress(m_address);
    }
    if (m_radio->getChannel() != m_channel) {
      m_radio->setChannel(m_channel);
    }
    if (m_radio->getDatarate() != m_datarate) {
      m_radio->setDatarate(m_datarate);
    }
    m_radio->sendPacket(data, length, ack);
  }
  ack.data[ack.size] = 0;
  handleAck(ack);
}

void Crazyflie::handleAck(
  const Crazyradio::Ack& result)
{
  if (crtpConsoleResponse::match(result)) {
    crtpConsoleResponse* r = (crtpConsoleResponse*)result.data;
    std::cout << r->text << std::endl;
    // ROS_INFO("Console: %s", r->text);
  }

}
