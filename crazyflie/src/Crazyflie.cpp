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
  , m_logInfo()
  , m_logTocEntries()
  , m_logBlockCb()
  , m_blockReset(false)
  , m_blockCreated()
  , m_blockStarted()
  , m_blockStopped()
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
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
  else {
    std::cerr << "Uri is not valid!" << std::endl;
  }

  m_blockReset = false;
  do {
    crtpLogResetRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while (!m_blockReset);
}

void Crazyflie::sendSetpoint(
  float roll,
  float pitch,
  float yawrate,
  uint16_t thrust)
{
  crtpSetpointRequest request(roll, pitch, yawrate, thrust);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void Crazyflie::sendPing()
{
  uint8_t ping = 0xFF;
  sendPacket(&ping, sizeof(ping));
}

void Crazyflie::reboot()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  sendPacket(reboot_init, sizeof(reboot_init));

  // const uint8_t reboot_to_bootloader[] = {0xFF, 0xFE, 0xF0, 0x00};
  //

  const uint8_t reboot_to_firmware[] = {0xFF, 0xFE, 0xF0, 0x01};
  sendPacket(reboot_to_firmware, sizeof(reboot_to_firmware));
}

void Crazyflie::requestLogToc()
{
  m_logInfo.len = 0;
  m_logTocEntries.clear();
  do
  {
    crtpLogGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_logInfo.len == 0);
  std::cout << "Log: " << (int)m_logInfo.len << std::endl;

  for (size_t i = 0; i < m_logInfo.len; ++i)
  {
    // std::cout << i << std::endl;
    do
    {
      crtpLogGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    } while(m_logTocEntries.size() < i + 1);
  }

  for (auto&& entry : m_logTocEntries) {
    std::cout << entry.group << "." << entry.name << " type: " << (int)entry.type << std::endl;
  }
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
  if (ack.ack) {
    handleAck(ack);
  }
}

void Crazyflie::handleAck(
  const Crazyradio::Ack& result)
{
  if (crtpConsoleResponse::match(result)) {
    if (result.size > 0) {
      crtpConsoleResponse* r = (crtpConsoleResponse*)result.data;
      std::cout << r->text << std::endl;
    }
    // ROS_INFO("Console: %s", r->text);
  }
  else if (crtpLogGetInfoResponse::match(result)) {
    crtpLogGetInfoResponse* r = (crtpLogGetInfoResponse*)result.data;
    m_logInfo.len = r->log_len;
  }
  else if (crtpLogGetItemResponse::match(result)) {
    crtpLogGetItemResponse* r = (crtpLogGetItemResponse*)result.data;
    m_logTocEntries.resize(r->request.id + 1);
    LogTocEntry& entry = m_logTocEntries[r->request.id];
    entry.id = r->request.id;
    entry.type = (LogType)r->type;
    entry.group = std::string(&r->text[0]);
    entry.name = std::string(&r->text[entry.group.size() + 1]);
    // std::cout << "Got: " << (int)r->id << std::endl;
  }
  else if (crtpLogControlResponse::match(result)) {
    crtpLogControlResponse* r = (crtpLogControlResponse*)result.data;
    if (r->command == 0 && r->result == 0) {
      m_blockCreated.insert(r->requestByte1);
    }
    if (r->command == 3 && r->result == 0) {
      m_blockStarted.insert(r->requestByte1);
    }
    if (r->command == 4 && r->result == 0) {
      m_blockStopped.insert(r->requestByte1);
    }
    if (r->command == 5 && r->result == 0) {
      m_blockReset = true;
    }
    // std::cout << "LogControl: " << (int)r->command << " errno: " << (int)r->result << std::endl;
  }
  else if (crtpLogDataResponse::match(result)) {
    crtpLogDataResponse* r = (crtpLogDataResponse*)result.data;
    auto iter = m_logBlockCb.find(r->blockId);
    if (iter != m_logBlockCb.end()) {
      iter->second(r, result.size - 5);
    }
    else {
      std::cout << "Received unrequested data for block: " << (int)r->blockId << std::endl;
    }
  }
  else {
    crtp* header = (crtp*)result.data;
    std::cout << "Don't know ack: Port: " << (int)header->port << " Channel: " << (int)header->channel << " Len: " << (int)result.size << std::endl;
    for (size_t i = 1; i < result.size; ++i) {
      std::cout << "    " << (int)result.data[i] << std::endl;
    }
  }

}

const Crazyflie::LogTocEntry* Crazyflie::getLogTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_logTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

uint8_t Crazyflie::registerLogBlock(
  std::function<void(crtpLogDataResponse*, uint8_t)> cb)
{
  for (uint8_t id = 0; id < 255; ++id) {
    if (m_logBlockCb.find(id) == m_logBlockCb.end()) {
      m_logBlockCb[id] = cb;
      return id;
    }
  }
}

bool Crazyflie::unregisterLogBlock(
  uint8_t id)
{
  m_logBlockCb.erase(m_logBlockCb.find(id));
}

