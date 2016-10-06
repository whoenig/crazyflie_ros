//#include <regex>
#include <mutex>

#include "Crazyflie.h"
#include "crtp.h"

#include "Crazyradio.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <thread>

#define MAX_RADIOS 16

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
  , m_logTocEntriesRequested()
  , m_logBlockCb()
  , m_blockReset(false)
  , m_blockCreated()
  , m_blockStarted()
  , m_blockStopped()
  , m_paramInfo()
  , m_paramTocEntries()
  , m_paramTocEntriesRequested()
  , m_paramValues()
  , m_paramValuesRequested()
  , m_emptyAckCallback(nullptr)
  , m_linkQualityCallback(nullptr)
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

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_mutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId] = new Crazyradio(m_devId);
        // g_crazyradios[m_devId]->setAckEnable(false);
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    throw std::runtime_error("Uri is not valid!");
  }
}

void Crazyflie::logReset()
{
  m_blockReset = false;
  do {
    crtpLogResetRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

void Crazyflie::sendExternalPositionUpdate(
  float x,
  float y,
  float z)
{
  crtpExternalPositionUpdate position(x, y, z);
  sendPacket((const uint8_t*)&position, sizeof(position));
}

void Crazyflie::sendPing()
{
  uint8_t ping = 0xFF;
  sendPacket(&ping, sizeof(ping));
}

// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  while(!sendPacket(reboot_init, sizeof(reboot_init))) {}

  const uint8_t reboot_to_firmware[] = {0xFF, 0xFE, 0xF0, 0x01};
  while(!sendPacket(reboot_to_firmware, sizeof(reboot_to_firmware))) {}
}

void Crazyflie::rebootToBootloader()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  while(!sendPacket(reboot_init, sizeof(reboot_init))) {}

  const uint8_t reboot_to_bootloader[] = {0xFF, 0xFE, 0xF0, 0x00};
  while(!sendPacket(reboot_to_bootloader, sizeof(reboot_to_bootloader))) {}
}

void Crazyflie::requestLogToc()
{

  // Find the number of log variables in TOC
  m_logInfo.len = 0;
  do
  {
    crtpLogGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_logInfo.len == 0);
  std::cout << "Log: " << (int)m_logInfo.len << std::endl;

  // Prepare data structures to request detailed information
  m_logTocEntriesRequested.clear();
  m_logTocEntries.resize(m_logInfo.len);
  for (size_t i = 0; i < m_logInfo.len; ++i)
  {
    m_logTocEntriesRequested.insert(i);
  }

  // Request detailed information, until done
  while (m_logTocEntriesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_logTocEntriesRequested.size(); ++p)
    {
      auto iter = m_logTocEntriesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpLogGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }
}

void Crazyflie::requestParamToc()
{
  // Find the number of log variables in TOC
  m_paramInfo.len = 0;
  do
  {
    crtpParamTocGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_paramInfo.len == 0);
  std::cout << "Params: " << (int)m_paramInfo.len << std::endl;

  // Prepare data structures to request detailed information
  m_paramTocEntriesRequested.clear();
  m_paramValues.clear();
  m_paramTocEntries.resize(m_paramInfo.len);
  for (size_t i = 0; i < m_paramInfo.len; ++i)
  {
    m_paramTocEntriesRequested.insert(i);
    m_paramValuesRequested.insert(i);
  }

  // Request detailed information, until done
  while (m_paramTocEntriesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_paramTocEntriesRequested.size(); ++p)
    {
      auto iter = m_paramTocEntriesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpParamTocGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }

  // Request current values
  while (m_paramValuesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_paramValuesRequested.size(); ++p)
    {
      auto iter = m_paramValuesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpParamReadRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }
}

void Crazyflie::setParam(uint8_t id, const ParamValue& value) {

  m_paramValues.erase(id);
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      do
      {
        switch (entry.type) {
          case ParamTypeUint8:
            {
              crtpParamWriteRequest<uint8_t> request(id, value.valueUint8);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt8:
            {
              crtpParamWriteRequest<int8_t> request(id, value.valueInt8);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeUint16:
            {
              crtpParamWriteRequest<uint16_t> request(id, value.valueUint16);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt16:
            {
              crtpParamWriteRequest<int16_t> request(id, value.valueInt16);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeUint32:
            {
              crtpParamWriteRequest<uint32_t> request(id, value.valueUint32);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt32:
            {
              crtpParamWriteRequest<int32_t> request(id, value.valueInt32);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeFloat:
            {
              crtpParamWriteRequest<float> request(id, value.valueFloat);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
        }
      } while(m_paramValues.find(id) == m_paramValues.end());
      break;
    }
  }


}

bool Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  static uint32_t numPackets = 0;
  static uint32_t numAcks = 0;

  numPackets++;

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
    numAcks++;
  }
  if (numPackets == 100) {
    if (m_linkQualityCallback) {
      // We just take the ratio of sent vs. acked packets here
      // for a sliding window of 100 packets
      float linkQuality = numAcks / (float)numPackets;
      m_linkQualityCallback(linkQuality);
    }
    numPackets = 0;
    numAcks = 0;
  }
  return ack.ack;
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
    if (r->request.id < m_logTocEntries.size())
    {
      m_logTocEntriesRequested.erase(r->request.id);
      LogTocEntry& entry = m_logTocEntries[r->request.id];
      entry.id = r->request.id;
      entry.type = (LogType)r->type;
      entry.group = std::string(&r->text[0]);
      entry.name = std::string(&r->text[entry.group.size() + 1]);
    }
    // std::cout << "Got: " << (int)r->id << std::endl;
  }
  else if (crtpLogControlResponse::match(result)) {
    crtpLogControlResponse* r = (crtpLogControlResponse*)result.data;
    if (r->command == 0 &&
        (r->result == crtpLogControlResultOk || r->result == crtpLogControlResultBlockExists)) {
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
    else if (m_blockReset) {
      std::cout << "Received unrequested data for block: " << (int)r->blockId << std::endl;
    }
  }
  else if (crtpParamTocGetInfoResponse::match(result)) {
    crtpParamTocGetInfoResponse* r = (crtpParamTocGetInfoResponse*)result.data;
    m_paramInfo.len = r->numParam;
  }
  else if (crtpParamTocGetItemResponse::match(result)) {
    crtpParamTocGetItemResponse* r = (crtpParamTocGetItemResponse*)result.data;
    if (r->request.id < m_paramTocEntries.size())
    {
      m_paramTocEntriesRequested.erase(r->request.id);
      ParamTocEntry& entry = m_paramTocEntries[r->request.id];
      entry.id = r->request.id;
      entry.type = (ParamType)(r->length | r-> type << 2 | r->sign << 3);
      entry.readonly = r->readonly;
      entry.group = std::string(&r->text[0]);
      entry.name = std::string(&r->text[entry.group.size() + 1]);
    }
  }
  else if (crtpParamValueResponse::match(result)) {
    crtpParamValueResponse* r = (crtpParamValueResponse*)result.data;
    ParamValue v;
    std::memcpy(&v, &r->valueFloat, 4);
    // *v = r->valueFloat;
    m_paramValues[r->id] = v;//(ParamValue)(r->valueFloat);
    m_paramValuesRequested.erase(r->id);
  }
  else if (crtpPlatformRSSIAck::match(result)) {
    crtpPlatformRSSIAck* r = (crtpPlatformRSSIAck*)result.data;
    if (m_emptyAckCallback) {
      m_emptyAckCallback(r);
    }
  }
  else {
    crtp* header = (crtp*)result.data;
    std::cout << "Don't know ack: Port: " << (int)header->port << " Channel: " << (int)header->channel << " Len: " << (int)result.size << std::endl;
    // for (size_t i = 1; i < result.size; ++i) {
    //   std::cout << "    " << (int)result.data[i] << std::endl;
    // }
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

const Crazyflie::ParamTocEntry* Crazyflie::getParamTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_paramTocEntries) {
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

