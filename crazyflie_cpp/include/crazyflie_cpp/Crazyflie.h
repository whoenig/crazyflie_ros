#pragma once

#include <cstring>

#include "Crazyradio.h"
#include "crtp.h"
#include <list>
#include <set>
#include <map>
#include <iostream>

class Crazyflie
{
public:
  enum ParamType {
    ParamTypeUint8  = 0x00 | (0x00<<2) | (0x01<<3),
    ParamTypeInt8   = 0x00 | (0x00<<2) | (0x00<<3),
    ParamTypeUint16 = 0x01 | (0x00<<2) | (0x01<<3),
    ParamTypeInt16  = 0x01 | (0x00<<2) | (0x00<<3),
    ParamTypeUint32 = 0x02 | (0x00<<2) | (0x01<<3),
    ParamTypeInt32  = 0x02 | (0x00<<2) | (0x00<<3),
    ParamTypeFloat  = 0x02 | (0x01<<2) | (0x00<<3),
  };

  struct ParamTocEntry {
    uint8_t id;
    ParamType type;
    bool readonly;
    // ParamLength length;
    // ParamType type;
    // ParamSign sign;
    // bool readonly;
    // ParamGroup paramGroup;
    std::string group;
    std::string name;
  };

  union ParamValue{
    uint8_t valueUint8;
    int8_t valueInt8;
    uint16_t valueUint16;
    int16_t valueInt16;
    uint32_t valueUint32;
    int32_t valueInt32;
    float valueFloat;
  };

  enum LogType {
    LogTypeUint8  = 1,
    LogTypeUint16 = 2,
    LogTypeUint32 = 3,
    LogTypeInt8   = 4,
    LogTypeInt16  = 5,
    LogTypeInt32  = 6,
    LogTypeFloat  = 7,
    LogTypeFP16   = 8,
  };

  struct LogTocEntry {
    uint8_t id;
    LogType type;
    std::string group;
    std::string name;
  };

public:
  Crazyflie(
    const std::string& link_uri);

  void logReset();

  void sendSetpoint(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust);

  void sendExternalPositionUpdate(
    float x,
    float y,
    float z);

  void sendPing();

  void reboot();
  void rebootToBootloader();

  void requestLogToc();

  void requestParamToc();

  std::vector<ParamTocEntry>::const_iterator paramsBegin() const {
    return m_paramTocEntries.begin();
  }
  std::vector<ParamTocEntry>::const_iterator paramsEnd() const {
    return m_paramTocEntries.end();
  }

  std::vector<LogTocEntry>::const_iterator logVariablesBegin() const {
    return m_logTocEntries.begin();
  }
  std::vector<LogTocEntry>::const_iterator logVariablesEnd() const {
    return m_logTocEntries.end();
  }

  template<class T>
  void setParam(uint8_t id, const T& value) {
    ParamValue v;
    memcpy(&v, &value, sizeof(value));
    setParam(id, v);
  }

  template<class T>
  T getParam(uint8_t id) const {
    ParamValue v = getParam(id);
    return *(reinterpret_cast<T*>(&v));
  }

  const ParamTocEntry* getParamTocEntry(
    const std::string& group,
    const std::string& name) const;

  void setEmptyAckCallback(
    std::function<void(const crtpPlatformRSSIAck*)> cb) {
    m_emptyAckCallback = cb;
  }

  void setLinkQualityCallback(
    std::function<void(float)> cb) {
    m_linkQualityCallback = cb;
  }

  static size_t size(LogType t) {
    switch(t) {
      case LogTypeUint8:
      case LogTypeInt8:
        return 1;
        break;
      case LogTypeUint16:
      case LogTypeInt16:
      case LogTypeFP16:
        return 2;
        break;
      case LogTypeUint32:
      case LogTypeInt32:
      case LogTypeFloat:
      return 4;
      break;
    }
  }

protected:
  bool sendPacket(
    const uint8_t* data,
    uint32_t length);

  void handleAck(
    const Crazyradio::Ack& result);

private:
  struct logInfo {
    uint8_t len;
    uint32_t log_crc;
    uint8_t log_max_packet;
    uint8_t log_max_ops;
  };

  /////////

  struct paramInfo {
    uint8_t len;
    uint32_t crc;
  };

  // enum ParamLength {
  //   ParamLength1Byte  = 0,
  //   ParamLength2Bytes = 1,
  //   ParamLength3Bytes = 2,
  //   ParamLength4Bytes = 3,
  // };

  // enum ParamType {
  //   ParamTypeInt   = 0,
  //   ParamTypeFloat = 1,
  // };

  // enum ParamSign {
  //   ParamSignSigned   = 0,
  //   ParamSignUnsigned = 1,
  // };

  // enum ParamGroup {
  //   ParamGroupVariable = 0,
  //   ParamGroupGroup    = 1,
  // };


private:
  const LogTocEntry* getLogTocEntry(
    const std::string& group,
    const std::string& name) const;

  uint8_t registerLogBlock(
    std::function<void(crtpLogDataResponse*, uint8_t)> cb);

  bool unregisterLogBlock(
    uint8_t id);

  void setParam(uint8_t id, const ParamValue& value);
  const ParamValue& getParam(uint8_t id) const {
    return m_paramValues.at(id);
  }

private:
  Crazyradio* m_radio;
  int m_devId;

  uint8_t m_channel;
  uint64_t m_address;
  Crazyradio::Datarate m_datarate;

  logInfo m_logInfo;
  std::vector<LogTocEntry> m_logTocEntries;
  std::set<size_t> m_logTocEntriesRequested;

  std::map<uint8_t, std::function<void(crtpLogDataResponse*, uint8_t)> > m_logBlockCb;

  bool m_blockReset;
  std::set<uint8_t> m_blockCreated;
  std::set<uint8_t> m_blockStarted;
  std::set<uint8_t> m_blockStopped;

  paramInfo m_paramInfo;
  std::vector<ParamTocEntry> m_paramTocEntries;
  std::set<size_t> m_paramTocEntriesRequested;
  std::map<uint8_t, ParamValue> m_paramValues;
  std::set<size_t> m_paramValuesRequested;

  std::function<void(const crtpPlatformRSSIAck*)> m_emptyAckCallback;
  std::function<void(float)> m_linkQualityCallback;

  template<typename T>
  friend class LogBlock;
  friend class LogBlockGeneric;
};

template<class T>
class LogBlock
{
public:
  LogBlock(
    Crazyflie* cf,
    std::list<std::pair<std::string, std::string> > variables,
    std::function<void(uint32_t, T*)>& callback)
    : m_cf(cf)
    , m_callback(callback)
    , m_id(0)
  {
    m_id = m_cf->registerLogBlock([=](crtpLogDataResponse* r, uint8_t s) { this->handleData(r, s);});
    crtpLogCreateBlockRequest request;
    request.id = m_id;
    int i = 0;
    for (auto&& pair : variables) {
      const Crazyflie::LogTocEntry* entry = m_cf->getLogTocEntry(pair.first, pair.second);
      if (entry) {
        request.items[i].logType = entry->type;
        request.items[i].id = entry->id;
        ++i;
      }
      else {
        std::cerr << "Could not find " << pair.first << "." << pair.second << " in log toc!" << std::endl;
      }
    }
    m_cf->m_blockCreated.clear();
    do
    {
      m_cf->sendPacket((const uint8_t*)&request, 3 + 2*i);
    } while(m_cf->m_blockCreated.find(m_id) == m_cf->m_blockCreated.end());
  }

  ~LogBlock()
  {
    stop();
    m_cf->unregisterLogBlock(m_id);
  }


  void start(uint8_t period)
  {
    crtpLogStartRequest request(m_id, period);
    while (m_cf->m_blockStarted.find(m_id) == m_cf->m_blockStarted.end()) {
      m_cf->sendPacket((const uint8_t*)&request, sizeof(request));
    }
    m_cf->m_blockStopped.erase(m_id);
  }

  void stop()
  {
    int timeout = 50;
    crtpLogStopRequest request(m_id);
    while (m_cf->m_blockStopped.find(m_id) == m_cf->m_blockStopped.end()) {
      m_cf->sendPacket((const uint8_t*)&request, sizeof(request));
      if (!timeout--) {
        break;
      }
    }
    m_cf->m_blockStarted.erase(m_id);
  }

private:
  void handleData(crtpLogDataResponse* response, uint8_t size) {
    if (size == sizeof(T)) {
      uint32_t time_in_ms = ((uint32_t)response->timestampHi << 8) | (response->timestampLo);
      T* t = (T*)response->data;
      m_callback(time_in_ms, t);
    }
    else {
      std::cerr << "Size doesn't match!" << std::endl;
    }
  }

private:
  Crazyflie* m_cf;
  std::function<void(uint32_t, T*)> m_callback;
  uint8_t m_id;
};

///

class LogBlockGeneric
{
public:
  LogBlockGeneric(
    Crazyflie* cf,
    const std::vector<std::string>& variables,
    void* userData,
    std::function<void(uint32_t, std::vector<double>*, void* userData)>& callback)
    : m_cf(cf)
    , m_userData(userData)
    , m_callback(callback)
    , m_id(0)
  {
    m_id = m_cf->registerLogBlock([=](crtpLogDataResponse* r, uint8_t s) { this->handleData(r, s);});
    crtpLogCreateBlockRequest request;
    request.id = m_id;
    int i = 0;
    size_t s = 0;
    for (auto&& var : variables) {
      auto pos = var.find(".");
      std::string first = var.substr(0, pos);
      std::string second = var.substr(pos+1);
      const Crazyflie::LogTocEntry* entry = m_cf->getLogTocEntry(first, second);
      if (entry) {
        s += Crazyflie::size(entry->type);
        if (s > 26) {
          std::cerr << "Can't configure that many variables in a single log block!"
                    << " Ignoring " << first << "." << second << std::endl;
        } else {
          request.items[i].logType = entry->type;
          request.items[i].id = entry->id;
          ++i;
          m_types.push_back(entry->type);
        }
      }
      else {
        std::cerr << "Could not find " << first << "." << second << " in log toc!" << std::endl;
      }
    }
    m_cf->m_blockCreated.clear();
    do
    {
      m_cf->sendPacket((const uint8_t*)&request, 3 + 2*i);
    } while(m_cf->m_blockCreated.find(m_id) == m_cf->m_blockCreated.end());
  }

  ~LogBlockGeneric()
  {
    stop();
    m_cf->unregisterLogBlock(m_id);
  }


  void start(uint8_t period)
  {
    crtpLogStartRequest request(m_id, period);
    while (m_cf->m_blockStarted.find(m_id) == m_cf->m_blockStarted.end()) {
      m_cf->sendPacket((const uint8_t*)&request, sizeof(request));
    }
    m_cf->m_blockStopped.erase(m_id);
  }

  void stop()
  {
    int timeout = 50;
    crtpLogStopRequest request(m_id);
    while (m_cf->m_blockStopped.find(m_id) == m_cf->m_blockStopped.end()) {
      m_cf->sendPacket((const uint8_t*)&request, sizeof(request));
      if (!timeout--) {
        break;
      }
    }
    m_cf->m_blockStarted.erase(m_id);
  }

private:
  void handleData(crtpLogDataResponse* response, uint8_t size) {

    std::vector<double> result;
    size_t pos = 0;
    for (size_t i = 0; i < m_types.size(); ++i)
    {
      switch (m_types[i])
      {
      case Crazyflie::LogTypeUint8:
        {
          uint8_t value;
          memcpy(&value, &response->data[pos], sizeof(uint8_t));
          result.push_back(value);
          pos += sizeof(uint8_t);
          break;
        }
      case Crazyflie::LogTypeInt8:
        {
          int8_t value;
          memcpy(&value, &response->data[pos], sizeof(int8_t));
          result.push_back(value);
          pos += sizeof(int8_t);
          break;
        }
      case Crazyflie::LogTypeUint16:
        {
          uint16_t value;
          memcpy(&value, &response->data[pos], sizeof(uint16_t));
          result.push_back(value);
          pos += sizeof(uint16_t);
          break;
        }
      case Crazyflie::LogTypeInt16:
        {
          int16_t value;
          memcpy(&value, &response->data[pos], sizeof(int16_t));
          result.push_back(value);
          pos += sizeof(int16_t);
          break;
        }
      case Crazyflie::LogTypeUint32:
        {
          uint32_t value;
          memcpy(&value, &response->data[pos], sizeof(uint32_t));
          result.push_back(value);
          pos += sizeof(uint32_t);
          break;
        }
      case Crazyflie::LogTypeInt32:
        {
          int32_t value;
          memcpy(&value, &response->data[pos], sizeof(int32_t));
          result.push_back(value);
          pos += sizeof(int32_t);
          break;
        }
      case Crazyflie::LogTypeFloat:
        {
          float value;
          memcpy(&value, &response->data[pos], sizeof(float));
          result.push_back(value);
          pos += sizeof(float);
          break;
        }
      case Crazyflie::LogTypeFP16:
        {
          double value;
          memcpy(&value, &response->data[pos], sizeof(double));
          result.push_back(value);
          pos += sizeof(double);
          break;
        }
      }
    }

    uint32_t time_in_ms = ((uint32_t)response->timestampHi << 8) | (response->timestampLo);
    m_callback(time_in_ms, &result, m_userData);
  }

private:
  Crazyflie* m_cf;
  void* m_userData;
  std::function<void(uint32_t, std::vector<double>*, void*)> m_callback;
  uint8_t m_id;
  std::vector<Crazyflie::LogType> m_types;
};
