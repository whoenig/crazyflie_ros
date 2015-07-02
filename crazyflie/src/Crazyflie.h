#pragma once

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


public:
  Crazyflie(
    const std::string& link_uri);

  void sendSetpoint(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust);

  void sendPing();

  void reboot();

  void requestLogToc();

  void requestParamToc();

  std::vector<ParamTocEntry>::const_iterator paramsBegin() const {
    return m_paramTocEntries.begin();
  }
  std::vector<ParamTocEntry>::const_iterator paramsEnd() const {
    return m_paramTocEntries.end();
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

protected:
  void sendPacket(
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

  std::map<uint8_t, std::function<void(crtpLogDataResponse*, uint8_t)> > m_logBlockCb;

  bool m_blockReset;
  std::set<uint8_t> m_blockCreated;
  std::set<uint8_t> m_blockStarted;
  std::set<uint8_t> m_blockStopped;

  paramInfo m_paramInfo;
  std::vector<ParamTocEntry> m_paramTocEntries;
  std::map<uint8_t, ParamValue> m_paramValues;

  template<typename T>
  friend class LogBlock;
};

template<class T>
class LogBlock
{
public:
  LogBlock(
    Crazyflie* cf,
    std::list<std::pair<std::string, std::string> > variables,
    std::function<void(T*)>& callback)
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
    crtpLogStopRequest request(m_id);
    while (m_cf->m_blockStopped.find(m_id) == m_cf->m_blockStopped.end()) {
      m_cf->sendPacket((const uint8_t*)&request, sizeof(request));
    }
    m_cf->m_blockStarted.erase(m_id);
  }

private:
  void handleData(crtpLogDataResponse* response, uint8_t size) {
    if (size == sizeof(T)) {
      T* t = (T*)response->data;
      m_callback(t);
    }
    else {
      std::cerr << "Size doesn't match!" << std::endl;
    }
  }

private:
  Crazyflie* m_cf;
  std::function<void(T*)> m_callback;
  uint8_t m_id;
};
