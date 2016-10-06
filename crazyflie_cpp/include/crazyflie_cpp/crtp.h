#pragma once

// Header
struct crtp
{
  constexpr crtp(uint8_t port, uint8_t channel)
    : channel(channel)
    , link(3)
    , port(port)
  {
  }

  crtp(uint8_t byte)
  {
    channel = (byte >> 0) & 0x3;
    link    = (byte >> 2) & 0x3;
    port    = (byte >> 4) & 0xF;
  }

  bool operator==(const crtp& other) {
    return channel == other.channel && port == other.port;
  }

  uint8_t channel:2;
  uint8_t link:2;
  uint8_t port:4;
} __attribute__((packed));

// Port 0 (Console)
struct crtpConsoleResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return crtp(response.data[0]) == crtp(0, 0);
    }

    crtp header;
    char text[31];
};

// Port 2 (Parameters)

struct crtpParamTocGetItemRequest
{
  crtpParamTocGetItemRequest(
    uint8_t id)
    : header(2, 0)
    , command(0)
    , id(id)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
} __attribute__((packed));

struct crtpParamTocGetItemResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 5 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 0;
  }

  crtpParamTocGetItemRequest request;
  uint8_t length:2; // one of ParamLength
  uint8_t type:1;   // one of ParamType
  uint8_t sign:1;   // one of ParamSign
  uint8_t res0:2;   // reserved
  uint8_t readonly:1;
  uint8_t group:1;  // one of ParamGroup
  char text[28]; // group, name
} __attribute__((packed));

struct crtpParamTocGetInfoRequest
{
  crtpParamTocGetInfoRequest()
    : header(2, 0)
    , command(1)
    {
    }

    const crtp header;
    const uint8_t command;
} __attribute__((packed));

struct crtpParamTocGetInfoResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 7 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 1;
  }

  crtpParamTocGetInfoRequest request;
  uint8_t numParam;
  uint32_t crc;
} __attribute__((packed));

struct crtpParamReadRequest
{
  crtpParamReadRequest(
    uint8_t id)
    : header(2, 1)
    , id(id)
    {
    }

    const crtp header;
    const uint8_t id;
} __attribute__((packed));

template <class T>
struct crtpParamWriteRequest
{
  crtpParamWriteRequest(
    uint8_t id,
    const T& value)
    : header(2, 2)
    , id(id)
    , value(value)
    {
    }

    const crtp header;
    const uint8_t id;
    const T value;
} __attribute__((packed));

struct crtpParamValueResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 2 &&
           (crtp(response.data[0]) == crtp(2, 1) ||
            crtp(response.data[0]) == crtp(2, 2));
  }

  crtp header;
  uint8_t id;
  union {
    uint8_t valueUint8;
    int8_t valueInt8;
    uint16_t valueUint16;
    int16_t valueInt16;
    uint32_t valueUint32;
    int32_t valueInt32;
    float valueFloat;
  };
} __attribute__((packed));

// Port 3 (Commander)

struct crtpSetpointRequest
{
  crtpSetpointRequest(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust)
    : header(0x03, 0)
    , roll(roll)
    , pitch(pitch)
    , yawrate(yawrate)
    , thrust(thrust)
  {
  }
  const crtp header;
  float roll;
  float pitch;
  float yawrate;
  uint16_t thrust;
}  __attribute__((packed));

// Port 4 (Memory access)

// Port 5 (Data logging)

struct crtpLogGetInfoRequest
{
  crtpLogGetInfoRequest()
    : header(5, 0)
    , command(1)
    {
    }

    const crtp header;
    const uint8_t command;
} __attribute__((packed));

struct crtpLogGetInfoResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 9 &&
           crtp(response.data[0]) == crtp(5, 0) &&
           response.data[1] == 1;
  }

  crtpLogGetInfoRequest request;
  // Number of log items contained in the log table of content
  uint8_t log_len;
  // CRC values of the log TOC memory content. This is a fingerprint of the copter build that can be used to cache the TOC
  uint32_t log_crc;
  // Maximum number of log packets that can be programmed in the copter
  uint8_t log_max_packet;
  // Maximum number of operation programmable in the copter. An operation is one log variable retrieval programming
  uint8_t log_max_ops;
} __attribute__((packed));

struct crtpLogGetItemRequest
{
  crtpLogGetItemRequest(uint8_t id)
    : header(5, 0)
    , command(0)
    , id(id)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
} __attribute__((packed));

struct crtpLogGetItemResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 5 &&
             crtp(response.data[0]) == crtp(5, 0) &&
             response.data[1] == 0;
    }

    crtpLogGetItemRequest request;
    uint8_t type;
    char text[28]; // group, name
} __attribute__((packed));

struct logBlockItem {
  uint8_t logType;
  uint8_t id;
} __attribute__((packed));

struct crtpLogCreateBlockRequest
{
  crtpLogCreateBlockRequest()
  : header(5, 1)
  , command(0)
  {
  }

  const crtp header;
  const uint8_t command;
  uint8_t id;
  logBlockItem items[16];
} __attribute__((packed));

// struct logAppendBlockRequest
// {
//   logAppendBlockRequest()
//     : header(5, 1)
//     , command(1)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     uint8_t id;
//     logBlockItem items[16];
// } __attribute__((packed));

// struct logDeleteBlockRequest
// {
//   logDeleteBlockRequest()
//     : header(5, 1)
//     , command(2)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     uint8_t id;
// } __attribute__((packed));

struct crtpLogStartRequest
{
  crtpLogStartRequest(
    uint8_t id,
    uint8_t period)
    : header(5, 1)
    , command(3)
    , id(id)
    , period(period)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
    uint8_t period; // in increments of 10ms
} __attribute__((packed));

struct crtpLogStopRequest
{
  crtpLogStopRequest(
    uint8_t id)
    : header(5, 1)
    , command(4)
    , id(id)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
} __attribute__((packed));

struct crtpLogResetRequest
{
  crtpLogResetRequest()
    : header(5, 1)
    , command(5)
    {
    }

    const crtp header;
    const uint8_t command;
} __attribute__((packed));

enum crtpLogControlResult {
  crtpLogControlResultOk            = 0,
  crtpLogControlResultOutOfMemory   = 12, // ENOMEM
  crtpLogControlResultCmdNotFound   = 8,  // ENOEXEC
  crtpLogControlResultWrongBlockId  = 2,  // ENOENT
  crtpLogControlResultBlockTooLarge = 7,  // E2BIG
  crtpLogControlResultBlockExists   = 17, // EEXIST

};

struct crtpLogControlResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size == 4 &&
             crtp(response.data[0]) == crtp(5, 1);
    }

    crtp header;
    uint8_t command;
    uint8_t requestByte1;
    uint8_t result; // one of crtpLogControlResult
} __attribute__((packed));

struct crtpLogDataResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 4 &&
             crtp(response.data[0]) == crtp(5, 2);
    }

    crtp header;
    uint8_t blockId;
    uint8_t timestampLo;
    uint16_t timestampHi;
    uint8_t data[26];
} __attribute__((packed));


// Port 0x06 (External Position Update)

struct crtpExternalPositionUpdate
{
  crtpExternalPositionUpdate(
    float x,
    float y,
    float z)
    : header(0x06, 0)
    , x(x)
    , y(y)
    , z(z)
  {
  }
  const crtp header;
  float x;
  float y;
  float z;
}  __attribute__((packed));



// Port 13 (Platform)

// The crazyflie-nrf firmware sends empty packets with the signal strength, if nothing else is in the queue
struct crtpPlatformRSSIAck
{
    static bool match(const Crazyradio::Ack& response) {
      return crtp(response.data[0]) == crtp(15, 3);
    }

    crtp header;
    uint8_t reserved;
    uint8_t rssi;
};
