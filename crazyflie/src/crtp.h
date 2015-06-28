#pragma once

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

struct crtpConsoleResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 1 &&
             crtp(response.data[0]) == crtp(0, 0);
    }

    crtp header;
    char text[31];
};

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


////////

  // struct logGetInfoRequest
  // {
  //   logGetInfoRequest()
  //     : header(5, 0)
  //     , command(1)
  //     {
  //     }

  //     const crtp header;
  //     const uint8_t command;
  // } __attribute__((packed));

  // struct logGetInfoResponse
  // {
  //     static bool match(const Crazyradio::Ack& response) {
  //       return response.ack &&
  //              response.size == 9 &&
  //              crtp(response.data[0]) == crtp(5, 0) &&
  //              response.data[1] == 1;
  //     }

  //     logGetInfoResponse(const Crazyradio::Ack& response) {
  //       memcpy(this, &response.data[2], sizeof(this));
  //     }

  //     // Number of log items contained in the log table of content
  //     uint8_t log_len;
  //     // CRC values of the log TOC memory content. This is a fingerprint of the copter build that can be used to cache the TOC
  //     uint32_t log_crc;
  //     // Maximum number of log packets that can be programmed in the copter
  //     uint8_t log_max_packet;
  //     // Maximum number of operation programmable in the copter. An operation is one log variable retrieval programming
  //     uint8_t log_max_ops;
  // } __attribute__((packed));

  // struct logGetItemRequest
  // {
  //   logGetItemRequest(uint8_t id)
  //     : header(5, 0)
  //     , command(0)
  //     , id(id)
  //     {
  //     }

  //     const crtp header;
  //     const uint8_t command;
  //     uint8_t id;
  // } __attribute__((packed));

  // struct logGetItemResponse
  // {
  //     static bool match(const Crazyradio::Ack& response) {
  //       return response.ack &&
  //              response.size > 5 &&
  //              crtp(response.data[0]) == crtp(5, 0) &&
  //              response.data[1] == 0;
  //     }

  //     logGetItemResponse(const Crazyradio::Ack& response) {
  //       id = response.data[2];
  //       type = response.data[3];
  //       group = std::string((const char*)&response.data[4]);
  //       name = std::string((const char*)&response.data[5 + group.size()]);
  //     }

  //     uint8_t id;
  //     uint8_t type;
  //     std::string group;
  //     std::string name;
  // };

  // struct logBlockItem {
  //   uint8_t logType;
  //   uint8_t id;
  // } __attribute__((packed));

  // struct logCreateBlockRequest
  // {
  //   logCreateBlockRequest()
  //     : header(5, 1)
  //     , command(0)
  //     {
  //     }

  //     const crtp header;
  //     const uint8_t command;
  //     uint8_t id;
  //     logBlockItem items[16];
  // } __attribute__((packed));

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

  // struct logStartRequest
  // {
  //   logStartRequest()
  //     : header(5, 1)
  //     , command(3)
  //     {
  //     }

  //     const crtp header;
  //     const uint8_t command;
  //     uint8_t id;
  //     uint8_t period; // in increments of 10ms
  // } __attribute__((packed));

  // struct logStopRequest
  // {
  //   logStopRequest()
  //     : header(5, 1)
  //     , command(4)
  //     {
  //     }

  //     const crtp header;
  //     const uint8_t command;
  //     uint8_t id;
  // } __attribute__((packed));

  // struct logResetRequest
  // {
  //   logResetRequest()
  //     : header(5, 1)
  //     , command(5)
  //     {
  //     }

  //     const crtp header;
  //     const uint8_t command;
  // } __attribute__((packed));

  // enum LogType {
  //   LogTypeUint8  = 1,
  //   LogTypeUint16 = 2,
  //   LogTypeUint32 = 3,
  //   LogTypeInt8   = 4,
  //   LogTypeInt16  = 5,
  //   LogTypeInt32  = 6,
  //   LogTypeFloat  = 7,
  //   LogTypeFP16   = 8,
  // };

  // struct consoleResponse
  // {
  //     static bool match(const Crazyradio::Ack& response) {
  //       return response.ack &&
  //              response.size > 1 &&
  //              crtp(response.data[0]) == crtp(0, 0);
  //     }

  //     consoleResponse(const Crazyradio::Ack& response) {
  //       text = std::string((const char*)&response.data[1]);
  //     }

  //     std::string text;
  // };
