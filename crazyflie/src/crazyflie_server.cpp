#include "ros/ros.h"
#include "crazyflie/AddCrazyflie.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

//#include <regex>
#include <thread>
#include <mutex>

#include "Crazyradio.h"

#define MAX_RADIOS 4

Crazyradio* g_crazyradios[MAX_RADIOS];
std::mutex g_mutex[MAX_RADIOS];

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging)
    : m_radio(NULL)
    , m_setpoint()
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_serviceEmergency()
    , m_subscribeCmdVel()
  {
    // std::regex re("^radio://([0-9]+)((/([0-9]+))((/(250K|1M|2M))?(/([A-F0-9]+))?)?)?$");
    // std::smatch match;
    // if (std::regex_search(link_uri, match, re)) {
    //   std::cout << match.str(1);
    // }

    ros::NodeHandle n;
    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);

    int devId;
    int channel;
    int datarate;
    char datarateType;
    uint64_t address;

    if(std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
       &devId, &channel, &datarate,
       &datarateType, &address) != EOF) {
      Crazyradio::Datarate dr;
      if (datarate == 250 && datarateType == 'K') {
        dr = Crazyradio::Datarate_250KPS;
      }
      else if (datarate == 1 && datarateType == 'M') {
        dr = Crazyradio::Datarate_1MPS;
      }
      else if (datarate == 2 && datarateType == 'M') {
        dr = Crazyradio::Datarate_2MPS;
      }

      if (!g_crazyradios[devId]) {
        g_crazyradios[devId] = new Crazyradio(devId);
      }

      m_radio = g_crazyradios[devId];

      std::thread t(&CrazyflieROS::run, this, devId, channel, dr, address);
      t.detach();

    }
  }

private:

  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      m_setpoint.roll = msg->linear.y + m_roll_trim;
      m_setpoint.pitch = - (msg->linear.x + m_pitch_trim);
      m_setpoint.yawrate = msg->angular.z;
      m_setpoint.thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);
    }
  }

  void sendSetpoint(int devId, uint64_t address) {
    std::unique_lock<std::mutex> mlock(g_mutex[devId]);
    m_radio->setAddress(address);
    Crazyradio::Ack result;
    m_radio->sendPacket((const uint8_t*)&m_setpoint, sizeof(m_setpoint), result);
  }

  void run(int devId, int channel, Crazyradio::Datarate dr, uint64_t address)
  {

    m_radio->setChannel(channel);
    m_radio->setDatarate(dr);
    m_radio->setAddress(address);

    m_setpoint.link = 3;
    m_setpoint.port = 0x03;

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
      sendSetpoint(devId, address);
    }

    while(!m_isEmergency) {
      sendSetpoint(devId, address);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Make sure we turn the engines off
    m_setpoint.thrust = 0;
    for (int i = 0; i < 100; ++i) {
      sendSetpoint(devId, address);
    }

  }

private:
  struct setpoint
  {
      uint8_t channel:2;
      uint8_t link:2;
      uint8_t port:4;
      float roll;
      float pitch;
      float yawrate;
      uint16_t thrust;
  }  __attribute__((packed));


private:
  Crazyradio* m_radio;
  setpoint m_setpoint;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;

  ros::ServiceServer m_serviceEmergency;
  ros::Subscriber m_subscribeCmdVel;
};

bool add_crazyflie(
  crazyflie::AddCrazyflie::Request  &req,
  crazyflie::AddCrazyflie::Response &res)
{
  ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging);

  // Leak intentionally
  CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
