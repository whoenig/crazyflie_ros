#include "ros/ros.h"
#include "crazyflie/AddCrazyflie.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

//#include <regex>
#include <thread>
#include <mutex>

#include "Crazyflie.h"

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging)
    : m_cf(link_uri)
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_serviceEmergency()
    , m_subscribeCmdVel()
  {
    ros::NodeHandle n;
    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);

    std::thread t(&CrazyflieROS::run, this);
    t.detach();
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
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawrate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
    }
  }

  void run()
  {

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // sendSetpoint();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

  }

private:
  Crazyflie m_cf;
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
