#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/RemoveCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/sendPacket.h"
#include "crazyflie_driver/crtpPacket.h"
#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

//#include <regex>
#include <thread>
#include <mutex>

#include <string>
#include <map>

#include <crazyflie_cpp/Crazyflie.h>

constexpr double pi() { return 3.141592653589793238462643383279502884; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_parameters,
    std::vector<crazyflie_driver::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_imu,
    bool enable_logging_temperature,
    bool enable_logging_magnetic_field,
    bool enable_logging_pressure,
    bool enable_logging_battery,
    bool enable_logging_packets)
    : m_cf(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_enableParameters(enable_parameters)
    , m_logBlocks(log_blocks)
    , m_use_ros_time(use_ros_time)
    , m_enable_logging_imu(enable_logging_imu)
    , m_enable_logging_temperature(enable_logging_temperature)
    , m_enable_logging_magnetic_field(enable_logging_magnetic_field)
    , m_enable_logging_pressure(enable_logging_pressure)
    , m_enable_logging_battery(enable_logging_battery)
    , m_enable_logging_packets(enable_logging_packets)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_subscribeCmdVel()
    , m_subscribeCmdFullState()
    , m_subscribeCmdHover()
    , m_subscribeCmdStop()
    , m_subscribeCmdPosition()
    , m_subscribeExternalPosition()
    , m_pubImu()
    , m_pubTemp()
    , m_pubMag()
    , m_pubPressure()
    , m_pubBattery()
    , m_pubRssi()
    , m_sentSetpoint(false)
    , m_sentExternalPosition(false)
  {
    ros::NodeHandle n;
    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_subscribeCmdFullState = n.subscribe(tf_prefix + "/cmd_full_state", 1, &CrazyflieROS::cmdFullStateSetpoint, this);
    m_subscribeCmdHover = n.subscribe(tf_prefix + "/cmd_hover", 1, &CrazyflieROS::cmdHoverSetpoint, this);
    m_subscribeCmdStop = n.subscribe(tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);
    m_subscribeCmdPosition = n.subscribe(tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);
    m_subscribeExternalPosition = n.subscribe(tf_prefix + "/external_position", 1, &CrazyflieROS::positionMeasurementChanged, this);
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);

    if (m_enable_logging_imu) {
      m_pubImu = n.advertise<sensor_msgs::Imu>(tf_prefix + "/imu", 10);
    }
    if (m_enable_logging_temperature) {
      m_pubTemp = n.advertise<sensor_msgs::Temperature>(tf_prefix + "/temperature", 10);
    }
    if (m_enable_logging_magnetic_field) {
      m_pubMag = n.advertise<sensor_msgs::MagneticField>(tf_prefix + "/magnetic_field", 10);
    }
    if (m_enable_logging_pressure) {
      m_pubPressure = n.advertise<std_msgs::Float32>(tf_prefix + "/pressure", 10);
    }
    if (m_enable_logging_battery) {
      m_pubBattery = n.advertise<std_msgs::Float32>(tf_prefix + "/battery", 10);
    }
    if (m_enable_logging_packets) {
      m_pubPackets = n.advertise<crazyflie_driver::crtpPacket>(tf_prefix + "/packets", 10);
    }
    m_pubRssi = n.advertise<std_msgs::Float32>(tf_prefix + "/rssi", 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }

    m_sendPacketServer = n.advertiseService(tf_prefix + "/send_packet"  , &CrazyflieROS::sendPacket, this);

    m_thread = std::thread(&CrazyflieROS::run, this);
  }

  void stop()
  {
    ROS_INFO("Disconnecting ...");
    m_isEmergency = true;
    m_thread.join();
  }

  /**
   * Service callback which transmits a packet to the crazyflie
   * @param  req The service request, which contains a crtpPacket to transmit.
   * @param  res The service response, which is not used.
   * @return     returns true always
   */
  bool sendPacket (
    crazyflie_driver::sendPacket::Request &req,
    crazyflie_driver::sendPacket::Response &res)
  {
    /** Convert the message struct to the packet struct */
    crtpPacket_t packet;
    packet.size = req.packet.size;
    packet.header = req.packet.header;
    for (int i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
      packet.data[i] = req.packet.data[i];
    }
    m_cf.queueOutgoingPacket(packet);
    return true;
  }

private:
  ros::ServiceServer m_sendPacketServer;

  /**
   * Publishes any generic packets en-queued by the crazyflie to a crtpPacket
   * topic.
   */
  void publishPackets() {
    std::vector<Crazyradio::Ack> packets = m_cf.retrieveGenericPackets();
    if (!packets.empty())
    {
      std::vector<Crazyradio::Ack>::iterator it;
      for (it = packets.begin(); it != packets.end(); it++)
      {
        crazyflie_driver::crtpPacket packet;
        packet.size = it->size;
        packet.header = it->data[0];
        for(int i = 0; i < packet.size; i++)
        {
          packet.data[i] = it->data[i+1];
        }
        m_pubPackets.publish(packet);
      }
    }
  }

private:
  struct logImu {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } __attribute__((packed));

  struct log2 {
    float mag_x;
    float mag_y;
    float mag_z;
    float baro_temp;
    float baro_pressure;
    float pm_vbat;
  } __attribute__((packed));

private:
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

void cmdHoverSetpoint(
    const crazyflie_driver::Hover::ConstPtr& msg)
  {
     //ROS_INFO("got a hover setpoint");
    if (!m_isEmergency) {
      float vx = msg->vx;
      float vy = msg->vy;
      float yawRate = msg->yawrate;
      float zDistance = msg->zDistance;

      m_cf.sendHoverSetpoint(vx, vy, yawRate, zDistance);
      m_sentSetpoint = true;
      //ROS_INFO("set a hover setpoint");
    }
  }

void cmdStop(
    const crazyflie_driver::Stop::ConstPtr& msg)
  {
     //ROS_INFO("got a stop setpoint");
    if (!m_isEmergency) {
      m_cf.sendStop();
      m_sentSetpoint = true;
      //ROS_INFO("set a stop setpoint");
    }
  }

void cmdPositionSetpoint(
    const crazyflie_driver::Position::ConstPtr& msg)
  {
    if(!m_isEmergency) {
      float x = msg->x;
      float y = msg->y;
      float z = msg->z;
      float yaw = msg->yaw;

      m_cf.sendPositionSetpoint(x, y, z, yaw);
      m_sentSetpoint = true;
    }
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
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
      m_sentSetpoint = true;
    }
  }

  void cmdFullStateSetpoint(
    const crazyflie_driver::FullState::ConstPtr& msg)
  {
    //ROS_INFO("got a full state setpoint");
    if (!m_isEmergency) {
      float x = msg->pose.position.x;
      float y = msg->pose.position.y;
      float z = msg->pose.position.z;
      float vx = msg->twist.linear.x;
      float vy = msg->twist.linear.y;
      float vz = msg->twist.linear.z;
      float ax = msg->acc.x;
      float ay = msg->acc.y;
      float az = msg->acc.z;

      float qx = msg->pose.orientation.x;
      float qy = msg->pose.orientation.y;
      float qz = msg->pose.orientation.z;
      float qw = msg->pose.orientation.w;
      float rollRate = msg->twist.angular.x;
      float pitchRate = msg->twist.angular.y;
      float yawRate = msg->twist.angular.z;

      m_cf.sendFullStateSetpoint(
        x, y, z,
        vx, vy, vz,
        ax, ay, az,
        qx, qy, qz, qw,
        rollRate, pitchRate, yawRate);
      m_sentSetpoint = true;
      //ROS_INFO("set a full state setpoint");
    }
  }

  void positionMeasurementChanged(
    const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    m_cf.sendExternalPositionUpdate(msg->point.x, msg->point.y, msg->point.z);
    m_sentExternalPosition = true;
  }

  void run()
  {
    // m_cf.reboot();

    auto start = std::chrono::system_clock::now();

    m_cf.logReset();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);



    if (m_enableParameters)
    {
      ROS_INFO("Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
      ros::NodeHandle n;
      m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<log2> > logBlock2;
    std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      if (m_enable_logging_imu) {
        std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockImu.reset(new LogBlock<logImu>(
          &m_cf,{
            {"acc", "x"},
            {"acc", "y"},
            {"acc", "z"},
            {"gyro", "x"},
            {"gyro", "y"},
            {"gyro", "z"},
          }, cb));
        logBlockImu->start(1); // 10ms
      }

      if (   m_enable_logging_temperature
          || m_enable_logging_magnetic_field
          || m_enable_logging_pressure
          || m_enable_logging_battery)
      {
        std::function<void(uint32_t, log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock2.reset(new LogBlock<log2>(
          &m_cf,{
            {"mag", "x"},
            {"mag", "y"},
            {"mag", "z"},
            {"baro", "temp"},
            {"baro", "pressure"},
            {"pm", "vbat"},
          }, cb2));
        logBlock2->start(10); // 100ms
      }

      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }


    }

    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint && !m_sentExternalPosition) {
        m_cf.transmitPackets();
        m_cf.sendPing();
        if(m_enable_logging_packets) {
          this->publishPackets();
        }
      }
      m_sentSetpoint = false;
      m_sentExternalPosition = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

  }

  void onImuData(uint32_t time_in_ms, logImu* data) {
    if (m_enable_logging_imu) {
      sensor_msgs::Imu msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      msg.orientation_covariance[0] = -1;

      // measured in deg/s; need to convert to rad/s
      msg.angular_velocity.x = degToRad(data->gyro_x);
      msg.angular_velocity.y = degToRad(data->gyro_y);
      msg.angular_velocity.z = degToRad(data->gyro_z);

      // measured in mG; need to convert to m/s^2
      msg.linear_acceleration.x = data->acc_x * 9.81;
      msg.linear_acceleration.y = data->acc_y * 9.81;
      msg.linear_acceleration.z = data->acc_z * 9.81;

      m_pubImu.publish(msg);
    }
  }

  void onLog2Data(uint32_t time_in_ms, log2* data) {

    if (m_enable_logging_temperature) {
      sensor_msgs::Temperature msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      // measured in degC
      msg.temperature = data->baro_temp;
      m_pubTemp.publish(msg);
    }

    if (m_enable_logging_magnetic_field) {
      sensor_msgs::MagneticField msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

      // measured in Tesla
      msg.magnetic_field.x = data->mag_x;
      msg.magnetic_field.y = data->mag_y;
      msg.magnetic_field.z = data->mag_z;
      m_pubMag.publish(msg);
    }

    if (m_enable_logging_pressure) {
      std_msgs::Float32 msg;
      // hPa (=mbar)
      msg.data = data->baro_pressure;
      m_pubPressure.publish(msg);
    }

    if (m_enable_logging_battery) {
      std_msgs::Float32 msg;
      // V
      msg.data = data->pm_vbat;
      m_pubBattery.publish(msg);
    }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.values = *values;

    pub->publish(msg);
  }

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("Link Quality low (%f)", linkQuality);
      }
  }

private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  bool m_enableLogging;
  bool m_enableParameters;
  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_imu;
  bool m_enable_logging_temperature;
  bool m_enable_logging_magnetic_field;
  bool m_enable_logging_pressure;
  bool m_enable_logging_battery;
  bool m_enable_logging_packets;

  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_subscribeCmdFullState;
  ros::Subscriber m_subscribeCmdHover;
  ros::Subscriber m_subscribeCmdStop;
  ros::Subscriber m_subscribeCmdPosition;
  ros::Subscriber m_subscribeExternalPosition;
  ros::Publisher m_pubImu;
  ros::Publisher m_pubTemp;
  ros::Publisher m_pubMag;
  ros::Publisher m_pubPressure;
  ros::Publisher m_pubBattery;
  ros::Publisher m_pubPackets;
  ros::Publisher m_pubRssi;
  std::vector<ros::Publisher> m_pubLogDataGeneric;

  bool m_sentSetpoint, m_sentExternalPosition;

  std::thread m_thread;
};

static std::map<std::string, CrazyflieROS*> crazyflies;

bool add_crazyflie(
  crazyflie_driver::AddCrazyflie::Request  &req,
  crazyflie_driver::AddCrazyflie::Response &res)
{
  ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_parameters,
    req.enable_logging,
    req.use_ros_time);

  // Ignore if the uri is already in use
  if (crazyflies.find(req.uri) != crazyflies.end()) {
    ROS_ERROR("Cannot add %s, already added.", req.uri.c_str());
    return false;
  }

  CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_parameters,
    req.log_blocks,
    req.use_ros_time,
    req.enable_logging_imu,
    req.enable_logging_temperature,
    req.enable_logging_magnetic_field,
    req.enable_logging_pressure,
    req.enable_logging_battery,
    req.enable_logging_packets);

  crazyflies[req.uri] = cf;

  return true;
}

bool remove_crazyflie(
  crazyflie_driver::RemoveCrazyflie::Request  &req,
  crazyflie_driver::RemoveCrazyflie::Response &res)
{

  if (crazyflies.find(req.uri) == crazyflies.end()) {
    ROS_ERROR("Cannot remove %s, not connected.", req.uri.c_str());
    return false;
  }

  ROS_INFO("Removing crazyflie at uri %s.", req.uri.c_str());

  crazyflies[req.uri]->stop();
  delete crazyflies[req.uri];
  crazyflies.erase(req.uri);

  ROS_INFO("Crazyflie %s removed.", req.uri.c_str());

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");
  ros::NodeHandle n;

  ros::ServiceServer serviceAdd = n.advertiseService("add_crazyflie", add_crazyflie);
  ros::ServiceServer serviceRemove = n.advertiseService("remove_crazyflie", remove_crazyflie);
  ros::spin();

  return 0;
}
