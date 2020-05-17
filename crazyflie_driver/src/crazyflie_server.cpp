#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"

#include "crazyflie_driver/srv/add_crazyflie.hpp"
#include "crazyflie_driver/srv/go_to.hpp"
#include "crazyflie_driver/srv/land.hpp"
#include "crazyflie_driver/srv/notify_setpoints_stop.hpp"
#include "crazyflie_driver/srv/remove_crazyflie.hpp"
#include "crazyflie_driver/srv/set_group_mask.hpp"
#include "crazyflie_driver/srv/start_trajectory.hpp"
#include "crazyflie_driver/srv/stop.hpp"
#include "crazyflie_driver/srv/takeoff.hpp"
#include "crazyflie_driver/srv/update_params.hpp"
#include "crazyflie_driver/srv/upload_trajectory.hpp"
#include "crazyflie_driver/srv/send_packet.hpp"
#include "crazyflie_driver/srv/stop.hpp"

#include "crazyflie_driver/msg/log_block.hpp"
#include "crazyflie_driver/msg/generic_log_data.hpp"
#include "crazyflie_driver/msg/full_state.hpp"
#include "crazyflie_driver/msg/hover.hpp"
#include "crazyflie_driver/msg/position.hpp"
#include "crazyflie_driver/msg/velocity_world.hpp"
#include "crazyflie_driver/msg/crtp_packet.hpp"

#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float32.hpp"

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

class ROSLogger : public Logger
{
public:
  ROSLogger(const std::string& name)
    : m_logger(rclcpp::get_logger(name))
    , Logger()
  {
  }

  virtual ~ROSLogger() {}

  virtual void info(const std::string& msg) final override
  {
    RCLCPP_INFO(m_logger, "%s", msg.c_str());
  }

  virtual void warning(const std::string& msg) final override
  {
    RCLCPP_WARN(m_logger, "%s", msg.c_str());
  }

  virtual void error(const std::string& msg) final override
  {
    RCLCPP_ERROR(m_logger, "%s", msg.c_str());
  }

private:
  rclcpp::Logger m_logger;
};

static ROSLogger rosLogger("crazyflie_cpp");

class CrazyflieROS
{
public:
  CrazyflieROS(
    rclcpp::Node::SharedPtr node_handle,
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_parameters,
    std::vector<crazyflie_driver::msg::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_imu,
    bool enable_logging_temperature,
    bool enable_logging_magnetic_field,
    bool enable_logging_pressure,
    bool enable_logging_battery,
    bool enable_logging_pose,
    bool enable_logging_packets)
    : m_tf_prefix(tf_prefix)
    , m_nodeHandle(node_handle->create_sub_node(tf_prefix))
    , m_logger(m_nodeHandle->get_logger().get_child(tf_prefix))
    , m_cf(
      link_uri,
      rosLogger,
      std::bind(&CrazyflieROS::onConsole, this, std::placeholders::_1))
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
    , m_enable_logging_pose(enable_logging_pose)
    , m_enable_logging_packets(enable_logging_packets)
    , m_rosClock(RCL_SYSTEM_TIME)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_serviceSetGroupMask()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceStop()
    , m_serviceGoTo()
    , m_serviceUploadTrajectory()
    , m_serviceStartTrajectory()
    , m_serviceNotifySetpointsStop()
    , m_subscribeCmdVel()
    , m_subscribeCmdFullState()
    , m_subscribeCmdVelocityWorld()
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
    // If m_tf_prefix is empty, use the default node logger instead
    // to avoid the extra '.' character at the end of the logger name
    if (m_tf_prefix.empty())
    {
      m_logger = m_nodeHandle->get_logger();
    }
    m_thread = std::thread(&CrazyflieROS::run, this);
  }

  void terminate()
  {
    RCLCPP_INFO(m_logger, "Disconnecting ...");
    m_isEmergency = true;
    m_thread.join();
  }

  /**
   * Service callback which transmits a packet to the crazyflie
   * @param  req The service request, which contains a crtpPacket to transmit.
   * @param  res The service response, which is not used.
   * @return     returns true always
   */
  void sendPacket (
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::SendPacket::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::SendPacket::Response> res)
  {
    /** Convert the message struct to the packet struct */
    crtpPacket_t packet;
    packet.size = req->packet.size;
    packet.header = req->packet.header;
    for (int i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
      packet.data[i] = req->packet.data[i];
    }
    m_cf.queueOutgoingPacket(packet);
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

  struct logPose {
    float x;
    float y;
    float z;
    int32_t quatCompressed;
  } __attribute__((packed));

private:
  void emergency(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    RCLCPP_FATAL(m_logger, "Emergency requested!");
    m_isEmergency = true;
    m_cf.emergencyStop();
  }

  template<class T, class U>
  void updateParam(uint16_t id, const std::string& ros_param) {
      U value;
      if(m_nodeHandle->get_parameter(ros_param, value)) {
        m_cf.setParam<T>(id, (T)value);
      } else {
        RCLCPP_WARN(m_logger, "Unable to find ROS parameter: %s", ros_param.c_str());
      }
  }

void cmdHoverSetpoint(
    const crazyflie_driver::msg::Hover::SharedPtr msg)
  {
    //RCLCPP_INFO(m_logger, "got a hover setpoint");
    if (!m_isEmergency) {
      float vx = msg->v_x;
      float vy = msg->v_y;
      float yawRate = msg->yaw_rate;
      float zDistance = msg->z_distance;

      m_cf.sendHoverSetpoint(vx, vy, yawRate, zDistance);
      m_sentSetpoint = true;
      //RCLCPP_INFO(m_logger, "set a hover setpoint");
    }
  }

void cmdStop(
    const std_msgs::msg::Empty::SharedPtr msg)
  {
    //RCLCPP_INFO(m_logger, "got a stop setpoint");
    if (!m_isEmergency) {
      m_cf.sendStop();
      m_sentSetpoint = true;
      //RCLCPP_INFO(m_logger, "set a stop setpoint");
    }
  }

void cmdPositionSetpoint(
    const crazyflie_driver::msg::Position::SharedPtr msg)
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

  void updateParams(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::UpdateParams::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::UpdateParams::Response> res)
  {
    RCLCPP_INFO(m_logger, "Update parameters");
    for (auto&& p : req->params) {
      std::string ros_param = m_tf_prefix.empty() ? p : m_tf_prefix + "/" + p;
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
        RCLCPP_ERROR(m_logger, "Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
  }

  void cmdVelChanged(
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!m_isEmergency) {
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawRate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      m_cf.sendSetpoint(roll, pitch, yawRate, thrust);
      m_sentSetpoint = true;
    }
  }

  void cmdFullStateSetpoint(
    const crazyflie_driver::msg::FullState::SharedPtr msg)
  {
    //RCLCPP_INFO(m_logger, "got a full state setpoint");
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
      //RCLCPP_INFO(m_logger, "set a full state setpoint");
    }
  }

  void cmdVelocityWorldSetpoint(
    const crazyflie_driver::msg::VelocityWorld::SharedPtr msg)
  {
    //RCLCPP_INFO(m_logger, "got a velocity world setpoint");
    if (!m_isEmergency) {
      float x = msg->vel.x;
      float y = msg->vel.y;
      float z = msg->vel.z;
      float yawRate = msg->yaw_rate;

      m_cf.sendVelocityWorldSetpoint(
        x, y, z, yawRate);
      m_sentSetpoint = true;
      //RCLCPP_INFO(m_logger, "set a velocity world setpoint");
    }
  }

  void positionMeasurementChanged(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    m_cf.sendExternalPositionUpdate(msg->point.x, msg->point.y, msg->point.z);
    m_sentExternalPosition = true;
  }

  void poseMeasurementChanged(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    m_cf.sendExternalPoseUpdate(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    m_sentExternalPosition = true;
  }

  void run()
  {
    m_subscribeCmdVel = m_nodeHandle->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&CrazyflieROS::cmdVelChanged, this, std::placeholders::_1));
    m_subscribeCmdFullState = m_nodeHandle->create_subscription<crazyflie_driver::msg::FullState>(
      "cmd_full_state", 1, std::bind(&CrazyflieROS::cmdFullStateSetpoint, this, std::placeholders::_1));
    m_subscribeCmdVelocityWorld = m_nodeHandle->create_subscription<crazyflie_driver::msg::VelocityWorld>(
      "cmd_velocity_world", 1, std::bind(&CrazyflieROS::cmdVelocityWorldSetpoint, this, std::placeholders::_1));
    m_subscribeExternalPosition = m_nodeHandle->create_subscription<geometry_msgs::msg::PointStamped>(
      "external_position", 1, std::bind(&CrazyflieROS::positionMeasurementChanged, this, std::placeholders::_1));
    m_subscribeExternalPose = m_nodeHandle->create_subscription<geometry_msgs::msg::PoseStamped>(
      "external_pose", 1, std::bind(&CrazyflieROS::poseMeasurementChanged, this, std::placeholders::_1));
    m_serviceEmergency = m_nodeHandle->create_service<std_srvs::srv::Empty>(
      "emergency", std::bind(&CrazyflieROS::emergency, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_subscribeCmdHover = m_nodeHandle->create_subscription<crazyflie_driver::msg::Hover>(
      "cmd_hover", 1, std::bind(&CrazyflieROS::cmdHoverSetpoint, this, std::placeholders::_1));
    m_subscribeCmdStop = m_nodeHandle->create_subscription<std_msgs::msg::Empty>(
      "cmd_stop", 1, std::bind(&CrazyflieROS::cmdStop, this, std::placeholders::_1));
    m_subscribeCmdPosition = m_nodeHandle->create_subscription<crazyflie_driver::msg::Position>(
      "cmd_position", 1, std::bind(&CrazyflieROS::cmdPositionSetpoint, this, std::placeholders::_1));

    m_serviceSetGroupMask = m_nodeHandle->create_service<crazyflie_driver::srv::SetGroupMask>(
      "set_group_mask", std::bind(&CrazyflieROS::setGroupMask, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceTakeoff = m_nodeHandle->create_service<crazyflie_driver::srv::Takeoff>(
      "takeoff", std::bind(&CrazyflieROS::takeoff, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceLand = m_nodeHandle->create_service<crazyflie_driver::srv::Land>(
      "land", std::bind(&CrazyflieROS::land, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceStop = m_nodeHandle->create_service<crazyflie_driver::srv::Stop>(
      "stop", std::bind(&CrazyflieROS::stop, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceGoTo = m_nodeHandle->create_service<crazyflie_driver::srv::GoTo>(
      "go_to", std::bind(&CrazyflieROS::goTo, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceUploadTrajectory = m_nodeHandle->create_service<crazyflie_driver::srv::UploadTrajectory>(
      "upload_trajectory", std::bind(&CrazyflieROS::uploadTrajectory, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceStartTrajectory = m_nodeHandle->create_service<crazyflie_driver::srv::StartTrajectory>(
      "start_trajectory", std::bind(&CrazyflieROS::startTrajectory, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceNotifySetpointsStop = m_nodeHandle->create_service<crazyflie_driver::srv::NotifySetpointsStop>(
      "notify_setpoints_stop", std::bind(&CrazyflieROS::notifySetpointsStop, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    if (m_enable_logging_imu) {
      m_pubImu = m_nodeHandle->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }
    if (m_enable_logging_temperature) {
      m_pubTemp = m_nodeHandle->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    }
    if (m_enable_logging_magnetic_field) {
      m_pubMag = m_nodeHandle->create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 10);
    }
    if (m_enable_logging_pressure) {
      m_pubPressure = m_nodeHandle->create_publisher<std_msgs::msg::Float32>("pressure", 10);
    }
    if (m_enable_logging_battery) {
      m_pubBattery = m_nodeHandle->create_publisher<std_msgs::msg::Float32>("battery", 10);
    }
    if (m_enable_logging_pose) {
      m_pubPose = m_nodeHandle->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    }
    if (m_enable_logging_packets) {
      m_pubPackets = m_nodeHandle->create_publisher<crazyflie_driver::msg::CrtpPacket>("packets", 10);
      std::function<void(const ITransport::Ack&)> cb_genericPacket = std::bind(&CrazyflieROS::onGenericPacket, this, std::placeholders::_1);
      m_cf.setGenericPacketCallback(cb_genericPacket);
    }
    m_pubRssi = m_nodeHandle->create_publisher<std_msgs::msg::Float32>("rssi", 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(m_nodeHandle->create_publisher<crazyflie_driver::msg::GenericLogData>(logBlock.topic_name, 10));
    }

    m_sendPacketServer = m_nodeHandle->create_service<crazyflie_driver::srv::SendPacket>(
      "send_packet", std::bind(&CrazyflieROS::sendPacket, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // m_cf.reboot();

    auto start = std::chrono::system_clock::now();

    m_cf.logReset();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    if (m_enableParameters)
    {
      RCLCPP_INFO(m_logger, "Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = m_tf_prefix.empty() ? entry.group + "/" + entry.name
                                                    : m_tf_prefix + "/" + entry.group + "/" + entry.name;
        RCLCPP_INFO(m_logger, "Adding param: %s", paramName.c_str());
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            m_nodeHandle->set_parameter({paramName, m_cf.getParam<uint8_t>(entry.id)});
            break;
          case Crazyflie::ParamTypeInt8:
            m_nodeHandle->set_parameter({paramName, m_cf.getParam<int8_t>(entry.id)});
            break;
          case Crazyflie::ParamTypeUint16:
            m_nodeHandle->set_parameter({paramName, m_cf.getParam<uint16_t>(entry.id)});
            break;
          case Crazyflie::ParamTypeInt16:
            m_nodeHandle->set_parameter({paramName, m_cf.getParam<int16_t>(entry.id)});
            break;
          case Crazyflie::ParamTypeUint32:
            m_nodeHandle->set_parameter({paramName, (int)m_cf.getParam<uint32_t>(entry.id)});
            break;
          case Crazyflie::ParamTypeInt32:
            m_nodeHandle->set_parameter({paramName, m_cf.getParam<int32_t>(entry.id)});
            break;
          case Crazyflie::ParamTypeFloat:
            m_nodeHandle->set_parameter({paramName, m_cf.getParam<float>(entry.id)});
            break;
        }
      }
      m_serviceUpdateParams = m_nodeHandle->create_service<crazyflie_driver::srv::UpdateParams>(
        "update_params", std::bind(&CrazyflieROS::updateParams, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<log2> > logBlock2;
    std::unique_ptr<LogBlock<logPose> > logBlockPose;
    std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      RCLCPP_INFO(m_logger, "Requesting Logging variables...");
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

      if (m_enable_logging_pose) {
        std::function<void(uint32_t, logPose*)> cb = std::bind(&CrazyflieROS::onPoseData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockPose.reset(new LogBlock<logPose>(
          &m_cf,{
            {"stateEstimate", "x"},
            {"stateEstimate", "y"},
            {"stateEstimate", "z"},
            {"stateEstimateZ", "quat"}
          }, cb));
        logBlockPose->start(1); // 10ms
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
          (void*)m_pubLogDataGeneric[i].get(),
          cb));
        logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }


    }

    RCLCPP_INFO(m_logger, "Requesting memories...");
    m_cf.requestMemoryToc();

    RCLCPP_INFO(m_logger, "Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    RCLCPP_INFO(m_logger, "Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint && !m_sentExternalPosition) {
        m_cf.transmitPackets();
        m_cf.sendPing();
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
      sensor_msgs::msg::Imu msg;
      if (m_use_ros_time) {
        msg.header.stamp = m_rosClock.now();
      } else {
        msg.header.stamp = rclcpp::Time(time_in_ms / 1000.0);
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

      m_pubImu->publish(msg);
    }
  }

  void onLog2Data(uint32_t time_in_ms, log2* data) {
    if (m_enable_logging_temperature) {
      sensor_msgs::msg::Temperature msg;
      if (m_use_ros_time) {
        msg.header.stamp = m_rosClock.now();
      } else {
        msg.header.stamp = rclcpp::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      // measured in degC
      msg.temperature = data->baro_temp;
      m_pubTemp->publish(msg);
    }

    if (m_enable_logging_magnetic_field) {
      sensor_msgs::msg::MagneticField msg;
      if (m_use_ros_time) {
        msg.header.stamp = m_rosClock.now();
      } else {
        msg.header.stamp = rclcpp::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

      // measured in Tesla
      msg.magnetic_field.x = data->mag_x;
      msg.magnetic_field.y = data->mag_y;
      msg.magnetic_field.z = data->mag_z;
      m_pubMag->publish(msg);
    }

    if (m_enable_logging_pressure) {
      std_msgs::msg::Float32 msg;
      // hPa (=mbar)
      msg.data = data->baro_pressure;
      m_pubPressure->publish(msg);
    }

    if (m_enable_logging_battery) {
      std_msgs::msg::Float32 msg;
      // V
      msg.data = data->pm_vbat;
      m_pubBattery->publish(msg);
    }
  }

  void onPoseData(uint32_t time_in_ms, logPose* data) {
    if (m_enable_logging_pose) {
      geometry_msgs::msg::PoseStamped msg;
      if (m_use_ros_time) {
        msg.header.stamp = m_rosClock.now();
      } else {
        msg.header.stamp = rclcpp::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

      msg.pose.position.x = data->x;
      msg.pose.position.y = data->y;
      msg.pose.position.z = data->z;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.orientation.x = q[0];
      msg.pose.orientation.y = q[1];
      msg.pose.orientation.z = q[2];
      msg.pose.orientation.w = q[3];

      m_pubPose->publish(msg);
    }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {
    rclcpp::Publisher<crazyflie_driver::msg::GenericLogData>* pub =
      reinterpret_cast<rclcpp::Publisher<crazyflie_driver::msg::GenericLogData>*>(userData);

    crazyflie_driver::msg::GenericLogData msg;
    if (m_use_ros_time) {
      msg.header.stamp = m_rosClock.now();
    } else {
      msg.header.stamp = rclcpp::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.values = *values;

    pub->publish(msg);
  }

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::msg::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi->publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        RCLCPP_WARN(m_logger, "Link Quality low (%f)", linkQuality);
      }
  }

  void onConsole(const char* msg) {
    static std::string messageBuffer;
    messageBuffer += msg;
    size_t pos = messageBuffer.find('\n');
    if (pos != std::string::npos) {
      messageBuffer[pos] = 0;
      RCLCPP_INFO(m_logger, "CF Console: %s", messageBuffer.c_str());
      messageBuffer.erase(0, pos+1);
    }
  }

  void onGenericPacket(const ITransport::Ack& ack) {
    crazyflie_driver::msg::CrtpPacket packet;
    packet.size = ack.size;
    packet.header = ack.data[0];
    memcpy(&packet.data[0], &ack.data[1], ack.size);
    m_pubPackets->publish(packet);
  }

  void setGroupMask(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::SetGroupMask::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::SetGroupMask::Response> res)
  {
    RCLCPP_INFO(m_logger, "SetGroupMask requested");
    m_cf.setGroupMask(req->group_mask);
  }

  void takeoff(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::Takeoff::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::Takeoff::Response> res)
  {
    RCLCPP_INFO(m_logger, "Takeoff requested");
    m_cf.takeoff(req->height, req->duration.sec, req->group_mask);
  }

  void land(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::Land::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::Land::Response> res)
  {
    RCLCPP_INFO(m_logger, "Land requested");
    m_cf.land(req->height, req->duration.sec, req->group_mask);
  }

  void stop(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::Stop::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::Stop::Response> res)
  {
    RCLCPP_INFO(m_logger, "Stop requested");
    m_cf.stop(req->group_mask);
  }

  void goTo(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::GoTo::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::GoTo::Response> res)
  {
    RCLCPP_INFO(m_logger, "GoTo requested");
    m_cf.goTo(req->goal.x, req->goal.y, req->goal.z, req->yaw, req->duration.sec, req->relative, req->group_mask);
  }

  void uploadTrajectory(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::UploadTrajectory::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::UploadTrajectory::Response> res)
  {
    RCLCPP_INFO(m_logger, "UploadTrajectory requested");

    std::vector<Crazyflie::poly4d> pieces(req->pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i) {
      if (   req->pieces[i].poly_x.size() != 8
          || req->pieces[i].poly_y.size() != 8
          || req->pieces[i].poly_z.size() != 8
          || req->pieces[i].poly_yaw.size() != 8) {
        RCLCPP_FATAL(m_logger, "Wrong number of pieces!");
        return;
      }
      pieces[i].duration = req->pieces[i].duration.sec;
      for (size_t j = 0; j < 8; ++j) {
        pieces[i].p[0][j] = req->pieces[i].poly_x[j];
        pieces[i].p[1][j] = req->pieces[i].poly_y[j];
        pieces[i].p[2][j] = req->pieces[i].poly_z[j];
        pieces[i].p[3][j] = req->pieces[i].poly_yaw[j];
      }
    }
    m_cf.uploadTrajectory(req->trajectory_id, req->piece_offset, pieces);

    RCLCPP_INFO(m_logger, "Upload completed!");
  }

  void startTrajectory(
    const std::shared_ptr<rmw_request_id_t>,
    std::shared_ptr<crazyflie_driver::srv::StartTrajectory::Request> req,
    std::shared_ptr<crazyflie_driver::srv::StartTrajectory::Response> res)
  {
    RCLCPP_INFO(m_logger, "StartTrajectory requested");
    m_cf.startTrajectory(req->trajectory_id, req->timescale, req->reversed, req->relative, req->group_mask);
  }

  void notifySetpointsStop(
    const std::shared_ptr<rmw_request_id_t>,
    std::shared_ptr<crazyflie_driver::srv::NotifySetpointsStop::Request> req,
    std::shared_ptr<crazyflie_driver::srv::NotifySetpointsStop::Response> res)
  {
    RCLCPP_INFO(m_logger, "NotifySetpointsStop requested");
    m_cf.notifySetpointsStop(req->remain_valid_millisecs);
  }

private:
  std::string m_tf_prefix;
  Crazyflie m_cf;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  bool m_enableLogging;
  bool m_enableParameters;
  std::vector<crazyflie_driver::msg::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_imu;
  bool m_enable_logging_temperature;
  bool m_enable_logging_magnetic_field;
  bool m_enable_logging_pressure;
  bool m_enable_logging_battery;
  bool m_enable_logging_pose;
  bool m_enable_logging_packets;

  rclcpp::Node::SharedPtr m_nodeHandle;
  rclcpp::Logger m_logger;
  rclcpp::Clock m_rosClock;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_serviceEmergency;
  rclcpp::Service<crazyflie_driver::srv::UpdateParams>::SharedPtr m_serviceUpdateParams;
  rclcpp::Service<crazyflie_driver::srv::SendPacket>::SharedPtr m_sendPacketServer;

  // High-level setpoints
  rclcpp::Service<crazyflie_driver::srv::SetGroupMask>::SharedPtr m_serviceSetGroupMask;
  rclcpp::Service<crazyflie_driver::srv::Takeoff>::SharedPtr m_serviceTakeoff;
  rclcpp::Service<crazyflie_driver::srv::Land>::SharedPtr m_serviceLand;
  rclcpp::Service<crazyflie_driver::srv::Stop>::SharedPtr m_serviceStop;
  rclcpp::Service<crazyflie_driver::srv::GoTo>::SharedPtr m_serviceGoTo;
  rclcpp::Service<crazyflie_driver::srv::UploadTrajectory>::SharedPtr m_serviceUploadTrajectory;
  rclcpp::Service<crazyflie_driver::srv::StartTrajectory>::SharedPtr m_serviceStartTrajectory;
  rclcpp::Service<crazyflie_driver::srv::NotifySetpointsStop>::SharedPtr m_serviceNotifySetpointsStop;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscribeCmdVel;
  rclcpp::Subscription<crazyflie_driver::msg::FullState>::SharedPtr m_subscribeCmdFullState;
  rclcpp::Subscription<crazyflie_driver::msg::Hover>::SharedPtr m_subscribeCmdHover;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_subscribeCmdStop;
  rclcpp::Subscription<crazyflie_driver::msg::Position>::SharedPtr m_subscribeCmdPosition;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_subscribeExternalPosition;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subscribeExternalPose;
  rclcpp::Subscription<crazyflie_driver::msg::VelocityWorld>::SharedPtr m_subscribeCmdVelocityWorld;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pubImu;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_pubTemp;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_pubMag;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubPressure;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubBattery;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pubPose;
  rclcpp::Publisher<crazyflie_driver::msg::CrtpPacket>::SharedPtr m_pubPackets;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubRssi;
  std::vector<rclcpp::Publisher<crazyflie_driver::msg::GenericLogData>::SharedPtr> m_pubLogDataGeneric;

  bool m_sentSetpoint, m_sentExternalPosition;

  std::thread m_thread;
};

class CrazyflieServer : public rclcpp::Node
{
public:
  CrazyflieServer(const rclcpp::NodeOptions& options)
  : Node("crazyflie_server", options)
  {
    m_serviceAddCrazyflie = this->create_service<crazyflie_driver::srv::AddCrazyflie>(
      "add_crazyflie", std::bind(&CrazyflieServer::add_crazyflie, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_serviceRemoveCrazyflie = this->create_service<crazyflie_driver::srv::RemoveCrazyflie>(
      "remove_crazyflie", std::bind(&CrazyflieServer::remove_crazyflie, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:

  bool add_crazyflie(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::AddCrazyflie::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::AddCrazyflie::Response> res)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
      req->uri.c_str(),
      req->tf_prefix.c_str(),
      req->roll_trim,
      req->pitch_trim,
      req->enable_parameters,
      req->enable_logging,
      req->use_ros_time);

    // Ignore if the uri is already in use
    if (m_crazyflies.find(req->uri) != m_crazyflies.end()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot add %s, already added.", req->uri.c_str());
      return false;
    }

    CrazyflieROS* cf = new CrazyflieROS(
      shared_from_this(),
      req->uri,
      req->tf_prefix,
      req->roll_trim,
      req->pitch_trim,
      req->enable_logging,
      req->enable_parameters,
      req->log_blocks,
      req->use_ros_time,
      req->enable_logging_imu,
      req->enable_logging_temperature,
      req->enable_logging_magnetic_field,
      req->enable_logging_pressure,
      req->enable_logging_battery,
      req->enable_logging_pose,
      req->enable_logging_packets);

    m_crazyflies[req->uri] = cf;

    return true;
  }

  bool remove_crazyflie(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<crazyflie_driver::srv::RemoveCrazyflie::Request> req,
    const std::shared_ptr<crazyflie_driver::srv::RemoveCrazyflie::Response> res)
  {

    if (m_crazyflies.find(req->uri) == m_crazyflies.end()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot remove %s, not connected.", req->uri.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Removing crazyflie at uri %s.", req->uri.c_str());

    m_crazyflies[req->uri]->terminate();
    delete m_crazyflies[req->uri];
    m_crazyflies.erase(req->uri);

    RCLCPP_INFO(this->get_logger(), "Crazyflie %s removed.", req->uri.c_str());

    return true;
  }

  // bool takeoff(
  //   crazyflie_driver::Takeoff::Request& req,
  //   crazyflie_driver::Takeoff::Response& res)
  // {
  //   ROS_INFO("Takeoff requested");
  //   m_cfbc.takeoff(req.height, req.duration.toSec(), req.groupMask);
  //   return true;
  // }

  // bool land(
  //   crazyflie_driver::Land::Request& req,
  //   crazyflie_driver::Land::Response& res)
  // {
  //   ROS_INFO("Land requested");
  //   m_cfbc.land(req.height, req.duration.toSec(), req.groupMask);
  //   return true;
  // }

  // bool stop(
  //   crazyflie_driver::Stop::Request& req,
  //   crazyflie_driver::Stop::Response& res)
  // {
  //   ROS_INFO("Stop requested");
  //   m_cfbc.stop(req.groupMask);
  //   return true;
  // }

  // bool goTo(
  //   crazyflie_driver::GoTo::Request& req,
  //   crazyflie_driver::GoTo::Response& res)
  // {
  //   ROS_INFO("GoTo requested");
  //   // this is always relative
  //   m_cfbc.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.groupMask);
  //   return true;
  // }

  // bool startTrajectory(
  //   crazyflie_driver::StartTrajectory::Request& req,
  //   crazyflie_driver::StartTrajectory::Response& res)
  // {
  //   ROS_INFO("StartTrajectory requested");
  //   // this is always relative
  //   m_cfbc.startTrajectory(req.index, req.numPieces, req.timescale, req.reversed, req.groupMask);
  //   return true;
  // }

private:
  rclcpp::Service<crazyflie_driver::srv::AddCrazyflie>::SharedPtr m_serviceAddCrazyflie;
  rclcpp::Service<crazyflie_driver::srv::RemoveCrazyflie>::SharedPtr m_serviceRemoveCrazyflie;

  std::map<std::string, CrazyflieROS*> m_crazyflies;
};




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  rclcpp::spin(std::make_shared<CrazyflieServer>(options));
  rclcpp::shutdown();
  return 0;
}
