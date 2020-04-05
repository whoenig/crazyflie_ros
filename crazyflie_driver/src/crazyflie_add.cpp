#include "rclcpp/rclcpp.hpp"
#include "crazyflie_driver/srv/add_crazyflie.hpp"
#include "crazyflie_driver/msg/log_block.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("crazyflie_add", options);

  // read paramaters
  std::string uri;
  std::string tf_prefix;
  double roll_trim;
  double pitch_trim;
  bool enable_logging;
  bool enable_parameters;
  bool use_ros_time;
  bool enable_logging_imu;
  bool enable_logging_temperature;
  bool enable_logging_magnetic_field;
  bool enable_logging_pressure;
  bool enable_logging_battery;
  bool enable_logging_pose;
  bool enable_logging_packets;

  node->declare_parameter("uri", std::string("radio://0/80/2M"));
  node->declare_parameter("tf_prefix", std::string(""));
  node->declare_parameter("roll_trim", 0.0);
  node->declare_parameter("pitch_trim", 0.0);
  node->declare_parameter("enable_logging", true);
  node->declare_parameter("enable_parameters", true);
  node->declare_parameter("use_ros_time", true);
  node->declare_parameter("enable_logging_imu", true);
  node->declare_parameter("enable_logging_temperature", true);
  node->declare_parameter("enable_logging_magnetic_field", true);
  node->declare_parameter("enable_logging_pressure", true);
  node->declare_parameter("enable_logging_battery", true);
  node->declare_parameter("enable_logging_pose", false);
  node->declare_parameter("enable_logging_packets", true);

  node->get_parameter("uri", uri);
  node->get_parameter("tf_prefix", tf_prefix);
  node->get_parameter_or("roll_trim", roll_trim, 0.0);
  node->get_parameter_or("pitch_trim", pitch_trim, 0.0);
  node->get_parameter_or("enable_logging", enable_logging, true);
  node->get_parameter_or("enable_parameters", enable_parameters, true);
  node->get_parameter_or("use_ros_time", use_ros_time, true);
  node->get_parameter_or("enable_logging_imu", enable_logging_imu, true);
  node->get_parameter_or("enable_logging_temperature", enable_logging_temperature, true);
  node->get_parameter_or("enable_logging_magnetic_field", enable_logging_magnetic_field, true);
  node->get_parameter_or("enable_logging_pressure", enable_logging_pressure, true);
  node->get_parameter_or("enable_logging_battery", enable_logging_battery, true);
  node->get_parameter_or("enable_logging_pose", enable_logging_pose, false);
  node->get_parameter_or("enable_logging_packets", enable_logging_packets, true);

  RCLCPP_INFO(node->get_logger(), "wait_for_service /add_crazyflie");
  auto addCrazyflieService = node->create_client<crazyflie_driver::srv::AddCrazyflie>("/add_crazyflie");
  while (!addCrazyflieService->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(node->get_logger(), "found /add_crazyflie");

  auto addCrazyflie = std::make_shared<crazyflie_driver::srv::AddCrazyflie::Request>();
  addCrazyflie->uri = uri;
  addCrazyflie->tf_prefix = tf_prefix;
  addCrazyflie->roll_trim = roll_trim;
  addCrazyflie->pitch_trim = pitch_trim;
  addCrazyflie->enable_logging = enable_logging;
  addCrazyflie->enable_parameters = enable_parameters;
  addCrazyflie->use_ros_time = use_ros_time;
  addCrazyflie->enable_logging_imu = enable_logging_imu;
  addCrazyflie->enable_logging_temperature = enable_logging_temperature;
  addCrazyflie->enable_logging_magnetic_field = enable_logging_magnetic_field;
  addCrazyflie->enable_logging_pressure = enable_logging_pressure;
  addCrazyflie->enable_logging_battery = enable_logging_battery;
  addCrazyflie->enable_logging_pose = enable_logging_pose;
  addCrazyflie->enable_logging_packets = enable_logging_packets;

  // TODO -- specify log blocks as parameters
  //node->declare_parameter("genericLogTopics", std::vector<std::string>());
  //node->declare_parameter("genericLogTopicFrequencies", std::vector<int>());
  //std::vector<std::string> genericLogTopics;
  //node->get_parameter("genericLogTopics", genericLogTopics);
  //std::vector<int> genericLogTopicFrequencies;
  //node->get_parameter("genericLogTopicFrequencies", genericLogTopicFrequencies);

  //if (genericLogTopics.size() == genericLogTopicFrequencies.size())
  //{
  //  size_t i = 0;
  //  for (auto& topic : genericLogTopics)
  //  {
  //    crazyflie_driver::msg::LogBlock logBlock;
  //    logBlock.topic_name = topic;
  //    logBlock.frequency = genericLogTopicFrequencies[i];
  //    node->get_parameter("genericLogTopic_" + topic + "_Variables", logBlock.variables);
  //    addCrazyflie->log_blocks.push_back(logBlock);
  //    ++i;
  //  }
  //}
  //else
  //{
  //  RCLCPP_ERROR(node->get_logger(), "Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
  //}

  auto result = addCrazyflieService->async_send_request(addCrazyflie);
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Successfully added crazyfile");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to add crazyflie");
  }

  rclcpp::shutdown();

  return 0;
}
