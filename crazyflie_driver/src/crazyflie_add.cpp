#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_add", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");

  // read paramaters
  std::string uri;
  std::string tf_prefix;
  double roll_trim;
  double pitch_trim;
  bool enable_logging;

  n.getParam("uri", uri);
  n.getParam("tf_prefix", tf_prefix);
  n.param("roll_trim", roll_trim, 0.0);
  n.param("pitch_trim", pitch_trim, 0.0);
  n.param("enable_logging", enable_logging, true);

  ROS_INFO("wait_for_service /add_crazyflie");
  ros::ServiceClient addCrazyflieService = n.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie");
  addCrazyflieService.waitForExistence();
  ROS_INFO("found /add_crazyflie");
  crazyflie_driver::AddCrazyflie addCrazyflie;
  addCrazyflie.request.uri = uri;
  addCrazyflie.request.tf_prefix = tf_prefix;
  addCrazyflie.request.roll_trim = roll_trim;
  addCrazyflie.request.pitch_trim = pitch_trim;
  addCrazyflie.request.enable_logging = enable_logging;

  std::vector<std::string> genericLogTopics;
  n.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
  std::vector<int> genericLogTopicFrequencies;
  n.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());

  if (genericLogTopics.size() == genericLogTopicFrequencies.size())
  {
    size_t i = 0;
    for (auto& topic : genericLogTopics)
    {
      crazyflie_driver::LogBlock logBlock;
      logBlock.topic_name = topic;
      logBlock.frequency = genericLogTopicFrequencies[i];
      n.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
      addCrazyflie.request.log_blocks.push_back(logBlock);
      ++i;
    }
  }
  else
  {
    ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
  }


  addCrazyflieService.call(addCrazyflie);

  return 0;
}
