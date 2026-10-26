#include <rclcpp/rclcpp.hpp>
#include "mqttnode/mqttnode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MqttNode>("mqtt_node", true);
  node->connect();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
