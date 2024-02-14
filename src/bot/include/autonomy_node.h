#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "navigation_behaviors.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

class AutonomyNode: public rclcpp::Node
{
  public:
    explicit AutonomyNode(const std::string &node_name);
    void create_behavior_tree();
    void update_behavior_tree();
    void setup();
  
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
};