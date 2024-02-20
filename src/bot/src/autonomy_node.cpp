#include "autonomy_node.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir=
      ament_index_cpp::get_package_share_directory("maze_solver")+"/bt_xml";

AutonomyNode::AutonomyNode(const std::string & node_name)
  : Node(node_name)
{
  this->declare_parameter("location_file","none");
  RCLCPP_INFO(get_logger(),"Init done");
}

void AutonomyNode::setup()
{
  //Initial BT setup
  RCLCPP_INFO(get_logger(),"Autonomy Node Setup called");
  create_behavior_tree();

  const auto timer_period=50ms;
  this->timer_= this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree,this)
  );
  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
  std::cout<<timer_<<std::endl;
}

void AutonomyNode::create_behavior_tree()
{
  //Create BT
  RCLCPP_INFO(get_logger(),"Autonomy Node Create BT called");
  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder=
    [=](const std::string &name, const BT::NodeConfiguration &config) // Lamda function to create a node that takes more than 2 arguments
  {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  };

  factory.registerBuilder<GoToPose>("GoToPose",builder);
  RCLCPP_INFO(get_logger(),"BT Dir [%s]",bt_xml_dir.c_str());
  this->tree_=factory.createTreeFromFile(bt_xml_dir+"/tree.xml");
  RCLCPP_INFO(get_logger(),"Tree Created ");
}

void AutonomyNode::update_behavior_tree()
{
  //Tick BT when asked
  BT::NodeStatus tree_status=this->tree_.tickWhileRunning();
  // BT::NodeStatus tree_status=this->tree_.tickOnce();
  if (tree_status==BT::NodeStatus::RUNNING)
  {
    return;
  }
  else if(tree_status==BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(),"Finished Navigation");
  }
  else if(tree_status==BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(),"Navigation Failed");
    timer_->cancel();
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<AutonomyNode>("autonomy_node");
  node->setup();

  // rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}