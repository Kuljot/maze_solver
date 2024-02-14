#include "autonomy_node.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir=
      ament_index_cpp::get_package_share_directory("ball_follower")+"/bt_xml";

AutonomyNode::AutonomyNode(const std::string & node_name)
  : Node(node_name)
{
  this->declare_parameter("location_file","none");
  RCLCPP_INFO(get_logger(),"Init done");
}

void AutonomyNode::setup()
{
  //Initial BT setup
  create_behavior_tree();

  const auto timer_period=500ms;
  auto timer_= this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree,this)
  );
}

void AutonomyNode::create_behavior_tree()
{
  //Crete BT
  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder=
    [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  };

  factory.registerBuilder<GoToPose>("GoToPose",builder);

  tree_=factory.createTreeFromFile(bt_xml_dir+"/tree.xml");

}

void AutonomyNode::update_behavior_tree()
{
  //Tick BT when asked
  BT::NodeStatus tree_status=tree_.tickOnce();

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

  rclcpp::spin(node);
  rclcpp::shutdown();
}