#include "navigation_behaviors.h"
#include "rclcpp/rclcpp.hpp"


GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
        :BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    std::cout<<" GoToPose Constructor called"<<std::endl;
    action_client_ptr_=rclcpp_action::create_client<NavigateToPose>(node_ptr_,"/navigate_to_pose");
    done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("loc")}; //Input Port from which bot will get location of goal
}

BT::NodeStatus GoToPose::onStart()
{
    BT::Expected<std::string> loc=getInput<std::string>("loc");
    const std::string location_file=node_ptr_->get_parameter("location_file").as_string();

    YAML::Node locations=YAML::LoadFile(location_file);
    std::vector<float> pose=locations[loc.value()].as<std::vector<float>>();

    //setup action client goal
    auto send_goal_options=rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback=std::bind(&GoToPose::nav_to_pose_callback,this,std::placeholders::_1);

    //make pose
    auto goal_msg=NavigateToPose::Goal();
    goal_msg.pose.header.frame_id="map";
    goal_msg.pose.pose.position.x=pose[0];
    goal_msg.pose.pose.position.y=pose[1];

    tf2::Quaternion q;
    q.setRPY(0,0,pose[2]);
    q.normalize(); //x^2+y^2+z^2=1
    goal_msg.pose.pose.orientation=tf2::toMsg(q);

    //send pose
    done_flag_=false;
    action_client_ptr_->async_send_goal(goal_msg,send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
    if (done_flag_)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"Goal Reached\n");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"Running\n");
        return BT::NodeStatus::RUNNING;
    }
}

// void GoToPose::onHalted(){
//     RCLCPP_INFO(node_ptr_->get_logger(),"Halted\n");
// }

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        done_flag_=true;
    }
}