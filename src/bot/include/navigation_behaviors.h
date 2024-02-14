#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/basic_types.h"
// #include "behaviortree_cpp/utils/optional.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class GoToPose : public BT::StatefulActionNode
{
    public:
        GoToPose(const std::string &name,
                 const BT::NodeConfiguration &config,
                 rclcpp::Node::SharedPtr node_ptr
        );

        //TYPEDEF
        using NavigateToPose=nav2_msgs::action::NavigateToPose;
        using GoalHandleNav=rclcpp_action::ClientGoalHandle<NavigateToPose>;

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
        bool done_flag_;

        //Method Overwrides
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        //Action Client callback
        void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result); 
};
