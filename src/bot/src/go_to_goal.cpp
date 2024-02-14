#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class Quaternion
{
    public:
        Quaternion(float x, float y, float z, float w)
        {
            this->w=w;
            this->x=x;
            this->y=y;
            this->z=z;
            euler_from_quaternion();
        }

        float get_roll()
        {
            return this->roll;
        }

        float get_pitch()
        {
            return this->pitch;
        }
        
        float get_yaw()
        {
            return this->yaw;
        }

        void euler_from_quaternion()
        {
        float x = this->x;
        float y = this->y;
        float z = this->z;
        float w = this->w;

        float sinr_cosp = 2 * (w * x + y * z);
        float cosr_cosp = 1 - 2 * (x * x + y * y);
        this->roll = atan2(sinr_cosp, cosr_cosp);

        float sinp = 2 * (w * y - z * x);
        this->pitch = asin(sinp);

        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);
        this->yaw = atan2(siny_cosp, cosy_cosp);
        }

    private:
        float x = 0;
        float y = 0;
        float z = 0;
        float w = 0;
        float roll=0;
        float pitch=0;
        float yaw=0;
        
};
class GoToGoal : public rclcpp::Node
{
  public:
    GoToGoal(float goal_pose_x, float goal_pose_y)
    : Node("go_to_goal_node")
    {
      this->goal_pose_x=goal_pose_x;
      this->goal_pose_y=goal_pose_y;
      vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                  rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoToGoal::vel_callback, this, _1));
    }
  private:
    
    float goal_pose_x;
    float goal_pose_y;

    float distance(const float robot_pose_x,const float robot_pose_y) const
    {
        return sqrt(pow(robot_pose_x-this->goal_pose_x,2)+pow(robot_pose_y-this->goal_pose_y,2));
    }

    float angle_diff(const float robot_pose_x,const float robot_pose_y, const float yaw) const
    {
        float req_angle= atan((robot_pose_y-this->goal_pose_y)/(robot_pose_x-this->goal_pose_x));
        return yaw-req_angle;

    }

    void vel_callback(const nav_msgs::msg::Odometry & msg) const
    {

        auto vel_msg = geometry_msgs::msg::Twist();
        float robot_pose_x=msg.pose.pose.position.x;
        float robot_pose_y=msg.pose.pose.position.y;
        
        auto quaternion = msg.pose.pose.orientation;

        Quaternion q(quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w);
        
        float roll=q.get_roll();
        float pitch=q.get_pitch();
        float yaw=q.get_yaw();

        if (distance(robot_pose_x,robot_pose_y) >1)
        {
            RCLCPP_INFO(this->get_logger(),"Distance is [%f]",distance(robot_pose_x,robot_pose_y));
            RCLCPP_INFO(this->get_logger(),"Angle Diff is [%f]",angle_diff(robot_pose_x,robot_pose_y,yaw));
            if (abs(angle_diff(robot_pose_x,robot_pose_y,yaw)) >3.14/36)
            {
                RCLCPP_INFO(this->get_logger(),"Angle Diff is [%f]",angle_diff(robot_pose_x,robot_pose_y,yaw));
                if (angle_diff(robot_pose_x,robot_pose_y,yaw)<0){
                    vel_msg.linear.x=-0.05;
                }
                if (angle_diff(robot_pose_x,robot_pose_y,yaw)>=0){
                    vel_msg.linear.x=0.05;
                }
                vel_pub->publish(vel_msg);
                RCLCPP_INFO(this->get_logger(),"Publishing Angular Velocity");
                std::this_thread::sleep_for(100ms);
                vel_msg.linear.x=0;
                vel_pub->publish(vel_msg);
                RCLCPP_INFO(this->get_logger(),"Publishing to Stop Rotation");
            }
            else 
            {
                vel_msg.linear.x=0;
                vel_msg.angular.z=-1;
                vel_pub->publish(vel_msg);
            }
        }
        
    }
    

    rclcpp::TimerBase::SharedPtr vel_timer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: %s <goal_pose_x> <goal_pose_y>", argv[0]);
        return 1;
    }
  float goal_pose_x=*argv[1];
  float goal_pose_y=*argv[2];
  rclcpp::spin(std::make_shared<GoToGoal>(goal_pose_x,goal_pose_y));
  rclcpp::shutdown();
  return 0;
}