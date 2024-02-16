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
      this->goal_pose.position.x=goal_pose_x;
      this->goal_pose.position.y=goal_pose_y;
      this->reached=false;
      vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                  rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoToGoal::vel_callback, this, _1));
    }
  private:
    mutable geometry_msgs::msg::Pose current_pose;
    mutable geometry_msgs::msg::Pose goal_pose;
    mutable bool reached=false;
    float goal_tollerance=0.1;

    float distance() const
    {
        return sqrt(pow(this->current_pose.position.x-this->goal_pose.position.x,2)+pow(this->current_pose.position.y-this->goal_pose.position.y,2));
    }

    float angle_diff(const float yaw) const
    {
        float dx = this->goal_pose.position.x - this->current_pose.position.x;
        float dy = this->goal_pose.position.y - this->current_pose.position.y;
        float req_angle = atan2(dy, dx);
        return yaw - req_angle;
    }

    
    void vel_callback(const nav_msgs::msg::Odometry & msg) const
{
    geometry_msgs::msg::Twist vel_msg;
    this->current_pose.position.x = msg.pose.pose.position.x;
    this->current_pose.position.y = msg.pose.pose.position.y;

    auto quaternion = msg.pose.pose.orientation;

    Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    float yaw = q.get_yaw();
    if(this->reached==false)
    {
        if (distance()>this->goal_tollerance)
        {
            RCLCPP_INFO(this->get_logger(), "Current location is X[%f] & Y[%f]", this->current_pose.position.x, this->current_pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Distance is [%f]", distance());
            RCLCPP_INFO(this->get_logger(), "Angle Diff is [%f]", angle_diff(yaw));
            if (std::abs(angle_diff(yaw)) > M_PI / 36)
            {
                RCLCPP_INFO(this->get_logger(), "Angle Diff is [%f]", angle_diff(yaw));
                if (angle_diff(yaw) < 0)
                {
                    vel_msg.linear.x = -0.5;
                }
                else
                {
                    vel_msg.linear.x = 0.5;
                }
                vel_pub->publish(vel_msg);
                RCLCPP_INFO(this->get_logger(), "Publishing Angular Velocity");
            }
            else
            {
                vel_msg.linear.x = 0;
                RCLCPP_INFO(this->get_logger(), "Stop angular motion");
                if (distance() > this->goal_tollerance)
                {
                    RCLCPP_INFO(this->get_logger(), "Go to goal linear motion");
                    vel_msg.angular.z = -1; // Forward linear velocity when turning stops
                    vel_pub->publish(vel_msg);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Stop linear motion");
                    this->reached = true;
                    vel_msg.angular.z = 0; // Stop linear velocity
                    vel_pub->publish(vel_msg);
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Stop linear motion");
            this->reached = true;
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            vel_pub->publish(vel_msg);
        }
    }
    else
    {
        this->reached = true;
        vel_msg.angular.z = 0;
        vel_msg.linear.x = 0;
        vel_pub->publish(vel_msg);
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
  float goal_pose_x=std::stof(argv[1]);
  float goal_pose_y=std::stof(argv[2]);
  rclcpp::spin(std::make_shared<GoToGoal>(goal_pose_x,goal_pose_y));
  rclcpp::shutdown();
  return 0;
}
