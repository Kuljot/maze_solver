#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;
class VelPublisher : public rclcpp::Node
{
  public:
    VelPublisher()
    : Node("driving_node")
    {
      vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                  rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
      //Vel timer to be recalled after every 5s
      vel_timer = this->create_wall_timer(5000ms, std::bind(&VelPublisher::vel_callback, this));
    }
  private:

    void vel_callback()
    {

      auto vel_msg = geometry_msgs::msg::Twist();

      vel_msg.angular.z=0.5;
      vel_pub->publish(vel_msg);
    }


    rclcpp::TimerBase::SharedPtr vel_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelPublisher>());
  rclcpp::shutdown();
  return 0;
}