#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using std::placeholders::_1;


class LaserRemapper : public rclcpp::Node
{
  public:
    LaserRemapper()
    : Node("laser_remapper_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "laser_controller/out", 10, std::bind(&LaserRemapper::topic_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      RCLCPP_INFO(this->get_logger(), "Transferring laser scan data");

    };

  private:
    void topic_callback(const sensor_msgs::msg::LaserScan & msg) const
    {
      publisher_->publish(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserRemapper>());
  rclcpp::shutdown();
  return 0;
}
