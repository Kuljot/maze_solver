// // #include "date.h"
// #include <chrono>
// #include <iostream>
// #include <thread>
// #include <memory>
// #include <functional>
// #include <string>


// #include "rclcpp/rclcpp.hpp"
// // #include "rclcpp/qos.hpp"
// // #include "nav2_util/lifecycle_node.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// // #include "nav_msgs/srv/get_map.hpp"
// // #include "nav_msgs/srv/load_map.hpp"


// using std::placeholders::_1;


// class MapPublisher : public rclcpp::Node
// {
//   public:
//     MapPublisher()
//     : Node("map_publisher_node")
//     {
//       subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//       "map", 10, std::bind(&MapPublisher::topic_callback, this, _1));

//       publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
//                   "custom_map",
//                   rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

//       // Set up timer for continuous publishing
//       timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MapPublisher::timerCallback, this));
//     }
  

//   private:
//     void topic_callback(const nav_msgs::msg::OccupancyGrid & msg) const
//     {
//       while(1){
//       RCLCPP_INFO(this->get_logger(), "Republishing Map data");
//       // latest_map=msg;
//       publisher_->publish(msg);
//       std::this_thread::sleep_for(std::chrono::milliseconds(200));
//       }
//     }

//     void timerCallback() {
//     if (latest_map) {
//       publisher_->publish(*latest_map);  // Republish the latest map
//     }
//     }
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
//     nav_msgs::msg::OccupancyGrid::SharedPtr latest_map=nullptr;
//     // nav_msgs::msg::OccupancyGrid latest_map;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MapPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;
class MapPublisher : public rclcpp::Node
{
  public:
    MapPublisher()
    : Node("map_publisher_node")
    {
      og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid",
                  rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
      og_timer = this->create_wall_timer(500ms, std::bind(&MapPublisher::og_callback, this));
    }
  private:

    void og_callback()
    {

      auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
      std::vector<signed char> og_array(35);
      for(int i=0;i<35;i++){
        og_array[i] = i % 3 == 0 ? 100 : 0 ;
      }

      occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
      occupancy_grid_msg.header.frame_id = "map_frame";

      occupancy_grid_msg.info.resolution = 1;

      occupancy_grid_msg.info.width = 5;
      occupancy_grid_msg.info.height = 7;

      occupancy_grid_msg.info.origin.position.x = 0.0;
      occupancy_grid_msg.info.origin.position.y = 0.0;
      occupancy_grid_msg.info.origin.position.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.x = 0.0;
      occupancy_grid_msg.info.origin.orientation.y = 0.0;
      occupancy_grid_msg.info.origin.orientation.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.w = 1.0;
      occupancy_grid_msg.data = og_array;

      og_pub->publish(occupancy_grid_msg);
    }


    rclcpp::TimerBase::SharedPtr og_timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}