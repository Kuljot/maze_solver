#include <memory>
#include <string>
#include <vector>

// fromhttps://docs.ros2.org/latest/api/ament_index_cpp/get__package__share__directory_8hpp_source.html
// #include "ament_index_cpp/get_package_share_directory.hpp"
// #include "rclcpp/node_options.hpp"
// #include "rclcpp/context.hpp"
// #include "rclcpp/contexts/default_context.hpp"
// #include "rclcpp/parameter.hpp"
// #include "rclcpp/publisher_options.hpp"
// #include "rclcpp/qos.hpp"
// #include "rclcpp/node.hpp"

int main(int argc, char** argv)
{ 
    std::string package_name="ball_follower";
    // rclcpp::init(argc, argv);

    // Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    // auto node = rclcpp::Node::make_shared(package_name);
    return 0;

}