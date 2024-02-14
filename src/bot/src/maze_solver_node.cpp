#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <opencv2/opencv.hpp> 


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"


using namespace std::chrono_literals;
using std::placeholders::_1;


class MazeSolver : public rclcpp::Node
{
  public:
    MazeSolver()
    : Node("maze_solver_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/unit_box/image_raw", 10, std::bind(&MazeSolver::image_callback, this, _1));
    }
  private:
    
    

    void image_callback(const sensor_msgs::msg::Image & msg) const
    {

  
    // Show Image inside a window 
      try
      {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        if (cv_ptr==nullptr){
          RCLCPP_INFO(this->get_logger(), "cv_ptr is nullptr");
        }
        else{
          cv::Mat img;
          cv::resize(cv_ptr->image,img,cv::Size(240,240),cv::INTER_LINEAR);
          cv::imshow("Top Camera Image",img);
        }
      }
      catch (cv_bridge::Exception &e)
      {
         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      cv::waitKey(1);
        
    }
    

    rclcpp::TimerBase::SharedPtr vel_timer;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeSolver>());
  rclcpp::shutdown();
  return 0;
}