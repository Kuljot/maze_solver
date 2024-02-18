#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <opencv2/opencv.hpp> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class PathPlanner : public rclcpp::Node
{
  public:
    PathPlanner(int goal)
    : Node("bot_localizer_node")
    {
        std_msgs::msg::Int32 g;
        g.data=goal;
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("goal_node", 10); 
        publisher_->publish(g);
    }
  private:
    float goal_pose_x;
    float goal_pose_y;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int goal=std::stoi(argv[1]);
  rclcpp::spin(std::make_shared<PathPlanner>(goal));
  rclcpp::shutdown();
  return 0;
}

class Graph
{
    private:
    int last_node=0;
    
    public:
    std::unordered_map<int,cv::Point2f> graph={};
    Graph()
    {
        this->last_node=0;
        this->graph={};
        

    }

    bool add_node(cv::Point2f pt, int no_of_pathways)
    {
        bool nearby_node_present=false;
            if (this->last_node==0)
            {
                this->graph[this->last_node]=pt;
                this->last_node++;
                return true;
            }
            else
            {
                // std::cout<<this->graph.size()<<"Size of array"<<std::endl;
                for (int i=0;i<this->last_node;i++)
                {
                    if(cv::norm(graph[i]-pt)<1.0)
                    {
                        nearby_node_present=true;
                        break;
                    }
                    
                }
                if(nearby_node_present==false)
                {   
                    this->graph[this->last_node+1]=pt;
                    this->last_node++;
                    return true;
                }
            }
            return false;
    }


};



























// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <cmath>
// #include <thread>
// #include <unordered_map>
// #include <opencv2/opencv.hpp> 


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include <geometry_msgs/msg/point.hpp>

// using namespace std::chrono_literals;
// using std::placeholders::_1;

// class Graph
// {
//     private:
//     int last_node=0;

//     bool is_connected(cv::Point2f pt1,cv::Point2f pt2,cv::Mat img)
//     {
//         return false;
//     }

//     public:
//     std::unordered_map<int,cv::Point2f> graph={};
//     Graph()
//     {
//         this->last_node=0;
//         this->graph={};
        

//     }

//     bool add_node(cv::Point2f pt, int no_of_pathways)
//     {
//         bool nearby_node_present=false;
//             if (this->last_node==0)
//             {
//                 this->graph[this->last_node]=pt;
//                 this->last_node++;
//                 return true;
//             }
//             else
//             {
//                 // std::cout<<this->graph.size()<<"Size of array"<<std::endl;
//                 for (int i=0;i<this->last_node;i++)
//                 {
//                     if(cv::norm(graph[i]-pt)<1.0)
//                     {
//                         nearby_node_present=true;
//                         break;
//                     }
                    
//                 }
//                 if(nearby_node_present==false)
//                 {   
//                     this->graph[this->last_node+1]=pt;
//                     this->last_node++;
//                     return true;
//                 }
//             }
//             return false;
//     }


// };

// class PathPlanner : public rclcpp::Node
// {
//   public:
//     PathPlanner(float goal_pose_x,float goal_pose_y)
//     : Node("bot_localizer_node")
//     {
//         RCLCPP_INFO(this->get_logger(), "Path Planner constructor called");
//         this->goal_pose_x=goal_pose_x;
//         this->goal_pose_y=goal_pose_y;
//         cv::Mat graph_img=cv::imread("/home/kuljot/Project/maze_solver/maze_solver/src/bot/maps/cropped_image.png",cv::IMREAD_GRAYSCALE);
//         RCLCPP_INFO(this->get_logger(), "Image Read %d %d ",graph_img.size);
//         location_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
//             "/bot_location_in_image", 10, std::bind(&PathPlanner::location_callback, this, _1));
//         cv::imshow("Recieved Graph",graph_img);
        
//         RCLCPP_INFO(this->get_logger(), "Show image");
//     }
//   private:
//     mutable cv::Point2f img_location;
//     cv::Mat graph_img;
//     float goal_pose_x;
//     float goal_pose_y;
//     float tollerance=5;
//     // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
//     rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr location_subscription_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;


//     void location_callback(const geometry_msgs::msg::Point & msg) const
//     {
//     // Show Image inside a window 
//       cv::Mat path_img;
//       geometry_msgs::msg::Point pt;
//       try
//       {
//         RCLCPP_INFO(this->get_logger(), "Location recieved X[%d] Y[%d]",msg.x,msg.y);
//         cv::imshow("Image",this->graph_img);
//       }
//       catch (cv_bridge::Exception &e)
//       {
//          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//       }
      
//       cv::waitKey(1);
        
//     }

//     geometry_msgs::msg::Point img_to_location(geometry_msgs::msg::Point pt)
//     {
//         geometry_msgs::msg::Point result;
//         result.z=0;
//         double M[2][2]={ 
//                 {0.0976, 0.0180},
//                 {-0.1371, -0.0254}};
//         result.x=pt.x*M[0][0]+pt.y*M[1][0];
//         result.y=pt.x*M[0][1]+pt.y*M[1][1];
//         return result;
//     }
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   float goal_pose_x=std::stof(argv[1]);
//   float goal_pose_y=std::stof(argv[2]);
//   rclcpp::spin(std::make_shared<PathPlanner>(goal_pose_x,goal_pose_y));
//   rclcpp::shutdown();
//   return 0;
// }