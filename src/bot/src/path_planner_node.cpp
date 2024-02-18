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
