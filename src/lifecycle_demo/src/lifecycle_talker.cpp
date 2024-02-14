#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class LifecycleTalker :public rclcpp_lifecycle::LifecycleNode
{
private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher;
    std::shared_ptr<rclcpp::TimerBase> timer;

public:
    explicit LifecycleTalker(const std::string & nodeName, bool intraProcessComms=false)
    : rclcpp_lifecycle::LifecycleNode(
        nodeName,
        rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms)
    )
    {
    }

    void publish()
    {
        static size_t count=1;
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data="Lifecycle Talker"+std::to_string(++count);

        if(!publisher->is_activated()){
            RCLCPP_INFO(get_logger(),"publisher inactive");
        }
        else{
            RCLCPP_INFO(get_logger(),"publisher active. Publishing at [%s]", msg->data.c_str());
        }


        publisher->publish(std::move(msg));
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        publisher=this->create_publisher<std_msgs::msg::String>("messages",10);
        timer=this->create_wall_timer(1s, std::bind(&LifecycleTalker::publish,this));

        RCLCPP_INFO(get_logger(),"on_configure() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        publisher->on_activate();

        RCLCPP_INFO(get_logger(),"on_activate() called");
        std::this_thread::sleep_for(2s);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        publisher->on_activate();

        RCLCPP_INFO(get_logger(),"on_deactivate() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        publisher.reset();
        timer.reset();

        RCLCPP_INFO(get_logger(),"on_cleanup() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
    
        RCLCPP_INFO(get_logger(),"on_shutdown() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
   

};

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<LifecycleTalker> lc_talker_node=std::make_shared<LifecycleTalker>("lc_talker");
    executor.add_node(lc_talker_node->get_node_base_interface()); //Required for Lifecycle nodes

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
