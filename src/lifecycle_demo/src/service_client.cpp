#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

static constexpr char const * talkerNode="lc_talker";
static constexpr char const * nodeGetStateTopic="lc_talker/get_state";
static constexpr char const * nodeChangeStateTopic="lc_talker/change_state";

template <typename FutureT, typename WaitTimeT>
std::future_status
waitForResult(FutureT & future,WaitTimeT & timeout)
{
    auto end =std::chrono::steady_clock::now()+timeout;
    std::chrono::milliseconds waitPeriod(100);
    std::future_status status=std::future_status::timeout;

    do{
        auto now=std::chrono::steady_clock::now();
        auto timeLeft= end-now;

        if (timeLeft<=std::chrono::seconds(0))
        {
            break;
        }

        status=future.wait_for((timeLeft<waitPeriod)?timeLeft:waitPeriod);
    }
    while(rclcpp::ok() && status!=std::future_status::ready);
    return status;
}

class ServiceClient : public rclcpp::Node
{
public:
    explicit ServiceClient(const std::string & nodeName)
    :Node(nodeName)
    {
        clientGetState=this->create_client<lifecycle_msgs::srv::GetState>(nodeGetStateTopic);
        clientChangeState=this->create_client<lifecycle_msgs::srv::ChangeState>(nodeChangeStateTopic);
    }

    unsigned int get_state(std::chrono::seconds timeout = 3s)
    {
        auto request=std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if(!clientGetState->wait_for_service(timeout))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s not available",
                clientGetState->get_service_name()
            );
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto futureResult=clientGetState->async_send_request(request);
        auto futureStatus=waitForResult(futureResult,timeout);

        if(futureResult.get()){
            auto state=futureResult.get()->current_state.id;
            RCLCPP_INFO(
                get_logger(),
                "Node %s has current state %s",
                talkerNode,
                futureResult.get()->current_state.label.c_str()
            );
            return state;
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to get the current state of the node %s",
                talkerNode
            );
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }
    
    bool change_state(std::uint8_t transition, std::chrono::seconds timeout=3s) 
    {
        auto request=std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id=transition;

        if(!clientChangeState->wait_for_service(timeout))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s not available",
                clientGetState->get_service_name()
            );
            return false;
        }
        auto futureResult=clientChangeState->async_send_request(request);
        auto futureStatus=waitForResult(futureResult,timeout);

        if(futureStatus!=std::future_status::ready){
            // auto state=futureResult.get()->;
            RCLCPP_ERROR(
                get_logger(),
                "Server Timed out while getting current state of node %s",
                talkerNode
            );
            return false;
        }
        if(futureResult.get()->success){
            RCLCPP_INFO(
                get_logger(),
                "Transition %d successfully triggered",
                static_cast<unsigned int>(transition) 
            );
            return true;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Failed to trigger transition %d for node %s",
                static_cast<unsigned int>(transition),
                talkerNode
            );
            return false;
        }
        
    }
private:
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> clientGetState;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> clientChangeState;

};

void calee_script(std::shared_ptr<ServiceClient> serviceClient)
{
    rclcpp::WallRate stateChangeTime(0.1); //10s

    //configure
    {
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }

    //activate
    {
        stateChangeTime.sleep();
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }

    //deactivate
    {
        stateChangeTime.sleep();
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }

    //activate
    {
        stateChangeTime.sleep();
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }

    //deactivate
    {
        stateChangeTime.sleep();
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }

    //cleanup
    {
        stateChangeTime.sleep();
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }

    //shutdown
    {
        stateChangeTime.sleep();
        if(!serviceClient->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
        {
            return;
        }

        if(!serviceClient->get_state())
        {
            return;
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    auto serviceClient=std::make_shared<ServiceClient>("serviceClient");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(serviceClient);

    std::shared_future<void> script = std::async(
        std::launch::async,
        std::bind(calee_script,serviceClient)
    );

    executor.spin_until_future_complete(script);
    // script.get();  // Wait for script to complete

    rclcpp::shutdown();  // Shutdown ROS
    return 0;
}