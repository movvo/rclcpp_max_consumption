#include "component/component.hpp"
#include <cinttypes>


using namespace component;
using namespace std::literals;
using std::placeholders::_1;
using std::placeholders::_2;

Component::Component(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("component", options))
{
    rclcpp::uninstall_signal_handlers();

    transition_srv_ = node_->create_service<std_srvs::srv::Trigger>
        (node_->get_name()+"/change_state"s, std::bind(&Component::TransitionCallback, 
            this, _1, _2));
}

Component::~Component() {
}


void Component::TransitionCallback
    (const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (activated_)
    {
        RCLCPP_INFO(node_->get_logger(), "Transition to deactivation");
        activated_ = false;
        thread_obj.join();
        response->message = "Deactivated";
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Transition to activation");
        activated_ = true;
        thread_obj = std::thread(&Component::IntenseWork, this);
        response->message = "Activated";
    }
    response->success = true;
    RCLCPP_INFO(node_->get_logger(), "Transition completed with %s", response->success ? "SUCCESS" : "FAILURE");
}

int Component::IntenseWork()
{
    RCLCPP_INFO(node_->get_logger(), "Node %s high consumption process started", node_->get_name());
    while (activated_) {
        // Perform a CPU-intensive operation
        double result = 0.0;
        for (int i = 0; i < 1000000000; i++) {
            result += std::sqrt(i) * std::sin(i);
        }

        // Print something to indicate that the CPU is busy
        RCLCPP_DEBUG(node_->get_logger(), "Node %s working", node_->get_name());
    }
    RCLCPP_INFO(node_->get_logger(), "Node %s high consumption process finished", node_->get_name());
    return 0;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
Component::get_node_base_interface() const
{
    return this->node_->get_node_base_interface();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(component::Component)