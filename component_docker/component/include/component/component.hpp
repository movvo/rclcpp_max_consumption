#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

// cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

// rclcpp
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"

//own
#include "component/composition_visibility.h"



namespace component {

class Component
{
public:
	COMPOSITION_PUBLIC explicit Component(const rclcpp::NodeOptions & options);
    ~Component();
    
    COMPOSITION_PUBLIC
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
	get_node_base_interface() const;

private:

    /*!
     * @brief Callback function of the service to change state of component 
     (Active or Inactive)
     * @param[in] request Request from client
     * @param[in] response Response to client
     */
    void TransitionCallback(const std::shared_ptr
        <std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /*!
     * @brief Function to apply the logics to Activate the Component
     */
    bool Activate();

    /*!
     * @brief Function to apply the logics to Deactivate the Component
     */
    bool Deactivate();

    int IntenseWork();

	rclcpp::Node::SharedPtr node_;
    
    // Activate/Deactivate
    bool activated_; /*< Boolean to indicate if the Component is active or not */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr transition_srv_; 
        /*< Service to trigger the transition of activation/deactivation */

};

}  // namespace component

#endif  // COMPONENT_HPP_