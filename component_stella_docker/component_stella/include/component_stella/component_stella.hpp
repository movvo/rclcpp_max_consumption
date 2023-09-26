#ifndef COMPONENT_STELLA_HPP_
#define COMPONENT_STELLA_HPP_

// cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

// stella_vslam_ros
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/converter.h>
#include <stella_vslam/util/stereo_rectifier.h>
#include <stella_vslam/publish/map_publisher.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Own
#include "component_stella/composition_visibility.h"
#include "std_srvs/srv/trigger.hpp"

#include "spdlog/spdlog.h"
// Test if necessary dependecies

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
// Final needed depencies to test

enum TRACKING_STATE {
    Initializing = 0,
    Tracking,
    Lost
};

namespace component_stella {

class ComponentStella
{
public:
	COMPOSITION_PUBLIC explicit ComponentStella(const rclcpp::NodeOptions & options);
    ~ComponentStella();
    
    COMPOSITION_PUBLIC
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
	get_node_base_interface() const;

protected:
    void setParams();
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:

    /*!
     * @brief Callback function of the service to change state of component (Active or Inactive)
     * @param[in] request Request from client
     * @param[in] response Response to client
     */
    void TransitionCallback(const std::shared_ptr
        <std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Test
    rclcpp::Time last_;
    std::vector<double> frequencies_;
    std::shared_ptr<stella_vslam::publish::map_publisher> map_pub_;


	rclcpp::Node::SharedPtr node_;
    std::string cfg_path_;
    std::string vocab_file_path_;
    std::string mask_img_path_;
    std::string map_db_path_;
    bool rectify_;
	std::string status_topic_;
	std::string image_topic_;

    uint8_t last_track_state_;
    
    // Activate/Deactivate
    bool activated_; /*< Boolean to indicate if the Component is active or not */
    bool perform_action_ = false;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr transition_srv_; /*< Service to trigger the transition of activation/deactivation */

	// Stella_vslam_ros
    std::shared_ptr<stella_vslam::system> SLAM_;
    rclcpp::QoS custom_qos_;

    cv::Mat mask_;
    std::string odom_frame_, map_frame_, base_link_;
    std::string camera_frame_, camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    bool publish_tf_;
    double transform_tolerance_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace ikaros

#endif  // IKAROS_COMPONENT_HPP_