#include "component_stella/component_stella.hpp"
#include <cinttypes>


using namespace component_stella;
using namespace std::literals;
using std::placeholders::_1;
using std::placeholders::_2;

ComponentStella::ComponentStella(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("component_stella", options)), 
    custom_qos_(1)
{
    rclcpp::uninstall_signal_handlers();

    // Declare and get parameters
    setParams();

    // load configuration
    std::shared_ptr<stella_vslam::config> cfg;

    try {
        cfg = std::make_shared<stella_vslam::config>(cfg_path_);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "failed to ser stella cfg file");
        std::cerr << e.what() << std::endl;
    }


    SLAM_ = std::make_shared<stella_vslam::system>(cfg, vocab_file_path_);
    map_pub_ = SLAM_->get_map_publisher();

    mask_ = mask_img_path_.empty() ? cv::Mat{} : cv::imread(mask_img_path_, 
        cv::IMREAD_GRAYSCALE);

    activated_ = true;
    custom_qos_.best_effort();
    custom_qos_.keep_last(1);
    custom_qos_.durability_volatile();

    sub_ = node_->create_subscription<sensor_msgs::msg::Image>(image_topic_,
        custom_qos_, std::bind(&ComponentStella::img_callback, this, _1));
    
    // load the prebuilt map
    SLAM_->load_map_database(map_db_path_);
    // startup the SLAM process (it does not need initialization of a map)
    SLAM_->startup(false);

    // Localization Only mode
    SLAM_->disable_mapping_module();
    SLAM_->disable_loop_detector();

    transition_srv_ = node_->create_service<std_srvs::srv::Trigger>
        (node_->get_name()+"/change_state"s, std::bind(
            &ComponentStella::TransitionCallback, this, _1, _2));

    // // Init last track state
    // last_track_state_ = TRACKING_STATE::Initializing;

    // last_ = node_->now();
}

ComponentStella::~ComponentStella() {
    SLAM_->shutdown();
}

void ComponentStella::setParams() 
{
    image_topic_ = node_->declare_parameter<std::string>("image_topic", 
        "/camera/image_raw");

    status_topic_ = node_->declare_parameter<std::string>("status_topic", 
        "/status");

    cfg_path_ = node_->declare_parameter<std::string>("cfg_path", 
        "/config/config_mono.yaml");

    vocab_file_path_ = node_->declare_parameter<std::string>("vocab_file", 
        "/vocab/orb_vocab.fbow");

    mask_img_path_ = node_->declare_parameter<std::string>("mask_img_path", 
        "");

    std::string map_folder = node_->declare_parameter<std::string>(
        "map_folder_path", "/maps");

    std::string map = node_->declare_parameter<std::string>("map",
        "test_map.db");

    map_db_path_ = map_folder + "/" + map;

    rectify_ = node_->declare_parameter<bool>("rectify", false);

    odom_frame_ = node_->declare_parameter<std::string>("odom_frame",
        "odom");

    map_frame_ = node_->declare_parameter<std::string>("map_frame",
        "map");

    base_link_ = node_->declare_parameter<std::string>("base_link",
        "base_footprint");

    camera_frame_ = node_->declare_parameter<std::string>("camera_frame",
        "camera_frame");

    // Set publish_tf to false if not using TF
    publish_tf_ = node_->declare_parameter<bool>("publish_tf", false);

    // Publish pose's timestamp in the future
    transform_tolerance_ = node_->declare_parameter<double>(
        "transform_tolerance", 0.5);
}

void ComponentStella::TransitionCallback
    (const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (activated_)
    {
        RCLCPP_INFO(node_->get_logger(), "Transition to deactivation");
        activated_ = false;
        response->message = "Deactivated";
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Transition to activation");
        activated_ = true;
        response->message = "Activated";
    }
    response->success = true;
    RCLCPP_INFO(node_->get_logger(), "Transition completed with %s", 
    response->success ? "SUCCESS" : "FAILURE");
}

void ComponentStella::img_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
{
    if (activated_)
    {
        if (camera_optical_frame_.empty()) {
            camera_optical_frame_ = msg->header.frame_id;
        }
        const rclcpp::Time tp_1 = node_->now();
        const double timestamp = tp_1.seconds();

        // input the current frame and estimate the camera pose
        auto cam_pose_wc = SLAM_->feed_monocular_frame(
            cv_bridge::toCvShare(msg)->image, timestamp, mask_);

        uint8_t track_state = SLAM_->get_tracking_state();
        RCLCPP_DEBUG(node_->get_logger(), "Track state: %s", 
            std::to_string(track_state).c_str());

    }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
ComponentStella::get_node_base_interface() const
{
    return this->node_->get_node_base_interface();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be 
//discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(component_stella::ComponentStella)