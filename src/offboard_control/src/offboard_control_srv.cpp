/**
 * @brief Offboard controller
 * @file offboard_control_srv.cpp
 * @author Dion Walton <ddwalton@ualberta.ca>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl(std::string px4_namespace) :
        Node("offboard_control_srv"),
        control_mode_("position"),
        kp_(2.0),
        kv_(1.5)
    {
        // Declare Parameters
        this->declare_parameter<std::string>("control_mode", control_mode_);
        this->declare_parameter<double>("Kp", kp_);
        this->declare_parameter<double>("Kv", kv_);
        
        // Initialize gain matrices once
        K_p_ = kp_ * Eigen::Matrix3d::Identity();
        K_v_ = kv_ * Eigen::Matrix3d::Identity();

        // Setup Parameter Callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&OffboardControl::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Starting Offboard Control Node in Reactive Mode.");

        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
            px4_namespace + "in/offboard_control_mode", rclcpp::SensorDataQoS());
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
            px4_namespace + "in/trajectory_setpoint", rclcpp::SensorDataQoS());
        debug_trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
            "debug/trajectory_setpoint", rclcpp::SensorDataQoS());

        // Subscribers
        vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            px4_namespace + "out/vehicle_local_position",
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                if (!msg->xy_valid || !msg->z_valid || !msg->v_xy_valid || !msg->v_z_valid) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "EKF2 Local position invalid. Ignoring.");
                    return;
                }
                latest_local_pos_ = *msg;
                pos_received_ = true; 
            });
    
        trajectory_ref_subscriber_ = this->create_subscription<TrajectorySetpoint>(
            "custom/trajectory_reference",
            10,
            [this](const TrajectorySetpoint::SharedPtr msg) {
                latest_ref_ = *msg;
                ref_received_ = true;
            });

        // Note: Using the _v1 topic to match PX4's internal uORB updates
        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                bool was_offboard = is_offboard_;
                is_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
                
                if (is_offboard_ && !was_offboard) {
                    RCLCPP_INFO(this->get_logger(), "Offboard mode engaged. Executing trajectory.");
                    start_time_ = this->now();
                } else if (!is_offboard_ && was_offboard) {
                    RCLCPP_INFO(this->get_logger(), "Offboard mode disengaged.");
                }
            });

        timer_ = this->create_wall_timer(10ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Publishers & Subscribers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr debug_trajectory_setpoint_publisher_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_ref_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

    // State Variables
    px4_msgs::msg::VehicleLocalPosition latest_local_pos_{};
    TrajectorySetpoint latest_ref_;
    bool pos_received_{false};
    bool ref_received_{false};
    bool is_offboard_{false};
    rclcpp::Time start_time_;

    // Control Parameters
    std::string control_mode_;
    double kp_;
    double kv_;
    Eigen::Matrix3d K_p_;
    Eigen::Matrix3d K_v_;

    // Methods
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void timer_callback(void);
    Eigen::Vector3d compute_acceleration_command(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &p_d, const Eigen::Vector3d &v_d, const Eigen::Vector3d &a_d);
};

/**
 * @brief Handle dynamic parameter updates efficiently
 */
rcl_interfaces::msg::SetParametersResult OffboardControl::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters) {
        if (param.get_name() == "control_mode" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            control_mode_ = param.as_string();
            RCLCPP_INFO(this->get_logger(), "Control mode updated to: %s", control_mode_.c_str());
        } 
        else if (param.get_name() == "Kp" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            kp_ = param.as_double();
            K_p_ = kp_ * Eigen::Matrix3d::Identity();
            RCLCPP_INFO(this->get_logger(), "Kp updated to: %f", kp_);
        } 
        else if (param.get_name() == "Kv" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            kv_ = param.as_double();
            K_v_ = kv_ * Eigen::Matrix3d::Identity();
            RCLCPP_INFO(this->get_logger(), "Kv updated to: %f", kv_);
        }
    }
    return result;
}

/**
 * @brief Publish the offboard control mode flags.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = (control_mode_ == "position");
    msg.velocity = false;
    msg.acceleration = (control_mode_ == "acceleration");
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the computed trajectory setpoint
 */
void OffboardControl::publish_trajectory_setpoint()
{
    if (!pos_received_ || !ref_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Waiting for vehicle local position and trajectory reference");
        return;
    }   
    
    const Eigen::Vector3d p(latest_local_pos_.x, latest_local_pos_.y, latest_local_pos_.z);
    const Eigen::Vector3d v(latest_local_pos_.vx, latest_local_pos_.vy, latest_local_pos_.vz);
            
    Eigen::Vector3d p_ref(latest_ref_.position[0], latest_ref_.position[1], latest_ref_.position[2]);
    Eigen::Vector3d v_ref(latest_ref_.velocity[0], latest_ref_.velocity[1], latest_ref_.velocity[2]);
    Eigen::Vector3d a_ref(latest_ref_.acceleration[0], latest_ref_.acceleration[1], latest_ref_.acceleration[2]);

    const Eigen::Vector3d a_cmd = compute_acceleration_command(p, v, p_ref, v_ref, a_ref);
    uint64_t timestamp = this->get_clock()->now().nanoseconds() / 1000;

    TrajectorySetpoint msg{};
    msg.yaw = latest_ref_.yaw;
    msg.timestamp = timestamp;

    TrajectorySetpoint debug_msg{};
    debug_msg.yaw = latest_ref_.yaw;
    debug_msg.timestamp = timestamp;

    if (control_mode_ == "position") {
        msg.position = {static_cast<float>(p_ref.x()), static_cast<float>(p_ref.y()), static_cast<float>(p_ref.z())};
        msg.velocity = {NAN, NAN, NAN};
        msg.acceleration = {NAN, NAN, NAN};

        debug_msg.position = {NAN, NAN, NAN};
        debug_msg.velocity = {NAN, NAN, NAN};
        debug_msg.acceleration = {static_cast<float>(a_cmd.x()), static_cast<float>(a_cmd.y()), static_cast<float>(a_cmd.z())};
        debug_trajectory_setpoint_publisher_->publish(debug_msg);

    } else if (control_mode_ == "acceleration") {
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {NAN, NAN, NAN};
        msg.acceleration = {static_cast<float>(a_cmd.x()), static_cast<float>(a_cmd.y()), static_cast<float>(a_cmd.z())};
        
        debug_msg.position = {NAN, NAN, NAN};
        debug_msg.velocity = {NAN, NAN, NAN};
        debug_msg.acceleration = {NAN, NAN, NAN};
        debug_trajectory_setpoint_publisher_->publish(debug_msg);

    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Unknown control_mode '%s', falling back to position mode", control_mode_.c_str());
        msg.position = {static_cast<float>(p_ref.x()), static_cast<float>(p_ref.y()), static_cast<float>(p_ref.z())};
        msg.velocity = {NAN, NAN, NAN};
        msg.acceleration = {NAN, NAN, NAN};
    }

    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::timer_callback(void){
    publish_offboard_control_mode();

    if (is_offboard_) {
		// only publish setpoint in offboard mode
        publish_trajectory_setpoint();
	}
}

Eigen::Vector3d OffboardControl::compute_acceleration_command(
    const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &p_d, 
    const Eigen::Vector3d &v_d, const Eigen::Vector3d &a_d) {
        
        const Eigen::Vector3d e_p = p - p_d;
        const Eigen::Vector3d e_v = v - v_d;
        return a_d - K_v_ * e_v - K_p_ * e_p; // gravity compensation already accounted for
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto offboard_node = std::make_shared<OffboardControl>("/fmu/");
    rclcpp::spin(offboard_node);
    rclcpp::shutdown();
    return 0;
}