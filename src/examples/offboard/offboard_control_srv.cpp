#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl(std::string px4_namespace) :
        Node("offboard_control_srv"),
        state_{State::init},
        service_result_{0},
        service_done_{false},
        offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", 10)},
        trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", 10)},
        vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command")} 
    {
        control_mode_ = this->declare_parameter<std::string>("control_mode", "position");
        start_time_ = this->now();

        // Odometry subscription
        vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_namespace + "out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                latest_odometry_ = *msg;
                odom_received_ = true;
            });

        // Trajectory subscription
        trajectory_subscriber_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/trajectory_generator/reference", 10,
            [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
                target_setpoint_ = *msg;
                target_received_ = true;
            });

        RCLCPP_INFO(this->get_logger(), "Starting Offboard Control. Subscribed to external trajectory.");

        while (!vehicle_command_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        timer_ = this->create_wall_timer(10ms, std::bind(&OffboardControl::timer_callback, this));
    }

    void switch_to_offboard_mode();
    void arm();
    void disarm();

private:
    enum class State { init, offboard_requested, wait_for_stable_offboard_mode, arm_requested, armed } state_;
    uint8_t service_result_;
    bool service_done_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_subscriber_;
    
    px4_msgs::msg::TrajectorySetpoint target_setpoint_{};
    bool target_received_{false};

    px4_msgs::msg::VehicleOdometry latest_odometry_{};
    bool odom_received_{false};

    std::string control_mode_;
    Eigen::Matrix3d K_p_ = 1 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d K_v_ = 0.8 * Eigen::Matrix3d::Identity();
    rclcpp::Time start_time_;

    Eigen::Vector3d compute_acceleration_command(
        const Eigen::Vector3d &p, const Eigen::Vector3d &v,
        const Eigen::Vector3d &p_d, const Eigen::Vector3d &v_d,
        const Eigen::Vector3d &a_d) const 
    {
        const Eigen::Vector3d e_p = p - p_d;
        const Eigen::Vector3d e_v = v - v_d;
        return a_d - K_v_ * e_v - K_p_ * e_p;
    }

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timer_callback(void);
};

void OffboardControl::switch_to_offboard_mode() {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControl::arm() {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void OffboardControl::disarm() {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void OffboardControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = (control_mode_ == "position");
    msg.velocity = false;
    msg.acceleration = (control_mode_ == "acceleration");
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() {
    if (!odom_received_ || !target_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odom and Trajectory...");
        return;
    }

    const Eigen::Vector3d p(latest_odometry_.position[0], latest_odometry_.position[1], latest_odometry_.position[2]);
    const Eigen::Vector3d v(latest_odometry_.velocity[0], latest_odometry_.velocity[1], latest_odometry_.velocity[2]);

    const Eigen::Vector3d p_d(target_setpoint_.position[0], target_setpoint_.position[1], target_setpoint_.position[2]);
    const Eigen::Vector3d v_d(target_setpoint_.velocity[0], target_setpoint_.velocity[1], target_setpoint_.velocity[2]);
    const Eigen::Vector3d a_d(target_setpoint_.acceleration[0], target_setpoint_.acceleration[1], target_setpoint_.acceleration[2]);

    const Eigen::Vector3d a_cmd = compute_acceleration_command(p, v, p_d, v_d, a_d);

    TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    msg.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    msg.velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    msg.acceleration = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};

    msg.yaw = target_setpoint_.yaw;

    if (control_mode_ == "position") {
        msg.position = {(float)p_d.x(), (float)p_d.y(), (float)p_d.z()};
    } else {
        // When in acceleration mode, we MUST provide acceleration
        msg.acceleration = {(float)a_cmd.x(), (float)a_cmd.y(), (float)a_cmd.z()};
    }

    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2) {
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    VehicleCommand msg{};
    msg.param1 = param1; msg.param2 = param2; msg.command = command;
    msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    request->request = msg;
    service_done_ = false;
    vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this, std::placeholders::_1));
}

void OffboardControl::timer_callback(void) {
    static uint8_t num_of_steps = 0;
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    switch (state_) {
        case State::init: 
            if (odom_received_) { 
                switch_to_offboard_mode(); 
                state_ = State::offboard_requested; 
            } 
            break;
        case State::offboard_requested: 
            if (service_done_) { 
                if (service_result_ == 0) state_ = State::wait_for_stable_offboard_mode; 
                else rclcpp::shutdown(); 
            } 
            break;
        case State::wait_for_stable_offboard_mode: 
            if (++num_of_steps > 10) { 
                arm(); 
                state_ = State::arm_requested; 
            } 
            break;
        case State::arm_requested: 
            if (service_done_) { 
                if (service_result_ == 0) state_ = State::armed; 
                else rclcpp::shutdown(); 
            } 
            break;
        default: break;
    }
}

void OffboardControl::response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    if (future.wait_for(1s) == std::future_status::ready) {
        service_result_ = future.get()->reply.result;
        service_done_ = true;
    }
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));
    rclcpp::shutdown();
    return 0;
}