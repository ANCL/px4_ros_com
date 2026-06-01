/**
 * @brief Trajectory publisher / path generator
 * @file trajectory_publisher.cpp
 * @author Dion Walton <ddwalton@ualberta.ca>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {

public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/custom/trajectory_reference", 10);
        
        // declare parameter
        flight_path_ = this->declare_parameter<std::string>("flight_path", "hover");
        
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TrajectoryPublisher::parameters_callback, this, std::placeholders::_1));

        // subscribe to vehicle status to detect when Offboard mode is triggered via RC
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                bool was_offboard = is_offboard_;
                is_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
                
                // if we just switched into offboard mode, reset the trajectory clock to 0
                if (is_offboard_ && !was_offboard) {
                    RCLCPP_INFO(this->get_logger(), "Offboard mode engaged, trajectory clock started!");
                    start_time_ = this->now();
                }
        });

        // add a timer to run at 50Hz (20ms)
        start_time_ = this->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    }
private:
    // internal struct for math
    struct TrajectoryReference {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        float yaw;
    };

    /**
     * @brief Handle dynamic parameter updates efficiently
     */
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "flight_path" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                flight_path_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "Flight path updated to: %s. Resetting clock.", flight_path_.c_str());
                start_time_ = this->now(); // Reset time so trajectories start cleanly
            }
        }
        return result;
    }

    void timer_callback() {
        double t_sec = (this->now() - start_time_).seconds();
        TrajectoryReference ref;
        
        if (flight_path_ == "figure8") {
            ref = compute_figure8_reference(t_sec);
        } else if (flight_path_ == "circle") {
            ref = compute_circle_reference(t_sec);
        } else if (flight_path_ == "helix") {
            ref = compute_helix_reference(t_sec);
        } else if (flight_path_ == "step") {
            ref = compute_step_reference(-1.5, -1.5, -1.5);
        } else if (flight_path_ == "hover") {
            ref = compute_step_reference(0.0, 0.0, -1.2);
        } else if (flight_path_ == "mission") {
            ref = compute_mission_reference(t_sec);
        } else {
            // Default to hover and throttle the warning
            ref = compute_step_reference(0.0, 0.0, -1.2);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Fallback to hover -- '%s' does not exist. Options: hover, step, figure8, circle, helix, mission", 
                flight_path_.c_str());
        }

        // map the struct data to the ROS message
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = { (float)ref.position.x(), (float)ref.position.y(), (float)ref.position.z() };
        msg.velocity = { (float)ref.velocity.x(), (float)ref.velocity.y(), (float)ref.velocity.z() };
        msg.acceleration = { (float)ref.acceleration.x(), (float)ref.acceleration.y(), (float)ref.acceleration.z() };
        msg.yaw = ref.yaw;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        publisher_->publish(msg);
    }

    TrajectoryReference compute_figure8_reference(double t_sec) const {
        const double A = 2.0;
        const double B = 1.0;
        const double omega = 0.4;
        const double z_ref = -1.2;
        
        const double s = std::sin(omega * t_sec);
        const double c = std::cos(omega * t_sec);
        
        TrajectoryReference ref{};
        ref.position = Eigen::Vector3d(A * s, B * s * c, z_ref);
        ref.velocity = Eigen::Vector3d(A * omega * c, B * omega * (c * c - s * s), 0.0);
        ref.acceleration = Eigen::Vector3d(-A * omega * omega * s, -4.0 * B * omega * omega * s * c, 0.0);
        ref.yaw = 0.0f;
        
        return ref;
    }

    TrajectoryReference compute_circle_reference(double t_sec) const {
        const double R = 2.0;
        const double omega = 0.4;
        const double z_ref = -1.2;
        
        const double x = R * std::cos(omega * t_sec);
        const double y = R * std::sin(omega * t_sec);
        
        TrajectoryReference ref{};
        ref.position = Eigen::Vector3d(x, y, z_ref);
        ref.velocity = Eigen::Vector3d((omega * (-y)), (omega * x), 0.0);
        ref.acceleration = Eigen::Vector3d((omega * omega * (-x)), (omega * omega * (-y)), 0.0);
        ref.yaw = 0.0f;
        
        return ref;
    }

    TrajectoryReference compute_helix_reference(double t_sec) const {
        const double R = 2.0;
        const double omega = 0.4;
        const double g = -0.2;

        const double x = R * std::cos(omega * t_sec);
        const double y = R * std::sin(omega * t_sec);

        const double max_z = -2.0;
        // clamp z so it doesn't fly into space
        double z = g * t_sec;
        double z_vel = g;
        if (z <= max_z) {
            z = max_z;
            z_vel = 0.0;
        }
        
        TrajectoryReference ref{};
        ref.position = Eigen::Vector3d(x, y, z);
        ref.velocity = Eigen::Vector3d((omega * (-y)), (omega * x), z_vel);
        ref.acceleration = Eigen::Vector3d((omega * omega * (-x)), (omega * omega * (-y)), 0.0);
        ref.yaw = 0.0f;
        
        return ref;
    }

    TrajectoryReference compute_mission_reference(double t_sec) {
        TrajectoryReference ref{};
        const double z = -1.2;

        ref.velocity = Eigen::Vector3d(NAN, NAN, NAN);
        ref.acceleration = Eigen::Vector3d(NAN, NAN , NAN);
        ref.yaw = 0.0f;

        // wrap the time to a 20-second repeating period
        double t_mod = std::fmod(t_sec, 20.0);

        if (t_mod < 5.0) {
            ref.position = Eigen::Vector3d(1.0, 0.0, z);
        } else if (t_mod < 10.0) {
            ref.position = Eigen::Vector3d(0.0, 1.0, z);
        } else if (t_mod < 15.0) {
            ref.position = Eigen::Vector3d(-1.0, 0.0, z);
        } else {
            ref.position = Eigen::Vector3d(0.0, -1.0, z);
        }

        return ref;
    }

    TrajectoryReference compute_step_reference(const double x, const double y, const double z) {
        TrajectoryReference ref{};
        
        ref.position = Eigen::Vector3d(x, y, z);
        ref.velocity = Eigen::Vector3d(NAN, NAN, NAN);
        ref.acceleration = Eigen::Vector3d(NAN, NAN, NAN);
        ref.yaw = 0.0f;

        return ref;
    }

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::string flight_path_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    bool is_offboard_{false};

    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}