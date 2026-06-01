/**
 * @brief Path Visualizer
 * @file path_visualizer.cpp
 * @author Dion Walton <ddwalton@ualberta.ca>>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleLocalPosition;

class PathVisualizer : public rclcpp::Node {
public:
    PathVisualizer() : Node("path_visualizer_node") {
        reference_position_subscriber_ =  this->create_subscription<TrajectorySetpoint>(
            "custom/trajectory_reference",
            10,
            [this](const TrajectorySetpoint::SharedPtr msg) {
                latest_reference_pos_ = *msg;
                ref_pos_received_ = true;
            }
        );

        actual_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 
            rclcpp::SensorDataQoS(),
            [this](const VehicleLocalPosition::SharedPtr msg) {
                latest_actual_pos_ = *msg;
                actual_pos_received_ = true;

                if (ref_pos_received_) {
                    update_viz();
                }
            }
        );

        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("viz/actual_path", 10);
        ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("viz/ref_path", 10);

        actual_path_msg_.header.frame_id = "map";
        ref_path_msg_.header.frame_id = "map";
    }

private:
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr reference_position_subscriber_;
    TrajectorySetpoint latest_reference_pos_{};
    bool ref_pos_received_{false};

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr actual_position_subscriber_;
    VehicleLocalPosition latest_actual_pos_{};
    bool actual_pos_received_{false};

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;

    nav_msgs::msg::Path actual_path_msg_;
    nav_msgs::msg::Path ref_path_msg_;

    const size_t MAX_PATH_LENGTH = 2000;

    void update_viz() {
        Eigen::Vector3d p(latest_actual_pos_.x, latest_actual_pos_.y, latest_actual_pos_.z);
        Eigen::Vector3d p_d(latest_reference_pos_.position[0], latest_reference_pos_.position[1], latest_reference_pos_.position[2]);
        publish_paths(p, p_d);
    }

    void publish_paths(const Eigen::Vector3d &p, const Eigen::Vector3d &p_d) {
        rclcpp::Time now = this->now();
    
        auto create_pose = [&](const Eigen::Vector3d &vec) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = now;
            ps.header.frame_id = "map";
            ps.pose.position.x = vec.y();  // East
            ps.pose.position.y = vec.x();  // North
            ps.pose.position.z = -vec.z(); // Up
            return ps;
        };
    
        actual_path_msg_.poses.push_back(create_pose(p));
        ref_path_msg_.poses.push_back(create_pose(p_d));
    
        if (actual_path_msg_.poses.size() > MAX_PATH_LENGTH) {
            actual_path_msg_.poses.erase(actual_path_msg_.poses.begin());
            ref_path_msg_.poses.erase(ref_path_msg_.poses.begin());
        }
    
        actual_path_msg_.header.stamp = now;
        ref_path_msg_.header.stamp = now;
    
        actual_path_pub_->publish(actual_path_msg_);
        ref_path_pub_->publish(ref_path_msg_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}