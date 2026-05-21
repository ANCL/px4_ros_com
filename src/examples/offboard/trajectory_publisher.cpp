#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <chrono>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {

public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/custom/trajectory_reference", 10);
        flight_path_ = this->declare_parameter<std::string>("flight_path", "figure8");

        // add a timer to run at 50Hz (20ms)
        start_time_ = this->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    }
private:

    // internal strtuct for math
    struct TrajectoryReference {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        float yaw;
    };

    void timer_callback() {
        double t_sec = (this->now() - start_time_).seconds();
        
        // compute the math
        TrajectoryReference ref;
        
        if (flight_path_ == "figure8") {
            ref = compute_figure8_reference(t_sec);
        } else if (flight_path_ == "circle") {
            ref = compute_circle_reference(t_sec);
        } else if (flight_path_ == "helix") {
            ref = compute_helix_reference(t_sec);
        } else if (flight_path_ == "step") {
            ref = compute_step_reference(t_sec);
        } else {
            ref = compute_figure8_reference(t_sec);
            RCLCPP_INFO(this->get_logger(), "Fallback to figure8 reference -- flight_path does not exist\nOptions:\nfigure8, circle, etc.");
        }

        // Map the struct data to the ROS message
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
        const double z_ref = -5.0;
        
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
        const double R = 3.0;
        const double omega = 0.4;
        const double z_ref = -5.0;
        
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
        const double g = -1.0;

        const double x = R * std::cos(omega * t_sec);
        const double y = R * std::sin(omega * t_sec);
        const double z = g * t_sec;

        TrajectoryReference ref{};
        ref.position = Eigen::Vector3d(x, y, z);
        ref.velocity = Eigen::Vector3d((omega * (-y)), (omega * x), g);
        ref.acceleration = Eigen::Vector3d((omega * omega * (-x)), (omega * omega * (-y)), 0.0);
        ref.yaw = 0.0f;
        
        return ref;
    }

    TrajectoryReference compute_step_reference(double t_sec) const {
        //static t_0 = t_sec;
        TrajectoryReference ref{};
        
        const double x = -5.0;
        const double y = -5.0;
        const double z = -5.0;
        
        ref.position = Eigen::Vector3d(x, y, z);
        ref.velocity = Eigen::Vector3d(0, 0, 0);
        ref.acceleration = Eigen::Vector3d(0, 0 , 0);
        ref.yaw = 0.0f;

        
        return ref;
    }

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
    std::string flight_path_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}