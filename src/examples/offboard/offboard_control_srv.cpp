/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples * 
 * @author Beniamino Pozzan <beniamino.pozzan@gmail.com>
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

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

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) :
		Node("offboard_control_srv"),
		state_{State::init},
		service_result_{0},
		service_done_{false},
		offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>(px4_namespace+"in/offboard_control_mode", 10)},
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(px4_namespace+"in/trajectory_setpoint", 10)},
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command")},
		vehicle_odometry_subscriber_{this->create_subscription<VehicleOdometry>(
			px4_namespace + "out/vehicle_odometry",
			rclcpp::SensorDataQoS(),
			[this](const VehicleOdometry::SharedPtr msg) {
				latest_odom_ = *msg;
				odom_received_ = true;
			})},
		control_mode_{this->declare_parameter<std::string>("control_mode", "position")},
		start_time_{this->now()}
		{
		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
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
	enum class State{
		init,
		offboard_requested,
		wait_for_stable_offboard_mode,
		arm_requested,
		armed
	} state_;

	struct TrajectoryReference {
		Eigen::Vector3d position;
		Eigen::Vector3d velocity;
		Eigen::Vector3d acceleration;
		float yaw;
	};

	uint8_t service_result_;
	bool service_done_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	// setup odometer subscription
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	VehicleOdometry latest_odom_{};
	bool odom_received_{false};

	// control mode
	std::string control_mode_;

	// init gain params/consts
	Eigen::Matrix3d K_p_ = 2.0 * Eigen::Matrix3d::Identity();
	Eigen::Matrix3d K_v_ = 1.5 * Eigen::Matrix3d::Identity();
	Eigen::Vector3d gravity_{0.0, 0.0, 9.81};

	// 
	Eigen::Vector3d p_d{0.0, 0.0, -5.0};
	Eigen::Vector3d v_d = Eigen::Vector3d::Zero();
	Eigen::Vector3d a_d = Eigen::Vector3d::Zero();

	// time
	rclcpp::Time start_time_;

	// methods
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timer_callback(void);
	TrajectoryReference compute_figure8_reference(double t_sec) const;
    Eigen::Vector3d compute_acceleration_command(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &p_d, const Eigen::Vector3d &v_d, const Eigen::Vector3d &a_d);
};

/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = (control_mode_ == "position");
	msg.velocity = false;
	msg.acceleration = (control_mode_ == "acceleration");
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = latest_odom_.timestamp;
	//msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        
 */
void OffboardControl::publish_trajectory_setpoint()
{

	if (!odom_received_) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Waiting for vehicle odometry");
		return;
	}

	const double t_sec = (this->now() - start_time_).seconds();
	const TrajectoryReference ref = compute_figure8_reference(t_sec);

	const Eigen::Vector3d p(
		latest_odom_.position[0],
		latest_odom_.position[1],
		latest_odom_.position[2]);

	const Eigen::Vector3d v(
		latest_odom_.velocity[0],
		latest_odom_.velocity[1],
		latest_odom_.velocity[2]);

	const Eigen::Vector3d a_cmd =
		compute_acceleration_command(p, v, ref.position, ref.velocity, ref.acceleration);

	TrajectorySetpoint msg{};
	msg.yaw = ref.yaw;
	msg.timestamp = latest_odom_.timestamp;

	Eigen::Vector3d p_d = ref.position;

	if (control_mode_ == "position") {
		msg.position = {
			static_cast<float>(p_d.x()),
			static_cast<float>(p_d.y()),
			static_cast<float>(p_d.z())
		};
	} else if (control_mode_ == "acceleration") {
		msg.position = {NAN, NAN, NAN};
		msg.velocity = {NAN, NAN, NAN};
		msg.acceleration = {
			static_cast<float>(a_cmd.x()),
			static_cast<float>(a_cmd.y()),
			static_cast<float>(a_cmd.z())
		};
	} else {
		RCLCPP_WARN(this->get_logger(),
		"Unknown control_mode '%s', falling back to position mode",
		control_mode_.c_str());
		msg.position = {
			static_cast<float>(p_d.x()),
			static_cast<float>(p_d.y()),
			static_cast<float>(p_d.z())
		};
	}

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	switch (state_)
	{
	case State::init :
		if (odom_received_) {
			switch_to_offboard_mode();
			state_ = State::offboard_requested;
		}	

		break;
	case State::offboard_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
				state_ = State::wait_for_stable_offboard_mode;				
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps>100) {
			arm();
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "vehicle is armed");
				state_ = State::armed;
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	default:
		break;
	}
}

void OffboardControl::response_callback(
      rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
	  auto reply = future.get()->reply;
	  service_result_ = reply.result;
      switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

OffboardControl::TrajectoryReference OffboardControl::compute_figure8_reference(double t_sec) const {
	const double A = 2.0;
	const double B = 1.0;
	const double omega = 0.4;
	const double z_ref = -5.0;

	const double s = std::sin(omega * t_sec);
	const double c = std::cos(omega * t_sec);

	TrajectoryReference ref{};

	ref.position = Eigen::Vector3d(
		A * s,
		B * s * c,
		z_ref);

	ref.velocity = Eigen::Vector3d(
		A * omega * c,
		B * omega * (c * c - s * s),
		0.0);

	ref.acceleration = Eigen::Vector3d(
		-A * omega * omega * s,
		-4.0 * B * omega * omega * s * c,
		0.0);

	ref.yaw = 0.0f;
	return ref;
}

Eigen::Vector3d OffboardControl::compute_acceleration_command(
	const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &p_d, 
	const Eigen::Vector3d &v_d, const Eigen::Vector3d &a_d) {
		const Eigen::Vector3d e_p = p - p_d;
		const Eigen::Vector3d e_v = v - v_d;
		const Eigen::Vector3d w = a_d - K_v_ * e_v - K_p_ * e_p;
		const Eigen::Vector3d a_cmd = w - gravity_;
		return a_cmd;
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));

	rclcpp::shutdown();
	return 0;
}
