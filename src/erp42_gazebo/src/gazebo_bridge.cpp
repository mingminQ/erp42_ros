/**
 * -------------------------------------------------------------------------------------------------
 * 
 * Copyright 2025 Minkyu Kil
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file    gazebo_bridge.cpp
 * @brief   ERP42 platform Gazebo bridge
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gazebo/gazebo_bridge.hpp"
#include "erp42_description/vehicle_parameters.hpp"
#include "erp42_util/log.hpp"

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>

// Chrono literals
using namespace std::chrono_literals;

/**
 * @brief Construct the ERP42 Gazebo bridge node with safe defaults.
 * @details
 * - Initializes rclcpp::Node with the name "erp42_gazebo_bridge".
 * - Declares parameters before wiring ROS interfaces so that publishers/subscribers/services
 *   and initial state can use parameterized limits/offsets.
 */
erp42::gazebo::GazeboBridge::GazeboBridge(const rclcpp::NodeOptions &options)
  : rclcpp::Node("erp42_gazebo_bridge", options)
{
    declare_parameters();
    initialize_node();
}

/**
 * @brief Destructor.
 * @details
 * - Currently trivial. If resources requiring explicit teardown are added (timers,
 *   subscriptions, service clients, async callbacks capturing 'this'), ensure they
 *   are cancelled/reset here to avoid use-after-free during shutdown.
 */
erp42::gazebo::GazeboBridge::~GazeboBridge()
{
}

/**
 * @brief JointState callback to update current speed, steering, and encoder count.
 * @param msg JointState message containing joint names, positions, and velocities.
 * @details
 *  - Resolve indices of required joints by searching names array. (No bounds checks here.)
 *  - Compute vehicle speed from the front-left wheel angular velocity times wheel radius.
 *  - Compute steering as the simple mean of left/right steering hub joint angles (Ackermann approx).
 *  - Compute encoder count from front-left wheel angle changes.
 */
void erp42::gazebo::GazeboBridge::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    auto front_left_wheel_iter = std::find(msg->name.begin(), msg->name.end(), "front_left_wheel_joint" );
    int front_left_wheel_index = std::distance(msg->name.begin(), front_left_wheel_iter);

    auto front_left_steering_hub_iter = std::find(msg->name.begin(), msg->name.end(), "front_left_steering_hub_joint");
    int front_left_steering_hub_index = std::distance(msg->name.begin(), front_left_steering_hub_iter);

    auto front_right_steering_hub_iter = std::find(msg->name.begin(), msg->name.end(), "front_right_steering_hub_joint");
    int front_right_steering_hub_index = std::distance(msg->name.begin(), front_right_steering_hub_iter);

    // Current speed from left front wheel encoder
    current_speed_ = msg->velocity[front_left_wheel_index] * WHEEL_RADIUS;

    // Current steering angle from front left/right wheel steering joints
    double left_wheel_angle  = msg->position[front_left_steering_hub_index];
    double right_wheel_angle = msg->position[front_right_steering_hub_index];
    current_steering_ = (left_wheel_angle + right_wheel_angle) / 2.0 - steering_offset_rad_;

    // Current encoder count from left front wheel encoder
    double current_wheel_angle = msg->position[front_left_wheel_index];
    double delta_wheel_angle = std::remainder(current_wheel_angle - prev_front_left_wheel_angle_, 2.0 * M_PI);

    if((2.0 * M_PI / ENCODER_CPR) < std::abs(delta_wheel_angle))
    {
        encoder_count_ += static_cast<int>(delta_wheel_angle / (2.0 * M_PI) * ENCODER_CPR);
        prev_front_left_wheel_angle_ = current_wheel_angle;
    }
}

/**
 * @brief  Handle incoming control command and translate it to speed/steering/brake commands.
 * @param  msg Shared pointer to erp42_msgs::msg::ControlCommand.
 * @details
 * - Clamps speed/steering into valid ranges, applies steering offset.
 * - Applies E-Stop/Manual overrides (full brake, zero speed/steer).
 * - Applies gear logic (reverse/neutral handling).
 * - Publishes the brake effect to Gazebo by adjusting ODE damping via SetJointProperties service.
 */
void erp42::gazebo::GazeboBridge::control_command_callback(const erp42_msgs::msg::ControlCommand::SharedPtr msg)
{
    speed_command_    = std::clamp(msg->speed, -max_speed_mps_, max_speed_mps_);
    steering_command_ = std::clamp(msg->steering + steering_offset_rad_, -max_steering_rad_, max_steering_rad_);
    brake_command_    = msg->brake;

    // Emergency stop or Manual mode
    if(emergency_stop_ || manual_mode_)
    {
        speed_command_ = 0.0;
        steering_command_ = 0.0;
        brake_command_ = 150;
    }

    // Gear
    if(gear_ == erp42_msgs::srv::ModeCommand::Request::GEAR_REVERSE)
    {
        speed_command_ = -speed_command_;
        steering_command_ = -steering_command_;
    } 
    else if(gear_ == erp42_msgs::srv::ModeCommand::Request::GEAR_NEUTRAL) 
    {
        speed_command_ = 0.0;
    }

    // Update current brake value for feedback
    current_brake_ = brake_command_;

    // Brake
    if (!joint_properties_client_->wait_for_service(200ms))
    {
        return;
    }

    // Brake command threshold
    if(brake_command_ < 25)
    {
        brake_command_ = 0;
    } 

    // Set brake by setting damping coefficient of wheel joints
    auto request = std::make_shared<gazebo_msgs::srv::SetJointProperties::Request>();
    request->ode_joint_config.damping.push_back(
        100.0 * static_cast<double>(brake_command_) / 150.0
    );
    for(const auto& joint_name : wheel_joints_)
    {
        request->joint_name = joint_name;
        joint_properties_client_->async_send_request(request);
    }
}

/**
 * @brief  Service callback to update driving mode flags and gear state.
 * @param  request Shared pointer to ModeCommand request.
 * - request->manual_mode    : true if manual (operator) mode is active.
 * - request->emergency_stop : true to request immediate safe stop.
 * - request->gear           : enum (e.g., GEAR_DRIVE / GEAR_REVERSE / GEAR_NEUTRAL).
 * @param  response Shared pointer to ModeCommand response. Sets response->success to true on accept.
 * @details
 * - This callback only *stores* the requested mode/gear into internal state. The actual effects
 *   (e.g., zeroing speed, applying full brake) should be enforced by the control loop that reads
 *   these members (e.g., in on_update() / command processing).
 * - Emergency stop (E-Stop) typically overrides other commands. Ensure the control path treats
 *   `emergency_stop_ == true` as highest priority (e.g., speed=0, steer=0, max brake).
 * - Manual mode can be used to ignore autonomous commands while allowing operator inputs.
 * - Keep this callback non-blocking. It runs in an rclcpp executor thread.
 */
void erp42::gazebo::GazeboBridge::mode_command_callback(
    const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
    erp42_msgs::srv::ModeCommand::Response::SharedPtr response
)
{
    manual_mode_      = request->manual_mode;
    emergency_stop_   = request->emergency_stop;
    gear_             = request->gear;
    response->success = true;
}

/**
 * @brief Periodic timer callback to publish command and feedback topics.
 * @details 
 * - Publishes the current control command as geometry_msgs/Twist:
 *   - linear.x : forward speed command [m/s]
 *   - angular.z: steering command [rad]
 * - Publishes ERP42 feedback snapshot (mode flags, current states, encoder, heartbeat).
 */
void erp42::gazebo::GazeboBridge::timer_callback()
{
    // Publish control command
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = speed_command_;
    cmd_vel_msg.angular.z = steering_command_;
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Publish feedback
    erp42_msgs::msg::Feedback feedback_msg;
    feedback_msg.header.stamp = this->now();
    feedback_msg.header.frame_id = "";

    feedback_msg.manual_mode = manual_mode_;
    feedback_msg.emergency_stop = emergency_stop_;
    feedback_msg.gear = gear_;

    feedback_msg.speed = current_speed_;
    feedback_msg.steering = current_steering_;
    feedback_msg.brake = current_brake_;

    feedback_msg.encoder_count = encoder_count_;
    feedback_msg.heartbeat = heartbeat_++;

    feedback_pub_->publish(feedback_msg);

    // Publish odometry
    odom_msg_.twist.twist.linear.x = current_speed_;
    odom_msg_.twist.twist.angular.z = current_speed_ / WHEELBASE_LENGTH * std::tan(current_steering_);

    odom_msg_.header.stamp = this->now();
    odom_pub_->publish(odom_msg_);
}

/**
 * @brief Initialize ROS 2 interfaces (timer, pubs/subs, services, clients) for the Gazebo bridge.
 * @details
 * - Creates a periodic wall timer to publish command/feedback at a fixed rate.
 * - Sets up publishers:
 *     /erp42/cmd_vel   : geometry_msgs/Twist command for the simulator/controller
 *     /erp42/feedback  : erp42_msgs/Feedback telemetry mirror
 * - Sets up subscribers:
 *     /erp42/control_command : high-level control input (speed/steer/brake)
 *     /erp42/joint_states    : joint state feedback from Gazebo (for speed/encoder/steer)
 * - Sets up services/clients:
 *     /erp42/mode_command        : mode/gear/E-Stop updates
 *     /erp42/set_joint_properties: (client) to tune joint damping (used for brake emulation)
 */
void erp42::gazebo::GazeboBridge::initialize_node()
{
    // Timer
    timer_ = this->create_wall_timer(20ms, std::bind(&GazeboBridge::timer_callback, this));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/erp42/cmd_vel", 10);

    feedback_pub_ = this->create_publisher<erp42_msgs::msg::Feedback>(
        "/erp42/feedback",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/erp42/odometry_wheel", 10);

    // Subscribers
    control_command_sub_ = this->create_subscription<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command", 
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&GazeboBridge::control_command_callback, this, std::placeholders::_1)
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/erp42/joint_states", 
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&GazeboBridge::joint_state_callback, this, std::placeholders::_1)
    );

    // Services
    mode_command_srv_ = this->create_service<erp42_msgs::srv::ModeCommand>(
        "/erp42/mode_command",
        std::bind(&GazeboBridge::mode_command_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    joint_properties_client_ = this->create_client<gazebo_msgs::srv::SetJointProperties>(
        "/erp42/set_joint_properties"
    );

    // Initialize odometry data
    odom_msg_.header.frame_id = odometry_frame_id_;
    odom_msg_.child_frame_id  = odometry_child_frame_id_;
}

/**
 * @brief Declare and read runtime parameters for ERP42 limits and steering offset.
 * @details
 * - Declares three parameters with default values:
 *   - max_speed_mps        (double, m/s)  : maximum absolute speed command, default 7.0
 *   - max_steering_deg     (double, deg)  : steering limit in degrees, default 25.0
 *   - steering_offset_deg  (double, deg)  : static steering bias in degrees, default 0.0
 * - Converts degree-based parameters to radians for internal use.
 */
void erp42::gazebo::GazeboBridge::declare_parameters()
{
    // Odometry frame id
    this->declare_parameter<std::string>("odometry_frame_id", "odom");
    odometry_frame_id_ = this->get_parameter("odometry_frame_id").as_string();

    this->declare_parameter<std::string>("odometry_child_frame_id", "base_footprint");
    odometry_child_frame_id_ = this->get_parameter("odometry_child_frame_id").as_string();

    // ERP42 parameters
    this->declare_parameter("max_speed_mps", 7.0);
    max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();

    this->declare_parameter("max_steering_deg", 25.0);
    max_steering_deg_ = this->get_parameter("max_steering_deg").as_double();
    max_steering_rad_ = max_steering_deg_ * M_PI / 180.0;

    this->declare_parameter("steering_offset_deg", 0.0);
    steering_offset_deg_ = this->get_parameter("steering_offset_deg").as_double();
    steering_offset_rad_ = steering_offset_deg_ * M_PI / 180.0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(erp42::gazebo::GazeboBridge)