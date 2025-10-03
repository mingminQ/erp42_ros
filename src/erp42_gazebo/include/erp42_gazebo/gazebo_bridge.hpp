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
 * @file    gazebo_bridge.hpp
 * @brief   ERP42 platform Gazebo bridge
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_GAZEBO__GAZEBO_BRIDGE_HPP_
#define ERP42_GAZEBO__GAZEBO_BRIDGE_HPP_

#include "erp42_msgs/msg/control_command.hpp"
#include "erp42_msgs/msg/feedback.hpp"
#include "erp42_msgs/srv/mode_command.hpp"

#include "gazebo_msgs/srv/set_joint_properties.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <string>

namespace erp42
{
    /**
     * @brief ROS2 Gazebo bridge for ERP42 commands and feedback.
     * @details Subscribes /erp42/control_command, /erp42/joint_states; 
     * publishes /erp42/cmd_vel, /erp42/feedback at 20 ms; 
     * serves /erp42/mode_command; optionally uses /erp42/set_joint_properties for braking. 
     * Starts safe (manual_mode, E-Stop, NEUTRAL) and derives speed/steer/encoder from joint states.
     */
    class GazeboBridge : public rclcpp::Node
    {
    // "GazeboBridge" member functions
    public:

        /**
         * @brief Construct the ERP42 Gazebo bridge node with safe defaults.
         * @details
         * - Initializes rclcpp::Node with the name "erp42_gazebo_bridge".
         * - Declares parameters before wiring ROS interfaces so that publishers/subscribers/services
         *   and initial state can use parameterized limits/offsets.
         */
        GazeboBridge();

        /**
         * @brief Destructor.
         * @details
         * - Currently trivial. If resources requiring explicit teardown are added (timers,
         *   subscriptions, service clients, async callbacks capturing 'this'), ensure they
         *   are cancelled/reset here to avoid use-after-free during shutdown.
         */
        ~GazeboBridge();

    private:

        /**
         * @brief JointState callback to update current speed, steering, and encoder count.
         * @param msg JointState message containing joint names, positions, and velocities.
         * @details
         *  - Resolve indices of required joints by searching names array. (No bounds checks here.)
         *  - Compute vehicle speed from the front-left wheel angular velocity times wheel radius.
         *  - Compute steering as the simple mean of left/right steering hub joint angles (Ackermann approx).
         *  - Compute encoder count from front-left wheel angle changes.
         */
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        /**
         * @brief  Handle incoming control command and translate it to speed/steering/brake commands.
         * @param  msg Shared pointer to erp42_msgs::msg::ControlCommand.
         * @details
         * - Clamps speed/steering into valid ranges, applies steering offset.
         * - Applies E-Stop/Manual overrides (full brake, zero speed/steer).
         * - Applies gear logic (reverse/neutral handling).
         * - Publishes the brake effect to Gazebo by adjusting ODE damping via SetJointProperties service.
         */
        void control_command_callback(const erp42_msgs::msg::ControlCommand::SharedPtr msg);

        /**
         * @brief  Service callback to update driving mode flags and gear state.
         * @param  request Shared pointer to ModeCommand request.
         * - request->manual_mode    : true if manual (operator) mode is active.
         * - request->emergency_stop : true to request immediate safe stop.
         * - request->gear           : enum (e.g., GEAR_DRIVE / GEAR_REVERSE / GEAR_NEUTRAL).
         * @param  response Shared pointer to ModeCommand response. Sets response->result to true on accept.
         * @details
         * - This callback only *stores* the requested mode/gear into internal state. The actual effects
         *   (e.g., zeroing speed, applying full brake) should be enforced by the control loop that reads
         *   these members (e.g., in on_update() / command processing).
         * - Emergency stop (E-Stop) typically overrides other commands. Ensure the control path treats
         *   `emergency_stop_ == true` as highest priority (e.g., speed=0, steer=0, max brake).
         * - Manual mode can be used to ignore autonomous commands while allowing operator inputs.
         * - Keep this callback non-blocking. It runs in an rclcpp executor thread.
         */
        void mode_command_callback(
            const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
            erp42_msgs::srv::ModeCommand::Response::SharedPtr response
        );

        /**
         * @brief Periodic timer callback to publish command and feedback topics.
         * @details 
         * - Publishes the current control command as geometry_msgs/Twist:
         *   - linear.x : forward speed command [m/s]
         *   - angular.z: steering command [rad]
         * - Publishes ERP42 feedback snapshot (mode flags, current states, encoder, heartbeat).
         */
        void timer_callback();

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
        void initialize_node();

        /**
         * @brief Declare and read runtime parameters for ERP42 limits and steering offset.
         * @details
         * - Declares three parameters with default values:
         *   - max_speed_mps        (double, m/s)  : maximum absolute speed command, default 7.0
         *   - max_steering_deg     (double, deg)  : steering limit in degrees, default 25.0
         *   - steering_offset_deg  (double, deg)  : static steering bias in degrees, default 0.0
         * - Converts degree-based parameters to radians for internal use.
         */
        void declare_parameters();

    // "GazeboBridge" member variables
    private:

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<erp42_msgs::msg::Feedback>::SharedPtr feedback_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        // Subscribers
        rclcpp::Subscription<erp42_msgs::msg::ControlCommand>::SharedPtr control_command_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        // Services
        rclcpp::Service<erp42_msgs::srv::ModeCommand>::SharedPtr mode_command_srv_;
        rclcpp::Client<gazebo_msgs::srv::SetJointProperties>::SharedPtr joint_properties_client_;

        // Odometry frame id
        std::string odometry_frame_id_;
        std::string odometry_child_frame_id_;

        // Odometry data
        nav_msgs::msg::Odometry odom_msg_;

        // ERP42 parameters
        double max_speed_mps_;
        double max_steering_deg_;
        double max_steering_rad_;
        double steering_offset_deg_;
        double steering_offset_rad_;

        // Joints
        const std::vector<std::string> wheel_joints_{
            "front_left_wheel_joint", "front_right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint"
        };

        // Mode command data
        bool manual_mode_                   {true};
        bool emergency_stop_                {true};
        uint8_t gear_                          {1};

        // Control command data
        double  speed_command_               {0.0};
        double  steering_command_            {0.0};
        uint8_t brake_command_                 {0};

        // Feedback data
        double  prev_front_left_wheel_angle_ {0.0};
        double  current_speed_               {0.0};
        double  current_steering_            {0.0};
        uint8_t current_brake_                 {0};
        int     prev_encoder_count_            {0};
        int     encoder_count_                 {0};
        uint8_t heartbeat_                     {0};

    }; // class GazeboBridge

} // namespace erp42

#endif // ERP42_GAZEBO__GAZEBO_BRIDGE_HPP_