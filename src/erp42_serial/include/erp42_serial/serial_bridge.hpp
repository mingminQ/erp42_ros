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
 * @file    serial_bridge.hpp
 * @brief   ERP42 platform serial bridge
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_SERIAL__SERIAL_BRIDGE_HPP_
#define ERP42_SERIAL__SERIAL_BRIDGE_HPP_

#include "erp42_serial/serial_port.hpp"
#include "erp42_serial/serial_packet.hpp"

#include "erp42_msgs/msg/control_command.hpp"
#include "erp42_msgs/srv/mode_command.hpp"
#include "erp42_msgs/msg/feedback.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <array>

namespace erp42
{
    /**
     * @brief ROS2 node for serial communication on the ERP42 platform.
     * @details Manages parameter declaration, publishers/subscriptions, and serial port I/O.
     */
    class SerialBridge : public rclcpp::Node
    {
    // "SerialBridge" memebr functions
    public:

        /**
         * @brief Default class contructor
         * @details Initializes the base Node with name "erp42_serial", 
         * resets the heartbeat counter to zero.
         */
        SerialBridge();

        /**
         * @brief Default class destructor
         * @details Destroys the SerialBridge node, closing and deallocating the serial port.
         */
        ~SerialBridge();

    private:

        /**
         * @brief Receives and publishes feedback from the serial port.
         * @details Validates packet structure (STX/ETX), parses raw bytes into a Feedback message, 
         * and publishes it to the feedback topic.
         * @return 'true' if the packet was received, parsed, and published successfully; 'false' otherwise.
         */
        bool receive_feedback();

        /**
         * @brief Prepares and transmits the command packet over the serial port.
         * @details
         *   - Sets the start-of-text bytes (STX_S, STX_T, STX_X) and end-of-text bytes (ETX_0, ETX_1).  
         *   - Updates the heartbeat field in the packet.  
         *   - Sends the packet via 'serial_port_->transmit_packet()'.  
         * @return 'true' if the packet was transmitted successfully; 'false' otherwise.
         */
        bool transmit_command();

        /**
         * @brief Periodic timer callback for serial communication.
         * @details Calls 'receive_feedback()', 'transmit_command()', and increments the heartbeat counter.
         */
        void timer_callback();

        /**
         * @brief Processes incoming mode commands and updates the transmit packet.
         * @param request ModeCommand service containing manual mode flag, emergency stop flag, and gear selection.
         * @param response Returns whether the service processing was successful.
         * @details
         *   - Sets CONTROL_MODE to 1 for autonomous (when 'manual_mode' is false), 0 otherwise.  
         *   - Sets EMERGENCY_STOP flag based on 'msg.emergency_stop'.  
         *   - Sets GEAR field to 'msg.gear'.
         */
        void mode_command_callback(
            const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
            erp42_msgs::srv::ModeCommand::Response::SharedPtr response
        );

        /**
         * @brief Processes incoming control commands and updates the transmit packet.
         * @param msg ControlCommand message containing desired speed (m/s), steering (rad), and brake (0–100).
         * @details
         *   - Clamps speed to [0, max_speed_mps_] and converts to raw units.  
         *   - Adjusts steering by offset, clamps to ±max_steering_rad_, and converts to raw units.  
         *   - Inserts brake value directly into the packet.
         */
        void control_command_callback(const erp42_msgs::msg::ControlCommand::SharedPtr msg);

        /** @brief Initializes timers, publishers, subscriptions, and the serial port. */
        void initialize_node();

        /** @brief Declares and retrieves ROS2 parameters for serial and ERP42 configuration. */
        void declare_parameters();

    // "SerialBridge" member variables
    private:

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Publishers
        rclcpp::Publisher<erp42_msgs::msg::Feedback>::SharedPtr feedback_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        // Subscribers
        rclcpp::Subscription<erp42_msgs::msg::ControlCommand>::SharedPtr control_command_sub_;

        // Services
        rclcpp::Service<erp42_msgs::srv::ModeCommand>::SharedPtr mode_command_srv_;

        // Odometry data
        nav_msgs::msg::Odometry odom_msg_;

        // Fixed frame id
        std::string odometry_frame_id_;
        std::string odometry_child_frame_id_;

        // Serial port
        std::unique_ptr<SerialPort> serial_port_;
        std::string port_path_;
        int baud_rate_;

        // ERP42 parameters
        double max_speed_mps_;
        double max_steering_rad_;
        double steering_offset_rad_;

        // Serial packets
        std::array<uint8_t, TX::PACKET_SIZE> tx_packet_;
        std::array<uint8_t, RX::PACKET_SIZE> rx_packet_;

        // Health checker counter
        uint8_t heartbeat_;

    }; // class SerialBridge

} // namespace erp42

#endif // ERP42_SERIAL__SERIAL_BRIDGE_HPP_