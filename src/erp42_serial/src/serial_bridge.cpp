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
 * @file    serial_bridge.cpp
 * @brief   ERP42 platform serial bridge
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_serial/serial_bridge.hpp"
#include "erp42_serial/serial_packet.hpp"

#include "erp42_util/exception.hpp"
#include "erp42_util/log.hpp"

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>

// Chrono literals
using namespace std::chrono_literals;

// Place holder for binding
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Default class contructor
 * @details Initializes the base Node with name "erp42_serial", 
 * resets the heartbeat counter to zero.
 */
erp42::SerialBridge::SerialBridge()
  : Node("erp42_serial_bridge"),
    heartbeat_(0)
{
    declare_parameters();
    initialize_node();
}

/**
 * @brief Default class destructor
 * @details Destroys the SerialBridge node, closing and deallocating the serial port.
 */
erp42::SerialBridge::~SerialBridge()
{
    // Close and deallocate serial port
    if(!serial_port_)
    {
        ERP42_ERROR("SerialBridge::~SerialBridge() Serial port %s is deallocated already.", 
            port_path_.c_str());
    }
    else
    {
        serial_port_->close_port();
        serial_port_.reset();
    }
}

/**
 * @brief Receives and publishes feedback from the serial port.
 * @details Validates packet structure (STX/ETX), parses raw bytes into a Feedback message, 
 * and publishes it to the feedback topic.
 * @return 'true' if the packet was received, parsed, and published successfully; 'false' otherwise.
 */
bool erp42::SerialBridge::receive_feedback()
{
    // Failed to receive packet
    if(!serial_port_->receive_packet(rx_packet_.data(), RX::PACKET_SIZE))
    {
        return false;
    }

    // Check packet protocols
    if(rx_packet_[RX::STX_S] != 0x53 || // S
       rx_packet_[RX::STX_T] != 0x54 || // T
       rx_packet_[RX::STX_X] != 0x58 || // X
       rx_packet_[RX::ETX_0] != 0x0D || // ETX_0
       rx_packet_[RX::ETX_1] != 0x0A )  // ETX_1
    {
        ERP42_ERROR("SerialBridge::receive_feedback() Packet protocol is ruined.");
        return false;
    }

    // Byte to ROS message
    erp42_msgs::msg::Feedback feedback_msg;
    feedback_msg.header.stamp = this->now();

    // Control mode
    feedback_msg.manual_mode = !(rx_packet_[RX::CONTROL_MODE]);

    // Emergency stop
    feedback_msg.emergency_stop = rx_packet_[RX::EMERGENCY_STOP];

    // Gear
    feedback_msg.gear = rx_packet_[RX::GEAR];

    // Speed (m/s)
    int speed_raw = 0;
    speed_raw |= static_cast<int>((rx_packet_[RX::SPEED_RAW_0])      & 0xff  );
    speed_raw |= static_cast<int>((rx_packet_[RX::SPEED_RAW_1] << 8) & 0xff00);
    speed_raw  = 30000 < speed_raw ? speed_raw - 65536 : speed_raw;
    feedback_msg.speed = static_cast<double>(speed_raw) * BYTE2MPS;

    // Steering (rad)
    int steering_raw = 0;
    steering_raw |= static_cast<int>((rx_packet_[RX::STEERING_100_0])      & 0xff  );
    steering_raw |= static_cast<int>((rx_packet_[RX::STEERING_100_1] << 8) & 0xff00);
    steering_raw = 30000 < steering_raw ? steering_raw - 65536 : steering_raw;
    feedback_msg.steering = static_cast<double>(steering_raw) * BYTE2RAD - steering_offset_rad_;

    // Brake (1-150)
    feedback_msg.brake = rx_packet_[RX::BRAKE];

    // Encoder (rad)
    int pulse_count = 0;
    pulse_count |= static_cast<int>((rx_packet_[RX::ENCODER_0])       & 0xff      );
    pulse_count |= static_cast<int>((rx_packet_[RX::ENCODER_1] << 8)  & 0xff00    );
    pulse_count |= static_cast<int>((rx_packet_[RX::ENCODER_2] << 16) & 0xff0000  );
    pulse_count |= static_cast<int>((rx_packet_[RX::ENCODER_3] << 24) & 0xff000000);
    feedback_msg.encoder_count = -1.0 * static_cast<double>(pulse_count);

    // Heartbeat
    feedback_msg.heartbeat = rx_packet_[RX::HEARTBEAT];

    // Publish feedback message
    feedback_pub_->publish(feedback_msg);
    
    return true;
}

/**
 * @brief Prepares and transmits the command packet over the serial port.
 * @details
 *   - Sets the start-of-text bytes (STX_S, STX_T, STX_X) and end-of-text bytes (ETX_0, ETX_1).  
 *   - Updates the heartbeat field in the packet.  
 *   - Sends the packet via 'serial_port_->transmit_packet()'.  
 * @return 'true' if the packet was transmitted successfully; 'false' otherwise.
 */
bool erp42::SerialBridge::transmit_command()
{
    // Set packet protocols
    tx_packet_[TX::STX_S] = 0x53;
    tx_packet_[TX::STX_T] = 0x54;
    tx_packet_[TX::STX_X] = 0x58;
    tx_packet_[TX::ETX_0] = 0x0D;
    tx_packet_[TX::ETX_1] = 0x0A;

    // Update heartbeat
    tx_packet_[TX::HEARTBEAT] = heartbeat_;

    // Packet transmission
    return serial_port_->transmit_packet(tx_packet_.data(), TX::PACKET_SIZE);
}

/**
 * @brief Periodic timer callback for serial communication.
 * @details Calls 'receive_feedback()', 'transmit_command()', and increments the heartbeat counter.
 */
void erp42::SerialBridge::timer_callback()
{
    transmit_command();
    receive_feedback();
    heartbeat_++;
}

/**
 * @brief Processes incoming mode commands and updates the transmit packet.
 * @param request ModeCommand service containing manual mode flag, emergency stop flag, and gear selection.
 * @param response Returns whether the service processing was successful.
 * @details
 *   - Sets CONTROL_MODE to 1 for autonomous (when 'manual_mode' is false), 0 otherwise.  
 *   - Sets EMERGENCY_STOP flag based on 'msg.emergency_stop'.  
 *   - Sets GEAR field to 'msg.gear'.
 */
void erp42::SerialBridge::mode_command_callback(
    const std::shared_ptr<erp42_msgs::srv::ModeCommand::Request> request,
    std::shared_ptr<erp42_msgs::srv::ModeCommand::Response> response)
{
    // Control mode
    tx_packet_[TX::CONTROL_MODE] = !(request->manual_mode);

    // Emergency stop
    tx_packet_[TX::EMERGENCY_STOP] = request->emergency_stop;

    // Gear
    tx_packet_[TX::GEAR] = request->gear;

    // Response flag
    response->result = true;
}

/**
 * @brief Processes incoming control commands and updates the transmit packet.
 * @param msg ControlCommand message containing desired speed (m/s), steering (rad), and brake (0–100).
 * @details
 *   - Clamps speed to [0, max_speed_mps_] and converts to raw units.  
 *   - Adjusts steering by offset, clamps to ±max_steering_rad_, and converts to raw units.  
 *   - Inserts brake value directly into the packet.
 */
void erp42::SerialBridge::control_command_callback(const erp42_msgs::msg::ControlCommand::SharedPtr msg)
{
    // Speed (m/s to motor raw command)
    double speed = msg->speed < 0 ? 0 : msg->speed;
    speed = max_speed_mps_ < speed ? max_speed_mps_ : speed;

    uint16_t speed_command = static_cast<uint16_t>(speed * MPS2BYTE);
    tx_packet_[TX::SPEED_RAW_1] =  speed_command & 0xff;
    tx_packet_[TX::SPEED_RAW_0] = (speed_command & 0xff00) >> 8;

    // Steering (rad to raw command)
    double steering = msg->steering + steering_offset_rad_; 
    steering = std::clamp(steering, -max_steering_rad_, max_steering_rad_);

    uint16_t steering_command = static_cast<uint16_t>(steering * RAD2BYTE);
    tx_packet_[TX::STEERING_100_1] =  steering_command & 0xff;
    tx_packet_[TX::STEERING_100_0] = (steering_command & 0xff00) >> 8;

    // Brake (0 - 150)
    uint8_t brake = std::clamp<uint8_t>(msg->brake, 0, 150);
    tx_packet_[TX::BRAKE] = brake;
}

/** @brief Initializes timers, publishers, subscriptions, and the serial port. */
void erp42::SerialBridge::initialize_node()
{
    // Timer
    timer_ = this->create_wall_timer(20ms, std::bind(&SerialBridge::timer_callback, this));

    // Publishers
    feedback_pub_ = this->create_publisher<erp42_msgs::msg::Feedback>(
        "/erp42/feedback",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
    );

    // Subscribers
    control_command_sub_ = this->create_subscription<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&SerialBridge::control_command_callback, this, _1)
    );

    // Services
    mode_command_srv_ = this->create_service<erp42_msgs::srv::ModeCommand>(
        "/erp42/mode_command",
        std::bind(&SerialBridge::mode_command_callback, this, _1, _2)
    );

    // Serial port
    serial_port_ = std::make_unique<SerialPort>(port_path_, baud_rate_);
    if(!serial_port_)
    {
        throw Exception("SerialBridge::initialize_node() Serial port allocation failed.");
    }
    else
    {
        serial_port_->open_port();
    }
}

/** @brief Declares and retrieves ROS2 parameters for serial and ERP42 configuration. */
void erp42::SerialBridge::declare_parameters()
{
    // Serial port
    this->declare_parameter<std::string>("port_path", "/dev/ttyUSB0");
    port_path_ = this->get_parameter("port_path").as_string();

    this->declare_parameter<int>("baud_rate", 115200);
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // ERP42  parameters
    this->declare_parameter<double>("max_speed_mps", 7.0);
    max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();

    this->declare_parameter<double>("max_steering_deg", 25.0);
    max_steering_rad_ = this->get_parameter("max_steering_deg").as_double() * M_PI / 180.0;

    this->declare_parameter<double>("steering_offset_deg", 0.0);
    steering_offset_rad_ = this->get_parameter("steering_offset_deg").as_double() * M_PI / 180.0;
}
