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
 * @file    feedback_monitor.cpp
 * @brief   ERP42 platform Qt5 feedback monitor
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gui/feedback_monitor.hpp"
#include "erp42_util/log.hpp"
#include "ui_feedback_monitor.h"

#include <QSignalBlocker>
#include <QMessageBox>
#include <QTimer>

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>

// Placeholders
using std::placeholders::_1;

/**
 * @brief Constructs the feedback monitor dock widget.
 * @details Inherits from Qt @c QDockWidget and ROS 2 @c Node so that the GUI widget
 * and the ROS node operate as a single object.
 * @param parent Parent widget pointer (default: nullptr).
 */
erp42::FeedbackMonitor::FeedbackMonitor(QWidget *parent)
  : QDockWidget(parent),
    Node("erp42_feedback_monitor")
{
    initialize_node();
}

/**
 * @brief Destructor.
 * @details No explicit cleanup is required. Qt and ROS resources are released
 * by their respective lifecycles.
 */
erp42::FeedbackMonitor::~FeedbackMonitor()
{
}

/**
 * @brief Callback for the ERP42 feedback topic.
 * @details Converts the received feedback message into human-readable strings
 * and updates the UI. Since UI updates must occur on the GUI thread,
 * this method uses @c QMetaObject::invokeMethod with @c Qt::QueuedConnection.
 * @param[in] msg Shared pointer to @c erp42_msgs::msg::Feedback .
 */
void erp42::FeedbackMonitor::feedback_callback(const erp42_msgs::msg::Feedback::SharedPtr msg)
{
    // Control mode
    const bool manual_mode = msg->manual_mode;
    const QString control_mode_text = manual_mode ? "Manual" : "Auto";

    // Emergency stop
    const bool emergency_stop = msg->emergency_stop;
    const QString emergency_stop_text = emergency_stop ? "E-Stop ON" : "E-Stop OFF";

    // Gear
    const uint8_t gear = msg->gear;
    const QString gear_text =
        (gear == erp42_msgs::msg::Feedback::GEAR_DRIVE)   ? "Drive"  :
        (gear == erp42_msgs::msg::Feedback::GEAR_REVERSE) ? "Reverse": "Neutral";
    
    // Speed
    const double speed = msg->speed;
    const QString speed_text = QString::number(speed, 'f', 3) + " m/s";

    // Steering
    const double steering_deg = msg->steering * 180.0 / M_PI;
    const QString steering_text = QString::number(steering_deg, 'f', 3) + " deg";

    // Brake
    const uint8_t brake = msg->brake;
    const QString brake_text = QString::number(brake);

    // Encoder
    const int32_t encoder_count = msg->encoder_count;
    const QString encoder_text = QString::number(encoder_count);

    // Heartbeat
    const uint8_t heartbeat = msg->heartbeat;
    const QString heartbeat_text = QString::number(heartbeat);

    QMetaObject::invokeMethod(this,
        [this, control_mode_text, emergency_stop_text, gear_text, speed_text, 
            steering_text, brake_text, encoder_text, heartbeat_text]()
        {
            feedback_monitor_widget_->control_mode_text_box->setPlainText(control_mode_text);
            feedback_monitor_widget_->estop_text_box->setPlainText(emergency_stop_text);
            feedback_monitor_widget_->gear_text_box->setPlainText(gear_text);

            feedback_monitor_widget_->speed_text_box->setPlainText(speed_text);
            feedback_monitor_widget_->steering_text_box->setPlainText(steering_text);
            feedback_monitor_widget_->brake_text_box->setPlainText(brake_text);

            feedback_monitor_widget_->encoder_text_box->setPlainText(encoder_text);
            feedback_monitor_widget_->heartbeat_text_box->setPlainText(heartbeat_text);
        }, 
        Qt::QueuedConnection
    );
}

/**
 * @brief Selects the target node (either @c erp42_serial_bridge or @c erp42_gazebo_bridge).
 * @details Checks the ROS 2 node graph for the existence of either node.
 * If found, sets @c node_name to the found node and returns true.
 * If neither node is found within @c 5000ms, returns false.
 * @param[out] node_name Name of the selected target node.
 * @return True if a target node is found; otherwise false.
 */
bool erp42::FeedbackMonitor::select_target_node(std::string &node_name)
{
    // Timeout and step durations
    std::chrono::milliseconds timeout(5000);
    std::chrono::milliseconds step(100);

    // Node graph interface and event
    auto graph = this->get_node_graph_interface();
    auto event = graph->get_graph_event();

    auto starting_time = std::chrono::steady_clock::now();
    while(rclcpp::ok() && ((std::chrono::steady_clock::now() - starting_time) < timeout))
    {
        // Get node names
        const auto node_names = graph->get_node_names();

        // Serial bridge node exists
        const auto serial_bridge_iter = std::find(node_names.begin(), node_names.end(), "/erp42_serial_bridge");
        if(serial_bridge_iter != node_names.end())
        {
            node_name = "/erp42_serial_bridge";
            return true;
        }

        // Gazebo bridge node exists
        const auto gazebo_bridge_iter = std::find(node_names.begin(), node_names.end(), "/erp42_gazebo_bridge");
        if(gazebo_bridge_iter != node_names.end())
        {
            node_name = "/erp42_gazebo_bridge";
            return true;
        }

        // Wait for graph change
        graph->wait_for_graph_change(event, step);
    }

    return false;
}

/**
 * @brief Reads parameters from the target node and displays them on the UI.
 * @details Selects the target node (either @c erp42_serial_bridge or @c erp42_gazebo_bridge )
 * and reads its parameters. If the service is not available within @c 1000ms, the method
 * returns without updating.
 */
void erp42::FeedbackMonitor::read_vehicle_parameters()
{
    // Select target node (erp42_serial_bridge or erp42_gazebo_bridge)
    std::string node_name;
    if(!select_target_node(node_name))
    {
        ERP42_ERROR("FeedbackMonitor::read_vehicle_parameters Failed to find target node (erp42_serial_bridge or erp42_gazebo_bridge).");
        return;
    }

    // Parameter client
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
    if(!parameter_client_->wait_for_service(std::chrono::milliseconds(1000)))
    {
        ERP42_ERROR("FeedbackMonitor::read_vehicle_parameters Parameter service not available.");
        return;
    }

    // Read erp42_serial_bridge parameters
    if(node_name == "/erp42_serial_bridge")
    {
        std::string port_path = parameter_client_->get_parameter<std::string>("port_path");
        feedback_monitor_widget_->serial_port_text_box->setPlainText(QString::fromStdString(port_path));

        int baud_rate = parameter_client_->get_parameter<int>("baud_rate");
        feedback_monitor_widget_->baud_rate_text_box->setPlainText(QString::number(baud_rate));

        double max_speed = parameter_client_->get_parameter<double>("max_speed_mps");
        feedback_monitor_widget_->max_speed_text_box->setPlainText(QString::number(max_speed, 'f', 3) + " m/s");

        double max_steering_deg = parameter_client_->get_parameter<double>("max_steering_deg");
        feedback_monitor_widget_->max_steering_text_box->setPlainText(QString::number(max_steering_deg, 'f', 3) + " deg");

        double steering_offset_deg = parameter_client_->get_parameter<double>("steering_offset_deg");
        feedback_monitor_widget_->steering_offset_text_box->setPlainText(QString::number(steering_offset_deg, 'f', 3) + " deg");
    }

    // Read erp42_gazebo_bridge parameters
    else if(node_name == "/erp42_gazebo_bridge")
    {
        double max_speed = parameter_client_->get_parameter<double>("max_speed_mps");
        feedback_monitor_widget_->max_speed_text_box->setPlainText(QString::number(max_speed, 'f', 3) + " m/s");

        double max_steering_deg = parameter_client_->get_parameter<double>("max_steering_deg");
        feedback_monitor_widget_->max_steering_text_box->setPlainText(QString::number(max_steering_deg, 'f', 3) + " deg");

        double steering_offset_deg = parameter_client_->get_parameter<double>("steering_offset_deg");
        feedback_monitor_widget_->steering_offset_text_box->setPlainText(QString::number(steering_offset_deg, 'f', 3) + " deg");
    }
}

/**
 * @brief Initializes the ROS interfaces and the Qt UI.
 * @details
 * - Subscribes to @c /erp42/feedback with QoS: KeepLast(1), reliable, volatile.
 * - Creates a synchronous parameter client for node @c erp42_serial_bridge .
 * - Sets up the Qt UI and fetches initial parameters to display.
 */
void erp42::FeedbackMonitor::initialize_node()
{
    // Subscibers
    feedback_sub_ = this->create_subscription<erp42_msgs::msg::Feedback>(
        "/erp42/feedback",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&FeedbackMonitor::feedback_callback, this, _1)
    );

    // Qt5 feedback monitor
    feedback_monitor_widget_ = std::make_unique<Ui::FeedbackMonitorDockWidget>();
    feedback_monitor_widget_->setupUi(this);

    // Read vehicle parameters and display
    read_vehicle_parameters();
}