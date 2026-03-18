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
 * @file    feedback_monitor_plugin.cpp
 * @brief   ERP42 platform RQT feedback monitor GUI plugin
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_rqt_plugin/feedback_monitor_plugin.hpp"
#include "erp42_util/log.hpp"
#include "ui_feedback_monitor.h"

#include <QSignalBlocker>
#include <QMessageBox>

#include <functional>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Constructor for the FeedbackMonitorPlugin class.
 * @details Initializes member variables and sets the plugin's object name.
 */
erp42_rqt_plugin::FeedbackMonitorPlugin::FeedbackMonitorPlugin()
  : rqt_gui_cpp::Plugin(), feedback_monitor_widget_(nullptr), spin_running_(false)
{
    setObjectName("FeedbackMonitorPlugin");
}

/**
 * @brief Destructor for the FeedbackMonitorPlugin class.
 * @details Ensures that the plugin is properly shut down by calling
 * the @c shutdownPlugin method to clean up resources and stop any
 * running threads before the object is destroyed.
 */
erp42_rqt_plugin::FeedbackMonitorPlugin::~FeedbackMonitorPlugin()
{
    shutdownPlugin();
}

/**
 * @brief Initializes the plugin, setting up the ROS 2 node, UI, and subscriptions.
 * @details This method creates a ROS 2 node named "erp42_feedback_monitor",
 * sets up the UI components, subscribes to the "/erp42/feedback" topic,
 * reads vehicle parameters, and starts a multi-threaded executor in a separate thread.
 * @param context The plugin context provided by the rqt framework.
 */
void erp42_rqt_plugin::FeedbackMonitorPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
    // Initialize ROS2 node
    node_ = std::make_shared<rclcpp::Node>("erp42_feedback_monitor");

    // Create UI and binding
    widget_ = new QWidget();
    feedback_monitor_widget_ = new Ui::FeedbackMonitorWidget();
    feedback_monitor_widget_->setupUi(widget_);

    // Register plugin
    context.addWidget(widget_);

    // Subscribers
    feedback_sub_ = node_->create_subscription<erp42_msgs::msg::Feedback>(
        "/erp42/feedback",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&FeedbackMonitorPlugin::feedback_callback, this, std::placeholders::_1)
    );

    // Read vehicle parameters and display
    read_vehicle_parameters();

    // Executor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_running_ = true;

    spin_thread_ = std::thread(
        [this]()
        {
            while(spin_running_ && rclcpp::ok())
            {
                executor_->spin_some(20ms);
            }
        }
    );
}

/**
 * @brief Shuts down the plugin, cleaning up resources.
 * @details This method stops the executor thread, shuts down ROS 2 entities,
 * and cleans up the UI components to ensure a graceful shutdown of the plugin.
 */
void erp42_rqt_plugin::FeedbackMonitorPlugin::shutdownPlugin()
{
    // Thread and executor reset
    spin_running_ = false;
    if(executor_)
    {
        executor_->cancel();
    }
    if(spin_thread_.joinable())
    {
        spin_thread_.join();
    }
    executor_.reset();

    // Shut down ROS2 entities
    feedback_sub_.reset();
    node_.reset();

    // Hide widget
    if (widget_) 
    {
        widget_->hide();
        widget_ = nullptr;
    }

    // Deallocate UI helper
    if (feedback_monitor_widget_) 
    {
        delete feedback_monitor_widget_;
        feedback_monitor_widget_ = nullptr;
    }
}

/**
 * @brief Saves plugin settings.
 * @details This method is currently a no-op as there are no settings to save.
 * It is provided for completeness and future extensibility.
 * @param plugin_settings Plugin-specific settings.
 * @param instance_settings Instance-specific settings.
 */
void erp42_rqt_plugin::FeedbackMonitorPlugin::saveSettings(
    qt_gui_cpp::Settings &plugin_settings, 
    qt_gui_cpp::Settings &instance_settings
) const
{
    Q_UNUSED(plugin_settings);
    Q_UNUSED(instance_settings);
}

/**
 * @brief Restores plugin settings.
 * @details This method is currently a no-op as there are no settings to restore.
 * It is provided for completeness and future extensibility.
 * @param plugin_settings Plugin-specific settings.
 * @param instance_settings Instance-specific settings.
 */
void erp42_rqt_plugin::FeedbackMonitorPlugin::restoreSettings(
    const qt_gui_cpp::Settings &plugin_settings, 
    const qt_gui_cpp::Settings &instance_settings
)
{
    Q_UNUSED(plugin_settings);
    Q_UNUSED(instance_settings);
}

/**
 * @brief Callback for the ERP42 feedback topic.
 * @details Converts the received feedback message into human-readable strings
 * and updates the UI. Since UI updates must occur on the GUI thread,
 * this method uses @c QMetaObject::invokeMethod with @c Qt::QueuedConnection.
 * @param msg Shared pointer to @c erp42_msgs::msg::Feedback .
 */
void erp42_rqt_plugin::FeedbackMonitorPlugin::feedback_callback(
    const erp42_msgs::msg::Feedback::SharedPtr msg
)
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

    QMetaObject::invokeMethod(widget_,
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
 * @brief Reads vehicle parameters from the ROS 2 parameter server and updates the UI.
 * @details This method attempts to read parameters from either the "serial_bridge" or
 * "erp42_gazebo_control" node, depending on which one is available.
 * If neither node is found, it logs a warning and returns. 
 * If parameters are successfully read, it updates the corresponding 
 * text boxes in the UI with the parameter values.
 */
void erp42_rqt_plugin::FeedbackMonitorPlugin::read_vehicle_parameters()
{
    bool serial_bridge_parameter_found  = false;
    bool gazebo_control_parameter_found = false;

    std::shared_ptr<rclcpp::SyncParametersClient> serial_bridge_parameter_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_, "serial_bridge");

    std::shared_ptr<rclcpp::SyncParametersClient> gazebo_control_parameter_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_, "erp42_gazebo_control");

    if(serial_bridge_parameter_client->wait_for_service(250ms))
    {
        serial_bridge_parameter_found = true;
    }
    if(gazebo_control_parameter_client->wait_for_service(250ms))
    {
        gazebo_control_parameter_found = true;
    }

    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client;
    if(serial_bridge_parameter_found)
    {
        parameter_client = serial_bridge_parameter_client;
    }
    else if(gazebo_control_parameter_found)
    {
        parameter_client = gazebo_control_parameter_client;
    }
    else
    {
        ERP42_WARN(
            "Failed to read vehicle parameters. "
            "Plugin expects 'serial_bridge' or 'erp42_gazebo_control' node."
        );
        return;
    }

    feedback_monitor_widget_->serial_port_text_box->setPlainText(
        QString::fromStdString(
            parameter_client->get_parameter<std::string>("port_path")
        )
    );

    feedback_monitor_widget_->baud_rate_text_box->setPlainText(
        QString::number(
            parameter_client->get_parameter<int>("baud_rate")
        )
    );

    feedback_monitor_widget_->max_speed_text_box->setPlainText(
        QString::number(
            parameter_client->get_parameter<double>("max_speed_mps"), 'f', 3
        ) + " m/s"
    );

    feedback_monitor_widget_->max_steering_text_box->setPlainText(
        QString::number(
            parameter_client->get_parameter<double>("max_steering_deg"), 'f', 3
        ) + " deg"
    );

    feedback_monitor_widget_->steering_offset_text_box->setPlainText(
        QString::number(
            parameter_client->get_parameter<double>("steering_offset_deg"), 'f', 3
        ) + " deg"
    );
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erp42_rqt_plugin::FeedbackMonitorPlugin, rqt_gui_cpp::Plugin)