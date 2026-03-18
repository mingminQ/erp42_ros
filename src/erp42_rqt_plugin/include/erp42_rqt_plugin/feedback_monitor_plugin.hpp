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
 * @file    feedback_monitor_plugin.hpp
 * @brief   ERP42 platform RQT feedback monitor GUI plugin
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_RQT_PLUGIN__FEEDBACK_MONITOR_PLUGIN_HPP_
#define ERP42_RQT_PLUGIN__FEEDBACK_MONITOR_PLUGIN_HPP_

#include "erp42_msgs/msg/feedback.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

#include <thread>

namespace Ui
{
    // Forwarding feedback monitor widget class
    class FeedbackMonitorWidget;

} // namespace Ui

namespace erp42_rqt_plugin
{
    class FeedbackMonitorPlugin : public rqt_gui_cpp::Plugin
    {
    // Qt5 Object macro
    Q_OBJECT

    // "FeedbackMonitorPlugin" member functions
    public:

        /**
         * @brief Constructor for the FeedbackMonitorPlugin class.
         * @details Initializes member variables and sets the plugin's object name.
         */
        FeedbackMonitorPlugin();

        /**
         * @brief Destructor for the FeedbackMonitorPlugin class.
         * @details Ensures that the plugin is properly shut down by calling
         * the @c shutdownPlugin method to clean up resources and stop any
         * running threads before the object is destroyed.
         */
        ~FeedbackMonitorPlugin() override;

        /**
         * @brief Initializes the plugin, setting up the ROS 2 node, UI, and subscriptions.
         * @details This method creates a ROS 2 node named "erp42_feedback_monitor",
         * sets up the UI components, subscribes to the "/erp42/feedback" topic,
         * reads vehicle parameters, and starts a multi-threaded executor in a separate thread.
         * @param context The plugin context provided by the rqt framework.
         */
        void initPlugin(qt_gui_cpp::PluginContext &context) override;

        /**
         * @brief Shuts down the plugin, cleaning up resources.
         * @details This method stops the executor thread, shuts down ROS 2 entities,
         * and cleans up the UI components to ensure a graceful shutdown of the plugin.
         */
        void shutdownPlugin() override;

        /**
         * @brief Saves plugin settings.
         * @details This method is currently a no-op as there are no settings to save.
         * It is provided for completeness and future extensibility.
         * @param plugin_settings Plugin-specific settings.
         * @param instance_settings Instance-specific settings.
         */
        void saveSettings(
            qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings
        ) const override;

        /**
         * @brief Restores plugin settings.
         * @details This method is currently a no-op as there are no settings to restore.
         * It is provided for completeness and future extensibility.
         * @param plugin_settings Plugin-specific settings.
         * @param instance_settings Instance-specific settings.
         */
        void restoreSettings(
            const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings
        ) override;

    private:

        /**
         * @brief Callback for the ERP42 feedback topic.
         * @details Converts the received feedback message into human-readable strings
         * and updates the UI. Since UI updates must occur on the GUI thread,
         * this method uses @c QMetaObject::invokeMethod with @c Qt::QueuedConnection.
         * @param msg Shared pointer to @c erp42_msgs::msg::Feedback .
         */
        void feedback_callback(const erp42_msgs::msg::Feedback::SharedPtr msg);

        /**
         * @brief Reads vehicle parameters from the ROS 2 parameter server and updates the UI.
         * @details This method attempts to read parameters from either the "serial_bridge" or
         * "erp42_gazebo_control" node, depending on which one is available.
         * If neither node is found, it logs a warning and returns. 
         * If parameters are successfully read, it updates the corresponding 
         * text boxes in the UI with the parameter values.
         */
        void read_vehicle_parameters();

    // "FeedbackMonitorPlugin" member variables
    private:

        // QT widget
        QWidget *widget_;
        Ui::FeedbackMonitorWidget *feedback_monitor_widget_;

        // ROS2 node
        rclcpp::Node::SharedPtr node_;

        // Subscribers
        rclcpp::Subscription<erp42_msgs::msg::Feedback>::SharedPtr feedback_sub_;

        // Executors
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
        std::thread spin_thread_;
        bool spin_running_;

    }; // class FeedbackMonitorPlugin

} // namespace erp42_rqt_plugin

#endif // ERP42_RQT_PLUGIN__FEEDBACK_MONITOR_PLUGIN_HPP_