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
 * @file    control_panel_plugin.hpp
 * @brief   ERP42 platform RQT control panel GUI plugin
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_RQT_PLUGIN__CONTROL_PANEL_PLUGIN_HPP_
#define ERP42_RQT_PLUGIN__CONTROL_PANEL_PLUGIN_HPP_

#include "erp42_msgs/msg/control_command.hpp"
#include "erp42_msgs/srv/mode_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rqt_gui_cpp/plugin.h>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QSlider>
#include <QWidget>

#include <thread>

namespace Ui
{
    // Forwarding control panel widget class
    class ControlPanelWidget;

} // namespace Ui

namespace erp42_rqt_plugin
{
    /**
     * @brief ERP42 platform RQT control panel GUI plugin
     * @details
     *  - Publishes ControlCommand messages at 50 Hz when activated.
     *  - Sends ModeCommand requests via service when "Apply Mode" button is clicked.
     *  - UI elements include speed, steering, brake controls, mode selection, E-Stop, and gear selection.
     */
    class ControlPanelPlugin : public rqt_gui_cpp::Plugin
    {
    // Qt5 Object macro
    Q_OBJECT

    // "ControlPanelPlugin" member functions
    public:

        /**
         * @brief Constructor for the ControlPanelPlugin class.
         * @details Initializes member variables and sets the plugin's object name.
         */
        ControlPanelPlugin();

        /**
         * @brief Destructor for the ControlPanelPlugin class.
         * @details Ensures that the plugin is properly shut down by calling
         * the @c shutdownPlugin method to clean up resources and stop any
         * running threads before the object is destroyed.
         */
        ~ControlPanelPlugin() override;

        /**
         * @brief Initializes the plugin, setting up the ROS 2 node, UI, and subscriptions.
         * @details This method creates a ROS 2 node named "erp42_control_panel",
         * sets up the UI components, binds UI elements to their respective handlers,
         * creates publishers and service clients, and starts a multi-threaded executor
         * in a separate thread to handle ROS 2 communication.
         * @param context The plugin context provided by the rqt framework.
         */
        void initPlugin(qt_gui_cpp::PluginContext &context) override;

        /**
         * @brief Shuts down the plugin, cleaning up resources and stopping threads.
         * @details This method stops the ROS 2 executor thread, resets publishers,
         * service clients, and the ROS 2 node, hides the UI widget, and deallocates
         * the UI helper object to ensure a clean shutdown of the plugin.
         */
        void shutdownPlugin() override;

        /**
         * @brief Save plugin settings
         * @details This function is called by the framework to save the plugin settings.
         * @param plugin_settings Plugin settings provided by the framework
         * @param instance_settings Instance settings provided by the framework
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

    private Q_SLOTS:

        /**
         * @brief Build and send an ERP42 ModeCommand via ROS 2 service.
         * @details
         *  - Verifies the client is initialized and the service is available (wait up to 200 ms).
         *  - Disables the "Apply" button while the request is in flight to prevent duplicate sends.
         *  - Populates manual/auto, E-Stop, and gear fields based on current UI state.
         *  - Shows a warning dialog on common errors (uninitialized client, service unavailable, apply failure).
         */
        void on_apply_mode_command();

    private:

        /**
         * @brief Periodic publisher callback for ControlCommand messages.
         * @details
         *  - Publishes speed [m/s], steering [rad], and brake [0..150] at the timer rate.
         *  - Steering entered in degrees is converted to radians before publishing.
         */
        void bind_slider_spin_box(
            QSlider *slider, QSpinBox *spin_box, int lower_bound, int upper_bound, int step
        );

        /**
         * @brief Periodic publisher callback for ControlCommand messages.
         * @details
         *  - Publishes speed [m/s], steering [rad], and brake [0..150] at the timer rate.
         *  - Steering entered in degrees is converted to radians before publishing.
         */
        void bind_slider_spin_box(
            QSlider *slider, QDoubleSpinBox *spin_box, double lower_bound, double upper_bound, double step
        );

        /**
         * @brief Periodic publisher callback for ControlCommand messages.
         * @details
         *  - Publishes speed [m/s], steering [rad], and brake [0..150] at the timer rate.
         *  - Steering entered in degrees is converted to radians before publishing.
         */
        void control_command_timer_callback();

    // "ControlPanelPlugin" member variables
    private:

        // QT widget
        QWidget *widget_;
        Ui::ControlPanelWidget *control_panel_widget_;

        // ROS2 node
        rclcpp::Node::SharedPtr node_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Publishers
        rclcpp::Publisher<erp42_msgs::msg::ControlCommand>::SharedPtr control_command_pub_;

        // Service clients
        rclcpp::Client<erp42_msgs::srv::ModeCommand>::SharedPtr mode_command_client_;

        // Control command message
        erp42_msgs::msg::ControlCommand control_command_msg_;

        // Executors
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
        std::thread spin_thread_;
        bool spin_running_;
    
    }; // class ControlPanelPlugin

} // namespace erp42_rqt_plugin

#endif // ERP42_RQT_PLUGIN__CONTROL_PANEL_PLUGIN_HPP_