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
 * @file    control_panel_plugin.cpp
 * @brief   ERP42 platform RQT control panel GUI plugin
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_rqt_plugin/control_panel_plugin.hpp"
#include "ui_control_panel.h"

#include <QSignalBlocker>
#include <QMessageBox>

#include <functional>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Constructor for the ControlPanelPlugin class.
 * @details Initializes member variables and sets the plugin's object name.
 */
erp42::rqt_plugin::ControlPanelPlugin::ControlPanelPlugin()
  : rqt_gui_cpp::Plugin(), control_panel_widget_(nullptr), spin_running_(false)
{
    setObjectName("ControlPanelPlugin");
}

/**
 * @brief Destructor for the ControlPanelPlugin class.
 * @details Ensures that the plugin is properly shut down by calling
 * the @c shutdownPlugin method to clean up resources and stop any
 * running threads before the object is destroyed.
 */
erp42::rqt_plugin::ControlPanelPlugin::~ControlPanelPlugin()
{
    shutdownPlugin();
}

/**
 * @brief Initializes the plugin, setting up the ROS 2 node, UI, and subscriptions.
 * @details This method creates a ROS 2 node named "erp42_control_panel",
 * sets up the UI components, binds UI elements to their respective handlers,
 * creates publishers and service clients, and starts a multi-threaded executor
 * in a separate thread to handle ROS 2 communication.
 * @param context The plugin context provided by the rqt framework.
 */
void erp42::rqt_plugin::ControlPanelPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
    // Initialize ROS2 node
    node_ = std::make_shared<rclcpp::Node>("erp42_control_panel");

    // Create UI and binding
    widget_ = new QWidget();
    control_panel_widget_ = new Ui::ControlPanelWidget();
    control_panel_widget_->setupUi(widget_);

    // Register plugin
    context.addWidget(widget_);

    QObject::connect(
        control_panel_widget_->apply_mode_button,
        &QPushButton::clicked,
        this,
        &ControlPanelPlugin::on_apply_mode_command
    );

    bind_slider_spin_box(
        control_panel_widget_->speed_slider, control_panel_widget_->speed_spin_box,
        0.0, 7.0, 0.1
    );

    bind_slider_spin_box(
        control_panel_widget_->steering_slider, control_panel_widget_->steering_spin_box,
         -28.0, 28.0, 1.0
    );

    bind_slider_spin_box(
        control_panel_widget_->brake_slider, control_panel_widget_->brake_spin_box, 
        0, 150, 10
    );

    control_panel_widget_->steering_slider->setValue(28);

    // Timer
    timer_ = node_->create_wall_timer(
        20ms, std::bind(&ControlPanelPlugin::control_command_timer_callback, this)
    );

    // Publishers
    control_command_pub_ = node_->create_publisher<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command",
        rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable()
    );

    // Service clients
    mode_command_client_ = node_->create_client<erp42_msgs::srv::ModeCommand>(
        "/erp42/mode_command"
    );

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
 * @brief Shuts down the plugin, cleaning up resources and stopping threads.
 * @details This method stops the ROS 2 executor thread, resets publishers,
 * service clients, and the ROS 2 node, hides the UI widget, and deallocates
 * the UI helper object to ensure a clean shutdown of the plugin.
 */
void erp42::rqt_plugin::ControlPanelPlugin::shutdownPlugin()
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
    timer_.reset();
    control_command_pub_.reset();
    mode_command_client_.reset();
    node_.reset();

    // Hide widget
    if (widget_) 
    {
        widget_->hide();
        widget_ = nullptr;
    }

    // Deallocate UI helper
    if (control_panel_widget_) 
    {
        delete control_panel_widget_;
        control_panel_widget_ = nullptr;
    }
}

/**
 * @brief Save plugin settings
 * @details This function is called by the framework to save the plugin settings.
 * @param plugin_settings Plugin settings provided by the framework
 * @param instance_settings Instance settings provided by the framework
 */
void erp42::rqt_plugin::ControlPanelPlugin::saveSettings(
    qt_gui_cpp::Settings & plugin_settings, 
    qt_gui_cpp::Settings & instance_settings
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
void erp42::rqt_plugin::ControlPanelPlugin::restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings, 
    const qt_gui_cpp::Settings & instance_settings
)
{
    Q_UNUSED(plugin_settings);
    Q_UNUSED(instance_settings);
}

/**
 * @brief Build and send an ERP42 ModeCommand via ROS 2 service.
 * @details
 *  - Verifies the client is initialized and the service is available (wait up to 200 ms).
 *  - Disables the "Apply" button while the request is in flight to prevent duplicate sends.
 *  - Populates manual/auto, E-Stop, and gear fields based on current UI state.
 *  - Shows a warning dialog on common errors (uninitialized client, service unavailable, apply failure).
 */
void erp42::rqt_plugin::ControlPanelPlugin::on_apply_mode_command()
{
    if(!mode_command_client_)
    {
        QMessageBox::warning(widget_, "ERROR", "Service client is not initialized.");
        return;
    }

    if(!mode_command_client_->wait_for_service(200ms))
    {
        QMessageBox::warning(widget_, "ERROR", "\"/erp42/mode_command\" service is not available.");
        return;
    }

    // Lock button
    control_panel_widget_->apply_mode_button->setEnabled(false);

    // Mode command
    auto mode_command_request = std::make_shared<erp42_msgs::srv::ModeCommand::Request>();

    // Control mode
    if (control_panel_widget_->auto_mode_button->isChecked())
    {
        mode_command_request->manual_mode = false;
    }
    else if(control_panel_widget_->manual_mode_button->isChecked())
    {
        mode_command_request->manual_mode = true;
    }

    // Emergency stop
    if(control_panel_widget_->estop_on_button->isChecked())
    {
        mode_command_request->emergency_stop = true;
    }
    else if(control_panel_widget_->estop_off_button->isChecked())
    {
        mode_command_request->emergency_stop = false;
    }

    // Gear
    if(control_panel_widget_->drive_button->isChecked())
    {
        mode_command_request->gear = erp42_msgs::srv::ModeCommand::Request::GEAR_DRIVE;
    }
    else if(control_panel_widget_->neutral_button->isChecked())
    {
        mode_command_request->gear = erp42_msgs::srv::ModeCommand::Request::GEAR_NEUTRAL;
    }
    else if(control_panel_widget_->reverse_button->isChecked())
    {
        mode_command_request->gear = erp42_msgs::srv::ModeCommand::Request::GEAR_REVERSE;
    }

    // Send mode command
    mode_command_client_->async_send_request(mode_command_request,
        [this](rclcpp::Client<erp42_msgs::srv::ModeCommand>::SharedFuture future)
        {
            auto response = future.get();
            if(response && response->success == false)
            {
                QMessageBox::warning(widget_, "ERROR", "Failed to apply mode.");
            }

            // Enable button
            control_panel_widget_->apply_mode_button->setEnabled(true);
        }
    );

    // Deallocation
    mode_command_request.reset();
}

/**
 * @brief Bind a QSlider to a QDoubleSpinBox with linear mapping.
 * @param slider Pointer to the slider (integer index space).
 * @param spin_box Pointer to the double spin box (continuous value space).
 * @param lower_bound Minimum value shown in the spin box.
 * @param upper_bound Maximum value shown in the spin box.
 * @param step Increment used for both the slider step and the spin box single step.
 */
void erp42::rqt_plugin::ControlPanelPlugin::bind_slider_spin_box(
    QSlider *slider, QDoubleSpinBox *spin_box, double lower_bound, double upper_bound, double step
)
{
    // Slider settings
    const int step_num = static_cast<int>(std::lround((upper_bound - lower_bound) / step));
    slider->setRange(0, step_num);

    // Spin box settings
    spin_box->setRange(lower_bound, upper_bound);
    spin_box->setSingleStep(step);
    spin_box->setDecimals(2);

    // Slider to spin box
    QObject::connect(slider, &QSlider::valueChanged, spin_box,
        [=](int slider_idx)
        {
            QSignalBlocker blocker(spin_box);
            double value = lower_bound + slider_idx * step;
            spin_box->setValue(value);
        }
    );

    // Spin box to slider
    QObject::connect(spin_box, qOverload<double>(&QDoubleSpinBox::valueChanged), slider,
        [=](double spin_box_value)
        {
            int step_idx = static_cast<int>(std::lround((spin_box_value - lower_bound) / step));
            step_idx = std::clamp(step_idx, 0, step_num);
            QSignalBlocker blocker(slider);
            slider->setValue(step_idx);
        }
    );
}

/**
 * @brief Bind a QSlider to a QDoubleSpinBox with linear mapping.
 * @param slider Pointer to the slider (integer index space).
 * @param spin_box Pointer to the double spin box (continuous value space).
 * @param lower_bound Minimum value shown in the spin box.
 * @param upper_bound Maximum value shown in the spin box.
 * @param step Increment used for both the slider step and the spin box single step.
 */
void erp42::rqt_plugin::ControlPanelPlugin::bind_slider_spin_box(
    QSlider *slider, QSpinBox *spin_box, int lower_bound, int upper_bound, int step
)
{
    // Slider settings
    const int step_num = (upper_bound - lower_bound) / std::max(step, 1);
    slider->setRange(0, step_num);

    // Spin box settings
    spin_box->setRange(lower_bound, upper_bound);
    spin_box->setSingleStep(step);

    // Slider to spin box
    QObject::connect(slider, &QSlider::valueChanged, spin_box,
        [=](int slider_idx)
        {
            QSignalBlocker blocker(spin_box);
            spin_box->setValue(lower_bound + slider_idx * step);
        }
    );

    // Spin box to slider
    QObject::connect(spin_box, qOverload<int>(&QSpinBox::valueChanged), slider,
        [=](int spin_box_value)
        {
            int step_idx = (spin_box_value - lower_bound) / std::max(step, 1);
            step_idx = std::clamp(step_idx, 0, step_num);
            QSignalBlocker blocker(slider);
            slider->setValue(step_idx);
        }
    );
}

/**
 * @brief Periodic publisher callback for ControlCommand messages.
 * @details
 *  - Publishes speed [m/s], steering [rad], and brake [0..150] at the timer rate.
 *  - Steering entered in degrees is converted to radians before publishing.
 */
void erp42::rqt_plugin::ControlPanelPlugin::control_command_timer_callback()
{
    if(!control_panel_widget_->activate_control_command_checkbox->isChecked())
    {
        return;
    }

    // Speed (m/s)
    control_command_msg_.speed = control_panel_widget_->speed_spin_box->value();

    // Steering (rad)
    double steering_rad = control_panel_widget_->steering_spin_box->value() * M_PI / 180.0;
    control_command_msg_.steering = steering_rad;
    
    // Brake (0-150)
    control_command_msg_.brake = control_panel_widget_->brake_spin_box->value();
    control_command_pub_->publish(control_command_msg_);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erp42::rqt_plugin::ControlPanelPlugin, rqt_gui_cpp::Plugin)