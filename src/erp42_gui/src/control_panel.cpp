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
 * @file    control_panel.cpp
 * @brief   ERP42 platform Qt5 GUI control panel
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gui/control_panel.hpp"
#include "erp42_util/log.hpp"
#include "ui_control_panel.h"

#include <QSignalBlocker>
#include <QMessageBox>
#include <QTimer>

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>

// Chrono literals
using namespace std::chrono_literals;

/**
 * @brief Construct the ERP42 GUI control panel widget and initialize the ROS node.
 * @param parent Optional Qt parent widget.
 * @details Calls initialize_node() to set up timers, publishers, service clients, and the UI.
 */
erp42::ControlPanel::ControlPanel(QWidget *parent)
  : QDockWidget(parent), 
    Node("erp42_control_panel")
{
    initialize_node();
}

/**
 * @brief Destructor.
 * @details Performs no explicit teardown; 
 * Qt child widgets and rclcpp resources are released via RAII.
 */
erp42::ControlPanel::~ControlPanel()
{
}

/**
 * @brief Bind a QSlider to a QDoubleSpinBox with linear mapping.
 * @param slider Pointer to the slider (integer index space).
 * @param spin_box Pointer to the double spin box (continuous value space).
 * @param lower_bound Minimum value shown in the spin box.
 * @param upper_bound Maximum value shown in the spin box.
 * @param step Increment used for both the slider step and the spin box single step.
 */
template<>
void erp42::ControlPanel::bind_slider_spin_box<QDoubleSpinBox, double>(
    QSlider *slider, QDoubleSpinBox *spin_box, double lower_bound, double upper_bound, double step)
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
 * @brief Bind a QSlider to a QSpinBox with linear mapping.
 * @param slider Pointer to the slider (integer index space).
 * @param spin_box Pointer to the integer spin box.
 * @param lower_bound Minimum value shown in the spin box.
 * @param upper_bound Maximum value shown in the spin box.
 * @param step Increment used for both the slider step and the spin box single step.
 */
template<>
void erp42::ControlPanel::bind_slider_spin_box<QSpinBox, int>(
    QSlider *slider, QSpinBox *spin_box, int lower_bound, int upper_bound, int step)
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
 * @brief Build and send an ERP42 ModeCommand via ROS 2 service.
 * @details
 *  - Verifies the client is initialized and the service is available (wait up to 200 ms).
 *  - Disables the "Apply" button while the request is in flight to prevent duplicate sends.
 *  - Populates manual/auto, E-Stop, and gear fields based on current UI state.
 *  - Shows a warning dialog on common errors (uninitialized client, service unavailable, apply failure).
 */
void erp42::ControlPanel::send_mode_command()
{
    if(!control_panel_widget_->activate_control_panel_checkbox->isChecked())
    {
        return;
    }

    if(!mode_command_client_)
    {
        ERP42_ERROR("ControlPanel::send_mode_command() Service client is not initialized.");
        QMessageBox::warning(this, "ERROR", "Service client is not initialized.");
        return;
    }

    if(!mode_command_client_->wait_for_service(200ms))
    {
        ERP42_ERROR("ControlPanel::send_mode_command() \"/erp42/mode_command\" service is not available.");
        QMessageBox::warning(this, "ERROR", "\"/erp42/mode_command\" service is not available.");
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
            if(response && response->result == false)
            {
                ERP42_ERROR("ControlPanel::send_mode_command() Failed to apply mode.");
                QMessageBox::warning(this, "ERROR", "Failed to apply mode.");
            }

            // Enable button
            control_panel_widget_->apply_mode_button->setEnabled(true);
        }
    );

    // Deallocation
    mode_command_request.reset();
}

/**
 * @brief Periodic publisher callback for ControlCommand messages.
 * @details
 *  - Publishes speed [m/s], steering [rad], and brake [0..150] at the timer rate.
 *  - Steering entered in degrees is converted to radians before publishing.
 */
void erp42::ControlPanel::control_command_timer_callback()
{
    if(!control_panel_widget_->activate_control_panel_checkbox->isChecked())
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

/**
 * @brief Initialize timers, publishers, service clients, initial message values, and the Qt UI.
 * @details
 *  - Creates a 20 ms wall timer (≈50 Hz) to publish ControlCommand.
 *  - Initializes a latched publisher with QoS KeepLast(1), reliable, volatile durability.
 *  - Creates a ModeCommand service client.
 *  - Zero-initializes the outgoing ControlCommand message.
 *  - Builds and wires up the Qt UI, including button and slider-spin bindings.
 */
void erp42::ControlPanel::initialize_node()
{
    // Timer
    timer_ = this->create_wall_timer(20ms, std::bind(&ControlPanel::control_command_timer_callback, this));

    // Publishers
    control_command_pub_ = this->create_publisher<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
    );

    // Service clients
    mode_command_client_ = this->create_client<erp42_msgs::srv::ModeCommand>(
        "/erp42/mode_command"
    );

    // Initialize data
    control_command_msg_.speed    = 0.0;
    control_command_msg_.steering = 0.0;
    control_command_msg_.brake    = 0;

    // Qt5 GUI control widget
    control_panel_widget_ = std::make_unique<Ui::ControlPanelDockWidget>();
    control_panel_widget_->setupUi(this);

    // Mode apply button
    QObject::connect(control_panel_widget_->apply_mode_button, &QPushButton::clicked, this, 
        &ControlPanel::send_mode_command);

    // Bond sliders and spin boxes
    bind_slider_spin_box<QDoubleSpinBox, double>(
        control_panel_widget_->speed_slider, control_panel_widget_->speed_spin_box, 0.0, 7.0, 0.1);

    bind_slider_spin_box<QDoubleSpinBox, double>(
        control_panel_widget_->steering_slider, control_panel_widget_->steering_spin_box, -28.0, 28.0, 1.0);
    control_panel_widget_->steering_slider->setValue(28);

    bind_slider_spin_box<QSpinBox, int>(
        control_panel_widget_->brake_slider, control_panel_widget_->brake_spin_box, 0, 150, 10);
}