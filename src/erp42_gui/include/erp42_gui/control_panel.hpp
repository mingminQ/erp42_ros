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
 * @file    control_panel.hpp
 * @brief   ERP42 platform Qt5 GUI control panel
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_GUI_CONTROL_PANEL_HPP_
#define ERP42_GUI_CONTROL_PANEL_HPP_

#include "erp42_msgs/msg/control_command.hpp"
#include "erp42_msgs/srv/mode_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QSlider>

#include <QApplication>
#include <csignal>

namespace Ui
{
    // Forwarding Qt5 GUI control dock widget class
    class ControlPanelDockWidget;

} // namespace Ui

namespace erp42
{
    /**
     * @brief ERP42 Qt5 GUI control panel integrating a QDockWidget with an rclcpp::Node.
     * @details Initializes the UI and ROS 2 entities, binds sliders/spin boxes to vehicle controls,
     * publishes ControlCommand at a fixed wall-timer rate, and sends ModeCommand via a service.
     */
    class ControlPanel : public QDockWidget, public rclcpp::Node
    {
    // Qt5 Object macro
    Q_OBJECT

    // "ControlPanel" member function
    public:

        /**
         * @brief Construct the ERP42 GUI control panel widget and initialize the ROS node.
         * @param parent Optional Qt parent widget.
         * @details Calls initialize_node() to set up timers, publishers, service clients, and the UI.
         */
        explicit ControlPanel(QWidget *parent = nullptr);

        /**
         * @brief Destructor.
         * @details Performs no explicit teardown; 
         * Qt child widgets and rclcpp resources are released via RAII.
         */
        ~ControlPanel() override;

    private:

        /**
         * @brief Bind a QSlider to a QSpinBox with linear mapping.
         * @param slider Pointer to the slider (integer index space).
         * @param spin_box Pointer to the integer spin box.
         * @param lower_bound Minimum value shown in the spin box.
         * @param upper_bound Maximum value shown in the spin box.
         * @param step Increment used for both the slider step and the spin box single step.
         */
        template <typename SpinBoxType, typename T>
        void bind_slider_spin_box(QSlider *slider, SpinBoxType *spin_box, T lower_bound, T upper_bound, T step);

        /**
         * @brief Build and send an ERP42 ModeCommand via ROS 2 service.
         * @details
         *  - Verifies the client is initialized and the service is available (wait up to 200 ms).
         *  - Disables the "Apply" button while the request is in flight to prevent duplicate sends.
         *  - Populates manual/auto, E-Stop, and gear fields based on current UI state.
         *  - Shows a warning dialog on common errors (uninitialized client, service unavailable, apply failure).
         */
        void send_mode_command();

        /**
         * @brief Periodic publisher callback for ControlCommand messages.
         * @details
         *  - Publishes speed [m/s], steering [rad], and brake [0..150] at the timer rate.
         *  - Steering entered in degrees is converted to radians before publishing.
         */
        void control_command_timer_callback();

        /**
         * @brief Initialize timers, publishers, service clients, initial message values, and the Qt UI.
         * @details
         *  - Creates a 20 ms wall timer (≈50 Hz) to publish ControlCommand.
         *  - Initializes a latched publisher with QoS KeepLast(1), reliable, volatile durability.
         *  - Creates a ModeCommand service client.
         *  - Zero-initializes the outgoing ControlCommand message.
         *  - Builds and wires up the Qt UI, including button and slider-spin bindings.
         */
        void initialize_node();

    // "ControlPanel" member variables
    private:

        // Qt5 GUI control widget
        std::unique_ptr<Ui::ControlPanelDockWidget> control_panel_widget_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Publishers
        rclcpp::Publisher<erp42_msgs::msg::ControlCommand>::SharedPtr control_command_pub_;

        // Service clients
        rclcpp::Client<erp42_msgs::srv::ModeCommand>::SharedPtr mode_command_client_;

        // Control command
        erp42_msgs::msg::ControlCommand control_command_msg_;

    }; // class ControlPanel

} // namespace erp42

#endif // ERP42_GUI_CONTROL_PANEL_HPP_