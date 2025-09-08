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
 * @file    feedback_monitor.hpp
 * @brief   ERP42 platform Qt5 feedback monitor
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_GUI_FEEDBACK_MONITOR_HPP_
#define ERP42_GUI_FEEDBACK_MONITOR_HPP_

#include "erp42_msgs/msg/feedback.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QDockWidget>

namespace Ui
{
    // Forwarding Qt5 feedback monitor widget class
    class FeedbackMonitorDockWidget;

} // namespace Ui

namespace erp42
{
    /**
     * @brief ERP42 feedback monitor implemented as a Qt5 dock widget and ROS 2 node.
     * @details Subscribes to /erp42/feedback to display control mode, E-Stop, gear, speed (m/s),
     * steering (deg), brake, encoder, and heartbeat; reads and shows parameters from
     * "erp42_serial_bridge" (port_path, baud_rate, max_speed, max_steering_deg, steering_offset_deg).
     * UI updates are dispatched to the GUI thread via Qt::QueuedConnection for thread safety.
     */
    class FeedbackMonitor : public QDockWidget, public rclcpp::Node
    {
    // Qt5 Object macro
    Q_OBJECT

    // "FeedbackMonitor" member functions
    public:

        /**
         * @brief Constructs the feedback monitor dock widget.
         * @details Inherits from Qt @c QDockWidget and ROS 2 @c Node so that the GUI widget
         * and the ROS node operate as a single object.
         * @param parent Parent widget pointer (default: nullptr).
         */
        explicit FeedbackMonitor(QWidget *parent = nullptr);

        /**
         * @brief Destructor.
         * @details No explicit cleanup is required. Qt and ROS resources are released
         * by their respective lifecycles.
         */
        ~FeedbackMonitor() override;

    private:

        /**
         * @brief Callback for the ERP42 feedback topic.
         * @details Converts the received feedback message into human-readable strings
         * and updates the UI. Since UI updates must occur on the GUI thread,
         * this method uses @c QMetaObject::invokeMethod with @c Qt::QueuedConnection.
         * @param[in] msg Shared pointer to @c erp42_msgs::msg::Feedback .
         */
        void feedback_callback(const erp42_msgs::msg::Feedback::SharedPtr msg);

        /**
         * @brief Reads parameters from @c erp42_serial_bridge and displays them on the UI.
         * @details Read the serial bridge parameters. 
         * If the service is not available within @c 200ms, the method returns without updating.
         */
        void read_serial_bridge_parameters();

        /**
         * @brief Initializes the ROS interfaces and the Qt UI.
         * @details
         * - Subscribes to @c /erp42/feedback with QoS: KeepLast(1), reliable, volatile.
         * - Creates a synchronous parameter client for node @c erp42_serial_bridge .
         * - Sets up the Qt UI and fetches initial parameters to display.
         */
        void initialize_node();

    // "FeedbackMonitor" member variables
    private:

        // Qt5 feedback monotor widget
        std::unique_ptr<Ui::FeedbackMonitorDockWidget> feedback_monitor_widget_;

        // Subscribers
        rclcpp::Subscription<erp42_msgs::msg::Feedback>::SharedPtr feedback_sub_;

        // Parameter clients
        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;

    }; // class FeedbackMonitor

} // namespace erp42

#endif //ERP42_GUI_FEEDBACK_MONITOR_HPP_