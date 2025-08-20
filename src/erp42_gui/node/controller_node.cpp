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
 * @file    controller_node.cpp
 * @brief   ERP42 platform Qt5 GUI controller node
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gui/feedback_monitor.hpp"
#include "erp42_gui/controller.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QMenuBar>
#include <QSettings>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // Host main window
    QMainWindow main_window;
    main_window.setDockOptions(QMainWindow::AllowTabbedDocks | QMainWindow::AllowNestedDocks);
    main_window.setCentralWidget(new QWidget(&main_window));

    // Dock widgets
    auto* controller       = new erp42::Controller(&main_window);
    auto* feedback_monitor = new erp42::FeedbackMonitor(&main_window);

    controller->setWindowTitle("Controller");
    feedback_monitor->setWindowTitle("Feedback Monitor");

    controller->setAllowedAreas(Qt::AllDockWidgetAreas);
    feedback_monitor->setAllowedAreas(Qt::AllDockWidgetAreas);

    controller->setFeatures(
        QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);
    feedback_monitor->setFeatures(
        QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);

    // Place dock widgets
    main_window.addDockWidget(Qt::LeftDockWidgetArea, controller);
    main_window.addDockWidget(Qt::LeftDockWidgetArea, feedback_monitor);
    main_window.splitDockWidget(controller, feedback_monitor, Qt::Horizontal);

    // View menu
    QMenu* view_menu = main_window.menuBar()->addMenu("&view");
    view_menu->addAction(controller->toggleViewAction());
    view_menu->addAction(feedback_monitor->toggleViewAction());

    // Show window
    main_window.resize(910, 650);
    main_window.show();

    // ROS executor
    rclcpp::executors::SingleThreadedExecutor exececutor;
    exececutor.add_node(controller->get_node_base_interface());
    exececutor.add_node(feedback_monitor->get_node_base_interface());
    std::thread ros_thread(
        [&exececutor]() 
        {
            exececutor.spin(); 
        }
    );

    // Exit process
    int exit_code = app.exec();
    exececutor.cancel();
    if(ros_thread.joinable())
    {
        ros_thread.join();
    }

    rclcpp::shutdown();
    return exit_code;
}