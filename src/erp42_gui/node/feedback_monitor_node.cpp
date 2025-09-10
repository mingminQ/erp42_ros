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
 * @file    feedback_monitor_node.cpp
 * @brief   ERP42 platform Qt5 feedback monitor node
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gui/feedback_monitor.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QMenuBar>
#include <QWidget>

#include <csignal>
#include <thread>

QApplication *g_app_ptr = nullptr;

void sigint_handler(int)
{
    if(g_app_ptr)
    {
        g_app_ptr->quit();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // Signal handler
    g_app_ptr = &app;
    std::signal(SIGINT, sigint_handler);

    // Host main window
    QMainWindow main_window;
    main_window.setDockOptions(QMainWindow::AllowTabbedDocks | QMainWindow::AllowNestedDocks);
    main_window.setCentralWidget(new QWidget(&main_window));

    // Dock widgets
    auto* feedback_monitor = new erp42::FeedbackMonitor(&main_window);
    feedback_monitor->setWindowTitle("Feedback Monitor");
    feedback_monitor->setAllowedAreas(Qt::AllDockWidgetAreas);
    feedback_monitor->setFeatures(
        QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);

    // Place dock widgets
    main_window.addDockWidget(Qt::LeftDockWidgetArea, feedback_monitor);

    // View menu
    QMenu* view_menu = main_window.menuBar()->addMenu("&view");
    view_menu->addAction(feedback_monitor->toggleViewAction());

    // Show window
    main_window.resize(400, 650);
    main_window.show();

    // ROS executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(feedback_monitor->get_node_base_interface());
    std::thread ros_thread(
        [&executor]() 
        {
            executor.spin(); 
        }
    );

    // Exit process
    int exit_code = app.exec();
    executor.cancel();
    if(ros_thread.joinable())
    {
        ros_thread.join();
    }

    rclcpp::shutdown();
    return exit_code;
}