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
 * @file    serial_bridge_node.cpp
 * @brief   ERP42 platform serial bridge executable node
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_serial/serial_bridge.hpp"
#include "erp42_util/log.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<erp42_serial::SerialBridge> node;

    try
    {
        node = std::make_shared<erp42_serial::SerialBridge>();
        rclcpp::spin(node);
    }
    catch(const std::exception &ex)
    {
        ERP42_ERROR("%s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
}