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
 * @file    serial_packet.hpp
 * @brief   ERP42 serial packet byte name wrapper and factors
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_SERIAL_SERIAL_PACKET_HPP_
#define ERP42_SERIAL_SERIAL_PACKET_HPP_

namespace erp42
{
    // ERP42 serial tx packet index mapping
    namespace TX
    {
        enum ByteName
        {
            STX_S          = 0,
            STX_T          = 1,
            STX_X          = 2,
            CONTROL_MODE   = 3,
            EMERGENCY_STOP = 4,
            GEAR           = 5,
            SPEED_RAW_0    = 6,
            SPEED_RAW_1    = 7,
            STEERING_100_0 = 8,
            STEERING_100_1 = 9,
            BRAKE          = 10,
            HEARTBEAT      = 11,
            ETX_0          = 12,
            ETX_1          = 13,
            PACKET_SIZE    = 14

        }; // enum ByteName

        // Speed(rad) -> Raw byte command
        static constexpr double SPEED_FACTOR {36.0};

        // Steering(rad) -> Raw byte command
        static constexpr double STEERING_FACTOR {-4068.00034543};

    } // namespace TX

    // ERP42 serial rx packet index mapping
    namespace RX
    {
        enum ByteName
        {
            STX_S          = 0,
            STX_T          = 1,
            STX_X          = 2,
            CONTROL_MODE   = 3,
            EMERGENCY_STOP = 4,
            GEAR           = 5,
            SPEED_RAW_0    = 6,
            SPEED_RAW_1    = 7,
            STEERING_100_0 = 8,
            STEERING_100_1 = 9,
            BRAKE          = 10,
            ENCODER_0      = 11,
            ENCODER_1      = 12,
            ENCODER_2      = 13,
            ENCODER_3      = 14,
            HEARTBEAT      = 15,
            ETX_0          = 16,
            ETX_1          = 17,
            PACKET_SIZE    = 18

        }; // enum ByteName
    
        // Raw byte command -> Speed (m/s)
        static constexpr double SPEED_FACTOR {0.02777777777};

        // Raw byte command -> Steering (rad)
        static constexpr double STEERING_FACTOR {0.00024582102};

    } // namespace RX

} // namespace erp42

#endif // ERP42_SERIAL_SERIAL_PACKET_HPP_