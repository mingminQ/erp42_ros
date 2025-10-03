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
 * @file    vehicle_parameters.hpp
 * @brief   ERP42 platform vehicle parameters
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_DESCRIPTION__VEHICLE_PARAMETERS_HPP_
#define ERP42_DESCRIPTION__VEHICLE_PARAMETERS_HPP_

namespace erp42
{
    // ERP42 kinematic parameters
    static constexpr double BASE_X_SIZE {2.020};
    static constexpr double BASE_Y_SIZE {1.160};
    static constexpr double BASE_Z_SIZE {0.550};

    static constexpr double WHEELBASE_LENGTH {1.040};
    static constexpr double TRACK_WIDTH      {0.985};

    static constexpr double WHEEL_RADIUS {0.270};
    static constexpr double WHEEL_WIDTH  {0.175};

    // Encoder parameters
    static constexpr double ENCODER_PPR {100.0};
    static constexpr double ENCODER_CPR {400.0};

} // namespace erp42

#endif // ERP42_DESCRIPTION__VEHICLE_PARAMETERS_HPP_