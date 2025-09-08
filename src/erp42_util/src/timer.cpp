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
 * @file    timer.cpp
 * @brief   Elapsed time measurement utility
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_util/timer.hpp"
#include "erp42_util/log.hpp"

using namespace std::chrono;

/**
 * @brief Constructs a Timer instance.
 * @details The internal flag indicating whether the start time is set is initialized to false.
 */
erp42::Timer::Timer()
  : starting_time_set_(false)
{
}

/**
 * @brief Starts the timing interval.
 * @details Records the current time using std::chrono::steady_clock::now() 
 * and marks the timer as started.
 */
void erp42::Timer::start()
{
    starting_time_     = steady_clock::now();
    starting_time_set_ = true;
}

/**
 * @brief Ends the timing interval and logs the elapsed time.
 * @details Computes elapsed time since the last start() call in microseconds, converts it
 * to seconds as a double, and logs it with INFO level.
 */
void erp42::Timer::end()
{
    if(!starting_time_set_)
    {
        ERP42_ERROR("Timer::end() Starting time is not set, call Timer::start() first.");
        return;
    }

    time_point<steady_clock> current_time = steady_clock::now();
    auto elapsed_time_us  = duration_cast<microseconds>(current_time - starting_time_).count();
    double elapsed_time_s = static_cast<double>(elapsed_time_us) * 1e-6;

    ERP42_INFO("Timer::end() Elapsed time is %lf sec.", elapsed_time_s);
    starting_time_set_ = false;
}
