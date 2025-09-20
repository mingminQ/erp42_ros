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
 * @file    timer.hpp
 * @brief   Elapsed time measurement utility
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_UTIL__TIMER_HPP_
#define ERP42_UTIL__TIMER_HPP_

#include <chrono>

namespace erp42
{
    /**
     * @brief   Lightweight stopwatch for measuring wall-clock durations.
     * @details Uses std::chrono::steady_clock to measure elapsed time 
     * between start() and end()
     */
    class Timer
    {
    // "Timer" member functions
    public:

        /**
         * @brief Constructs a Timer instance.
         * @details The internal flag indicating whether the start time is set is initialized to false.
         */
        Timer();

        /**
         * @brief Destroys the Timer.
         * @details Trivial destructor; no resources are released. It does not implicitly stop or log.
         */
        ~Timer() = default;

        /**
         * @brief Starts the timing interval.
         * @details Records the current time using std::chrono::steady_clock::now() 
         * and marks the timer as started.
         */
        void start();

        /**
         * @brief Ends the timing interval and logs the elapsed time.
         * @details Computes elapsed time since the last start() call in microseconds, converts it
         * to seconds as a double, and logs it with INFO level.
         */
        void end();

    // "Timer" member variables
    private:

        // Measurement starting time
        std::chrono::time_point<std::chrono::steady_clock> starting_time_;

        // Timer flags
        bool starting_time_set_;
        
    }; // class timer

} // namespace erp42

#endif // ERP42_UTIL__TIMER_HPP_