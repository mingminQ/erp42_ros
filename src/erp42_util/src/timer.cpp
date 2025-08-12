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
 *          and marks the timer as started.
 */
void erp42::Timer::start()
{
    starting_time_     = steady_clock::now();
    starting_time_set_ = true;
}

/**
 * @brief Ends the timing interval and logs the elapsed time.
 * @details Computes elapsed time since the last start() call in microseconds, converts it
 *          to seconds as a double, and logs it with INFO level.
 */
void erp42::Timer::end()
{
    if(!starting_time_set_)
    {
        ERP42_ERROR("Timer::end() starting time is not set, call Timer::start() first");
        return;
    }

    time_point<steady_clock> current_time = steady_clock::now();
    auto elapsed_time_us  = duration_cast<microseconds>(current_time - starting_time_).count();
    double elapsed_time_s = static_cast<double>(elapsed_time_us) * 1e-6;

    ERP42_INFO("Timer::end() elapsed time is %lf sec", elapsed_time_s);
    starting_time_set_ = false;
}