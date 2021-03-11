//
// Created by andreas on 06.11.20.
//

#include "timer.h"
/**
 * @fn Timer::Timer()
 * @brief constructor, starts the timer as soon as object is created
 */
Timer::Timer()
{
    reset();
}

/**
 * @fn double Timer::elapsed()
 * @brief returns the time elapsed since last reset
 * @return time elapsed sind last reset
 */
double Timer::elapsed()
{
    clock_gettime(CLOCK_REALTIME, &tEnd);
    return tEnd.tv_sec - tBegin.tv_sec + (tEnd.tv_nsec - tBegin.tv_nsec) / 1000000000.;
}

/**
 * @fn void Timer::reset()
 * @brief reset timer
 */
void Timer::reset()
{
    clock_gettime(CLOCK_REALTIME, &tBegin);
}
