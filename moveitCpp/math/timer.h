//
// Created by andreas on 06.11.20.
//

#ifndef MOVEIT_TIMER_H
#define MOVEIT_TIMER_H
#include <ctime>

class Timer
{
public:
    Timer();
    double elapsed();
    void reset();

private:
    timespec tBegin, tEnd;
};
#endif //MOVEIT_TIMER_H
