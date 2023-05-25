#include "DualLineFollower.h"

DualLineFollower::DualLineFollower(int left_sensor_pin, int right_sensor_pin, double sensor_distance_mm):
    _left_sensor(left_sensor_pin),
    _right_sensor(right_sensor_pin),
    SENSOR_DISTANCE_MM(sensor_distance_mm),
    READ_FREQUENCY_MS(5),
    _prev_read_time(0)
{
}

void DualLineFollower::run()
{
    run(millis());
}


void DualLineFollower::run(int curr_time)
{
    if (curr_time - _prev_read_time > READ_FREQUENCY_MS)
    {

    }
}