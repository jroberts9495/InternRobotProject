#ifndef DualLineFollower_H
#define DualLineFollower_H

#include "MeSingleLineFollower.h"

#include <stdint.h>

class DualLineFollower
{
    public:
        DualLineFollower(int left_sensor_pin, int right_sensor_pin, int sensor_distance_mm);
        void run();
        void run(int curr_time);
        // bool onLine(); 
        // int time_since_departure();
        // int direction_of_departure(); // return enum with { ON_LINE, ONE_OFF_TOWARDS_RIGHT, ONE_OFF_TOWARDS_LEFT, BOTH_OFF_TOWARDS_RIGHT, BOTH_OFF_TOWARDS_LEFT }
        // int speed_of_departure(); // number of ms between left and right sensor departing, divided by the sensor distance

    private:
        MeSingleLineFollower _left_sensor;
        MeSingleLineFollower _right_sensor;
        const int SENSOR_DISTANCE_MM;
        const int READ_FREQUENCY_MS;
        int _prev_read_time;
};

#endif