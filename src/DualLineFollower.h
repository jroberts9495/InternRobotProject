#ifndef DualLineFollower_H
#define DualLineFollower_H

#include "MeSingleLineFollower.h"

#include <stdint.h>

enum DepartureDirection
{
    ON_LINE,
    ONE_OFF_TOWARDS_RIGHT,
    ONE_OFF_TOWARDS_LEFT,
    BOTH_OFF_TOWARDS_RIGHT,
    BOTH_OFF_TOWARDS_LEFT
};

class DualLineFollower
{
    public:
        DualLineFollower(const int &left_sensor_pin, const int &right_sensor_pin, const double &sensor_distance_mm);
        void run();
        void run(const unsigned long &curr_time);
        bool onLine();
        int time_since_departure();
        DepartureDirection direction_of_departure(); // return enum with { ON_LINE=0, ONE_OFF_TOWARDS_RIGHT=1, ONE_OFF_TOWARDS_LEFT=2, BOTH_OFF_TOWARDS_RIGHT=3, BOTH_OFF_TOWARDS_LEFT=4 }
        double speed_of_departure(); // number of ms between left and right sensor departing, divided by the sensor distance

    private:
        MeSingleLineFollower _left_sensor;
        MeSingleLineFollower _right_sensor;
        const double SENSOR_DISTANCE_MM;
        unsigned long READ_FREQUENCY_MS;
        unsigned long _prev_read_time;
        int _time_since_departure;
        int _dir_of_departure;
        double _speed_of_departure;
};

#endif