#include "DualLineFollower.h"

DualLineFollower::DualLineFollower(int left_sensor_pin, int right_sensor_pin, double sensor_distance_mm):
    _left_sensor(left_sensor_pin),
    _right_sensor(right_sensor_pin),
    SENSOR_DISTANCE_MM(sensor_distance_mm),
    READ_FREQUENCY_MS(5),
    _prev_read_time(0),
    _time_since_departure(0),
    _dir_of_departure(ON_LINE),
    _speed_of_departure(0)
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
        if (onLine())
        {
            _dir_of_departure = ON_LINE;
            _time_since_departure = 0;
            _speed_of_departure = 0;
        }
        else
        {
            _time_since_departure += READ_FREQUENCY_MS;
            if (_left_sensor.onLine())
            {
                _dir_of_departure = ONE_OFF_TOWARDS_RIGHT;
            }
            else if (!_right_sensor.onLine() && _dir_of_departure == ONE_OFF_TOWARDS_RIGHT)
            {
                _dir_of_departure = BOTH_OFF_TOWARDS_RIGHT;
                if (!_speed_of_departure)
                {
                    _speed_of_departure = _time_since_departure / SENSOR_DISTANCE_MM;
                }
            }
            if (_right_sensor.onLine())
            {
                _dir_of_departure = ONE_OFF_TOWARDS_LEFT;
            }
            else if (!_left_sensor.onLine() && _dir_of_departure == ONE_OFF_TOWARDS_LEFT)
            {
                _dir_of_departure = BOTH_OFF_TOWARDS_LEFT;
                if (!_speed_of_departure)
                {
                    _speed_of_departure = _time_since_departure / SENSOR_DISTANCE_MM;
                }
            }
        }
    }
}

bool DualLineFollower::onLine()
{
    return _left_sensor.onLine() && _right_sensor.onLine();
}

int DualLineFollower::time_since_departure()
{
    return _time_since_departure;
}

DepartureDirection DualLineFollower::direction_of_departure()
{
    return _dir_of_departure;
}

double DualLineFollower::speed_of_departure()
{
    return _speed_of_departure;
}
