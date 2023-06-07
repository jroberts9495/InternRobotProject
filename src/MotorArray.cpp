#include "MotorArray.h"
#include <math.h>

MotorArray::MotorArray(int fl_pin, int fr_pin, int bl_pin, int br_pin, uint16_t wheel_width_mm, uint16_t wheel_depth_mm, double max_accel_255pmsms):
    _wheel_depth_mm(wheel_depth_mm),
    _wheel_width_mm(wheel_width_mm),
    _natural_radius((_wheel_depth_mm+_wheel_width_mm)/2),
    _max_accel_255pmsms(max_accel_255pmsms),
    ACCELERATION_FREQUENCY(5),
    _curr_fl_speed(0),
    _curr_fr_speed(0),
    _curr_bl_speed(0),
    _curr_br_speed(0),
    _desired_fl_speed(0),
    _desired_fr_speed(0),
    _desired_bl_speed(0),
    _desired_br_speed(0),
    _fl_accel_255msms(0),
    _fr_accel_255msms(0),
    _bl_accel_255msms(0),
    _br_accel_255msms(0),
    _fl_motor(fl_pin),
    _fr_motor(fr_pin),
    _bl_motor(bl_pin),
    _br_motor(br_pin),
    _prev_accel_time(0)
{
}

void MotorArray::run()
{
    run(millis());
}

void MotorArray::run(int curr_time)
{
    if (curr_time - _prev_accel_time >= ACCELERATION_FREQUENCY)
    {
        if (_curr_fl_speed != _desired_fl_speed)
        {
            if (abs(_desired_fl_speed - _curr_fl_speed) <= abs(_fl_accel_255msms * ACCELERATION_FREQUENCY))
            {
                _curr_fl_speed = _desired_fl_speed;
                _fl_accel_255msms = 0;
                _fl_motor.run(_curr_fl_speed);
            }
            else 
            {
                _curr_fl_speed += _fl_accel_255msms * ACCELERATION_FREQUENCY;
                _fl_motor.run(_curr_fl_speed);
            }
        }
        if (_curr_fr_speed != _desired_fr_speed)
        {
            if (abs(_desired_fr_speed - _curr_fr_speed) <= abs(_fr_accel_255msms * ACCELERATION_FREQUENCY))
            {
                _curr_fr_speed = _desired_fr_speed;
                _fr_accel_255msms = 0;
                _fr_motor.run(_curr_fr_speed);
            }
            else 
            {
                _curr_fr_speed += _fr_accel_255msms * ACCELERATION_FREQUENCY;
                _fr_motor.run(_curr_fr_speed);
            }
        }
        if (_curr_bl_speed != _desired_bl_speed)
        {
            if (abs(_desired_bl_speed - _curr_bl_speed) <= abs(_bl_accel_255msms * ACCELERATION_FREQUENCY))
            {
                _curr_bl_speed = _desired_bl_speed;
                _bl_accel_255msms = 0;
                _bl_motor.run(_curr_bl_speed);
            }
            else 
            {
                _curr_bl_speed += _bl_accel_255msms * ACCELERATION_FREQUENCY;
                _bl_motor.run(_curr_bl_speed);
            }
        }
        if (_curr_br_speed != _desired_br_speed)
        {
            if (abs(_desired_br_speed - _curr_br_speed) <= abs(_br_accel_255msms * ACCELERATION_FREQUENCY))
            {
                _curr_br_speed = _desired_br_speed;
                _br_accel_255msms = 0;
                _br_motor.run(_curr_br_speed);
            }
            else 
            {
                _curr_br_speed += _br_accel_255msms * ACCELERATION_FREQUENCY;
                _br_motor.run(_curr_br_speed);
            }
        }
        _prev_accel_time = curr_time;
    }
}

uint8_t MotorArray::spin(bool clkwise)
{
    return spin(clkwise, 255);
}

uint8_t MotorArray::spin(bool clkwise, int16_t vy)
{
    _desired_fl_speed = vy * (clkwise ? -1 : 1);
    _desired_fr_speed = vy * (clkwise ? -1 : 1);
    _desired_bl_speed = vy * (clkwise ? -1 : 1);
    _desired_br_speed = vy * (clkwise ? -1 : 1);
    
    return calcAccelerations();
}

uint8_t MotorArray::crawl(int16_t vx, int16_t vy)
{
    int16_t local_max_speed = fmax(abs(vx) + abs(vy), 255);
    _desired_fl_speed = map(
        -(vy + vx),
        -local_max_speed,
        local_max_speed,
        -255,
        255
    );
    _desired_fr_speed = map(
        vy - vx,
        -local_max_speed,
        local_max_speed,
        -255,
        255
    );
    _desired_bl_speed = map(
        vx - vy,
        -local_max_speed,
        local_max_speed,
        -255,
        255
    );
    _desired_br_speed = map(
        vy + vx,
        -local_max_speed,
        local_max_speed,
        -255,
        255
    );

    return calcAccelerations();
}

uint8_t MotorArray::crawl(int16_t vy)
{
    return crawl(0, vy);
}

uint8_t MotorArray::turn(int16_t vy, uint8_t rad, bool left)
{
    if (rad > 7)
    {
        if (!left)
        {
            _desired_bl_speed = -vy;
            _desired_br_speed = 0;
            _desired_fl_speed = -vy;
            _desired_fr_speed = 0;
        }
        else
        {
            _desired_bl_speed = 0;
            _desired_br_speed = vy;
            _desired_fl_speed = 0;
            _desired_fr_speed = vy;
        }
    }
    else
    {
        if (!left)
        {
            _desired_bl_speed = -vy;
            _desired_br_speed = -vy/2;
            _desired_fl_speed = -vy;
            _desired_fr_speed = -vy/2;
        }
        else
        {
            _desired_bl_speed = vy/2;
            _desired_br_speed = vy;
            _desired_fl_speed = vy/2;
            _desired_fr_speed = vy;
        }
    }

    return calcAccelerations();
}

uint8_t MotorArray::stop()
{
    return crawl(0, 0);
}

// Returns the number of ms until wheels are at desired speed (NOT rounded to nearest ACCELERATION_FREQUENCY)
uint8_t MotorArray::calcAccelerations()
{
    const int16_t max_diff = fmax(
        fmax(
            abs(_curr_fl_speed - _desired_fl_speed),
            abs(_curr_fr_speed - _desired_fr_speed)
        ),
        fmax(
            abs(_curr_bl_speed - _desired_bl_speed),
            abs(_curr_br_speed - _desired_br_speed)
        )
    );
    const double accel_ratio = abs(max_diff) / abs(
        fmin(
            abs(max_diff),
            abs(_max_accel_255pmsms)
        )
    );
    _fl_accel_255msms = (_desired_fl_speed - _curr_fl_speed) / accel_ratio;
    _fr_accel_255msms = (_desired_fr_speed - _curr_fr_speed) / accel_ratio;
    _bl_accel_255msms = (_desired_bl_speed - _curr_bl_speed) / accel_ratio;
    _br_accel_255msms = (_desired_br_speed - _curr_br_speed) / accel_ratio;

    return ceil((_desired_fl_speed - _curr_fl_speed) / _fl_accel_255msms);
}