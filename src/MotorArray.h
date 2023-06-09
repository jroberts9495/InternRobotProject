
#ifndef MotorArray_H
#define MotorArray_H

#include <stdint.h>
#include <MeMegaPiDCMotor.h>

class MotorArray
{
    public:
        MotorArray(const int &fl_pin, const int &fr_pin, const int &bl_pin, const int &br_pin, const uint16_t &wheel_width_mm, const uint16_t &wheel_depth_mm, const double &max_accel_255pmsms);
        uint8_t spin(bool clkwise);
        uint8_t MotorArray::spin(bool clkwise, const int16_t &vy);
        uint8_t crawl(const int16_t &vy);
        uint8_t crawl(const int16_t &vx, const int16_t &vy);
        uint8_t turn(int16_t vy, const uint8_t &rad, bool left);
        uint8_t stop();

        void run();
        void run(const unsigned long &curr_time);

    private:
        const uint16_t _natural_radius;
        const uint16_t _wheel_width_mm;
        const uint16_t _wheel_depth_mm;
        const double _max_accel_255pmsms;
        const unsigned long ACCELERATION_FREQUENCY;

        int16_t _curr_fl_speed;
        int16_t _curr_fr_speed;
        int16_t _curr_bl_speed;
        int16_t _curr_br_speed;

        int16_t _desired_fl_speed;
        int16_t _desired_fr_speed;
        int16_t _desired_bl_speed;
        int16_t _desired_br_speed;

        double _fl_accel_255msms;
        double _fr_accel_255msms;
        double _bl_accel_255msms;
        double _br_accel_255msms;

        MeMegaPiDCMotor _fl_motor;
        MeMegaPiDCMotor _fr_motor;
        MeMegaPiDCMotor _bl_motor;
        MeMegaPiDCMotor _br_motor;

        unsigned long _prev_accel_time;

        uint8_t calcAccelerations();
};

#endif