// Main

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include "src/MotorArray.h"
#include "src/DualLineFollower.h"
#include <MeMegaPi.h>


namespace //anonymous
{
// LOCAL CONSTANTS
const int MOTOR_ACTION_FREQUENCY(2000);
const int WHEEL_WIDTH(20);
const int WHEEL_DEPTH(30);
const int MAX_ACCELERATION_NO_SLIP_255_MS_MS(2.5);
const int MOTOR_SPEED(255);

const double LINE_TRACKER_WIDTH(4.5);

// LOCAL CLASSES
enum MotorState
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    SPIN,
    STOPPED
};

// LOCAL VARIABLES
MotorState ms = STOPPED;
int timeof_last_motor_action(0);

// If you built the MBot yourself, you were likely instructed to put barrier_right on A8 and barrier_mid on A7. Jonathan took some minor liberties with wiring, as reflected below.
MeBarrierSensor barrier_right(A7);
MeBarrierSensor barrier_mid(A8);
MeBarrierSensor barrier_left(A6);
DualLineFollower line_tracker(A9, A10, LINE_TRACKER_WIDTH);
MeCollisionSensor collide_left(A11);
MeCollisionSensor collide_right(A12);
MeNewRGBLed rgb_left(A13);
MeNewRGBLed rgb_right(A14);
MotorArray motorArray(
    1,
    10,
    9,
    2,
    WHEEL_WIDTH,
    WHEEL_DEPTH,
    MAX_ACCELERATION_NO_SLIP_255_MS_MS
);
} // end namespace //anonymous

void setup() {
  // Uncomment Serial.begin to use Serial.print.
  // Serial.begin(9600);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  rgb_left.setColor(55, 0, 0);
  rgb_left.show();
  rgb_right.setColor(55, 0, 0);
  rgb_right.show();
}

void loop() {
  int loop_time = millis();
  switch (ms)
  {
    case STOPPED:
        if (collide_left.isCollision() || collide_right.isCollision())
        {
            ms = LEFT;
            motorArray.crawl(MOTOR_SPEED, 0);
            rgb_left.setColor(55, 55, 0);
            rgb_right.setColor(55, 55, 0);
            rgb_left.show();
            rgb_right.show();
            timeof_last_motor_action = loop_time;
        }
        break;
    case LEFT:
        if (loop_time - timeof_last_motor_action >= MOTOR_ACTION_FREQUENCY)
        {
            ms = RIGHT;
            motorArray.crawl(-MOTOR_SPEED, 0);
            rgb_left.setColor(0, 55, 0);
            rgb_right.setColor(0, 55, 0);
            rgb_left.show();
            rgb_right.show();
            timeof_last_motor_action = millis();
        }
        break;
    case RIGHT:
        if (loop_time - timeof_last_motor_action >= MOTOR_ACTION_FREQUENCY)
        {
            ms = FORWARD;
            motorArray.crawl(MOTOR_SPEED);
            rgb_left.setColor(0, 55, 55);
            rgb_right.setColor(0, 55, 55);
            rgb_left.show();
            rgb_right.show();
            timeof_last_motor_action = millis();
        }
        break;
    case FORWARD:
        if (loop_time - timeof_last_motor_action >= MOTOR_ACTION_FREQUENCY)
        {
            ms = BACKWARD;
            motorArray.crawl(-MOTOR_SPEED);
            rgb_left.setColor(0, 0, 55);
            rgb_right.setColor(0, 0, 55);
            rgb_left.show();
            rgb_right.show();
            timeof_last_motor_action = millis();
        }
        break;
    case BACKWARD:
        if (loop_time - timeof_last_motor_action >= MOTOR_ACTION_FREQUENCY)
        {
            ms = SPIN;
            motorArray.spin(true);
            rgb_left.setColor(55, 0, 55);
            rgb_right.setColor(55, 0, 55);
            rgb_left.show();
            rgb_right.show();
            timeof_last_motor_action = millis();
        }
        break;
    case SPIN:
        if (loop_time - timeof_last_motor_action >= MOTOR_ACTION_FREQUENCY)
        {
            ms = STOPPED;
            motorArray.stop();
            rgb_left.setColor(55, 0, 0);
            rgb_right.setColor(55, 0, 0);
            rgb_left.show();
            rgb_right.show();
            timeof_last_motor_action = millis();
        }
        break;
  }
  motorArray.run();
}
