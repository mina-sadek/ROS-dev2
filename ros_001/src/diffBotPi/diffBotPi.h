
#ifndef DIFFWHEEL_ROBOT_H_
#define DIFFWHEEL_ROBOT_H_

#include "af_motor_wiringpi.h"

#define FORWARD	 1
#define BACKWARD 2
#define RIGHT    3
#define LEFT     4

int robot_init();
void robot_move(uint8_t direction, int speed);
void robot_stop();
void robot_exit();

int robot_init()
{
  return motors_init();
}

void robot_stop()
{
  DCMotorRunSpeed(1, RELEASE, 0);
  DCMotorRunSpeed(2, RELEASE, 0);
}

void robot_exit()
{
  robot_stop();
  DCMotorExit();
}

void robot_move(uint8_t direction, int speed)
{
  switch (direction)
  {
    case FORWARD:
      DCMotorRunSpeed(1, FORWARD, speed);
      DCMotorRunSpeed(2, FORWARD, speed);
      break;
    case BACKWARD:
      DCMotorRunSpeed(1, BACKWARD, speed);
      DCMotorRunSpeed(2, BACKWARD, speed);
      break;
    case RIGHT:
      DCMotorRunSpeed(1, BACKWARD, speed);
      DCMotorRunSpeed(2, FORWARD, speed);
      break;
    case LEFT:
      DCMotorRunSpeed(1, FORWARD, speed);
      DCMotorRunSpeed(2, BACKWARD, speed);
      break;
    default: return;
  }
}

#endif	// DIFFWHEEL_ROBOT_H_
