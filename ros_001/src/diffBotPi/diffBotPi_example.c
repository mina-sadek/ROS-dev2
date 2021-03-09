
#include "diffBotPi.h"
#define PWM_MIN 500
#define PWM_STEP 10

int main (int argc, char *argv[])
{
  int i, pwm_val;
  if (robot_init() <= -1)
    return 1;

  for (i = 0; i < 4; i++)
  {
    pwm_val = PWM_MIN + (PWM_STEP * (i + 1));
    printf("robot runngin @ speed = %d\n", pwm_val);
    robot_move(FORWARD, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
    robot_move(BACKWARD, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
    robot_move(RIGHT, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
    robot_move(LEFT, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
  }

  robot_exit();
  return 0;
}
