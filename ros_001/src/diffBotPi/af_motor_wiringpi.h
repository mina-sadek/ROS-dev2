#ifndef AF_MOTOR_WIRINGPI_H_
#define AF_MOTOR_WIRINGPI_H_

#include <stdio.h>
//#include <pigpio.h>
#include <wiringPi.h>
//#include <softPwm.h>

/*
   This code may be used to drive the Adafruit (or clones) Motor Shield.
   The code as written only supports DC motors.
   http://shieldlist.org/adafruit/motor

   The shield pinouts are

   D12 MOTORLATCH
   D11 PMW motor 1
   D10 Servo 1
   D9  Servo 2
   D8  MOTORDATA

   D7  MOTORENABLE
   D6  PWM motor 4
   D5  PWM motor 3
   D4  MOTORCLK
   D3  PWM motor 2

   The motor states (forward, backward, brake, release) are encoded using the
   MOTOR_ latch pins.  This saves four gpios.

----------------------------
software PWM functions in wiringpi library
----------------------------
int softPwmCreate (int pin, int initialValue, int pwmRange) ;

This creates a software controlled PWM pin. You can use any GPIO pin and the pin numbering will be that of the wiringPiSetup() function you used. Use 100 for the pwmRange, then the value can be anything from 0 (off) to 100 (fully on) for the given pin.
The return value is 0 for success. Anything else and you should check the global errno variable to see what went wrong.
----------------------------
void softPwmWrite (int pin, int value) ;

This updates the PWM value on the given pin. The value is checked to be in-range and pins that haven’t previously been initialised via softPwmCreate will be silently ignored.
----------------------------

*/

typedef unsigned char uint8_t;

#define BIT(bit) (1 << (bit))

/* assign gpios to drive the shield pins */

/*      Shield      Pi */

#define MOTORLATCH  21//14
#define MOTORCLK    20//24
//#define MOTORENABLE //25	// always enabled	// connected to GND in RPI
#define MOTORDATA   16//15

#define MOTOR_1_PWM  13	//7
#define MOTOR_2_PWM  19	//8
#define MOTOR_3_PWM  6	//13
#define MOTOR_4_PWM  26	//19

/*
   The only other connection needed between the Pi and the shield
   is ground to ground. I used Pi P1-6 to shield gnd (next to D13).
*/

/* assignment of motor states to latch */

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

#define FORWARD  1
#define BACKWARD 2
#define BRAKE    3
#define RELEASE  4

// API functions
int motors_init();
void DCMotorRun(uint8_t motornum, uint8_t cmd);
void DCMotorRunSpeed(uint8_t motornum, uint8_t cmd, int speed);

// Library Inner functions
static uint8_t latch_state;
static void latch_tx(void);
static void init(void);
static void DCMotorInit(uint8_t num);

void latch_tx(void)
{
   unsigned char i;
   digitalWrite(MOTORLATCH, 0);
   digitalWrite(MOTORDATA, 0);
   for (i=0; i<8; i++)
   {
      delayMicroseconds(10);  // 10 micros delay
      digitalWrite(MOTORCLK, 0);
      if (latch_state & BIT(7-i)) digitalWrite(MOTORDATA, 1);
      else                        digitalWrite(MOTORDATA, 0);
      delayMicroseconds(10);  // 10 micros delay
      digitalWrite(MOTORCLK, 1);
   }
   digitalWrite(MOTORLATCH, 1);
}

void init(void)
{
   latch_state = 0;
   latch_tx();
//   gpioWrite(MOTORENABLE, PI_LOW);
}

void DCMotorInit(uint8_t num)
{
   switch (num)
   {
      case 1: latch_state &= ~BIT(MOTOR1_A) & ~BIT(MOTOR1_B); break;
      case 2: latch_state &= ~BIT(MOTOR2_A) & ~BIT(MOTOR2_B); break;
      case 3: latch_state &= ~BIT(MOTOR3_A) & ~BIT(MOTOR3_B); break;
      case 4: latch_state &= ~BIT(MOTOR4_A) & ~BIT(MOTOR4_B); break;
      default: return;
   }

   latch_tx();

//   printf("Latch=%08X\n", latch_state);
}

void DCMotorRun(uint8_t motornum, uint8_t cmd)
{
   uint8_t a, b;
   switch (motornum)
   {
      case 1: a = MOTOR1_A; b = MOTOR1_B; break;
      case 2: a = MOTOR2_A; b = MOTOR2_B; break;
      case 3: a = MOTOR3_A; b = MOTOR3_B; break;
      case 4: a = MOTOR4_A; b = MOTOR4_B; break;
      default: return;
   }
 
   switch (cmd)
   {
      case FORWARD:  latch_state |=  BIT(a); latch_state &= ~BIT(b); break;
      case BACKWARD: latch_state &= ~BIT(a); latch_state |=  BIT(b); break;
      case RELEASE:  latch_state &= ~BIT(a); latch_state &= ~BIT(b); break;
      default: return;
   }

   latch_tx();
//   printf("Latch=%08X\n", latch_state);
}

void pins_init()
{
   pinMode(MOTORLATCH,  OUTPUT);
//   pinMode(MOTORENABLE, OUTPUT);
   pinMode(MOTORDATA,   OUTPUT);
   pinMode(MOTORCLK,    OUTPUT);

   pinMode(MOTOR_1_PWM, PWM_OUTPUT);
   pinMode(MOTOR_2_PWM, PWM_OUTPUT);
//   pinMode(MOTOR_3_PWM, PWM_OUTPUT);
//   pinMode(MOTOR_4_PWM, PWM_OUTPUT);

   pwmWrite(MOTOR_1_PWM, 0);
   pwmWrite(MOTOR_2_PWM, 0);
//   pwmWrite(MOTOR_3_PWM, 0);
//   pwmWrite(MOTOR_4_PWM, 0);

//   init();
}

void DCMotorRunSpeed(uint8_t motornum, uint8_t cmd, int speed)
{
   uint8_t a, b;
   if ((speed > 50) && (speed < 1024))
   {
     pinMode(MOTOR_1_PWM, PWM_OUTPUT);
     pinMode(MOTOR_2_PWM, PWM_OUTPUT);
     pwmWrite(MOTOR_1_PWM, 0);
     pwmWrite(MOTOR_2_PWM, 0);
     switch (motornum)
     {
      case 1: pwmWrite(MOTOR_1_PWM, speed); break;
      case 2: pwmWrite(MOTOR_2_PWM, speed); break;
      case 3: pwmWrite(MOTOR_3_PWM, speed); break;
      case 4: pwmWrite(MOTOR_4_PWM, speed); break;
      default: return;
     }
   }
   else
   {
//if (speed <= 0)
//  speed = LOW;
//else
//  speed = HIGH;
     pinMode(MOTOR_1_PWM, OUTPUT);
     pinMode(MOTOR_2_PWM, OUTPUT);
     switch (motornum)
     {
        case 1: digitalWrite(MOTOR_1_PWM, speed); break;
        case 2: digitalWrite(MOTOR_2_PWM, speed); break;
        case 3: digitalWrite(MOTOR_3_PWM, speed); break;
        case 4: digitalWrite(MOTOR_4_PWM, speed); break;
        default: return;
     }
   }
   DCMotorRun(motornum, cmd);
}

int motors_init()
{
  int i;
  if (wiringPiSetupGpio() <= -1)
    return -1 ;

  pins_init();
  for (i = 1; i <= 4; i++)
  {
    DCMotorInit(i);
//    DCMotorRunSpeed(i, RELEASE, 0);
  }
  init();

  return 0;
}

void DCMotorExit()
{
  int i;
  for (i = 1; i <= 4; i++)
  {
    DCMotorRunSpeed(i, RELEASE, 0);
  }
}

#endif // AF_MOTOR_WIRINGPI_H_
