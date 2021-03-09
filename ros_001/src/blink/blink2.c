#include <stdio.h>
#include <wiringPi.h>
#define LED 18          // BCM
//#define LED 1         // wiringpi

int main (void)
{
  int bright;
  printf ("Raspberry Pi blink\n") ;

  if (wiringPiSetupGpio() == -1)        // BCM
//  if (wiringPiSetup() == -1)          // wiringpi
    return 1 ;

  //pinMode (LED, OUTPUT) ;         // aka BCM_GPIO pin 17
  pinMode(LED, PWM_OUTPUT);

  for(;;)
  {
    for (bright = 0; bright < 2048; bright++)
    {
      pwmWrite(LED, bright);
      delay(1);
    }
  }
  return 0 ;
}

