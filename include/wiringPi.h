// License MIT

#include "c_gpio.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

int wiringPiSetup(int gpio_bank);
int digitalRead(int gpio);
void digitalWrite(int gpio, int value);
void pinMode(int gpio, int direction);
void initialiseEpoch (void);
void delayMicrosecondsHard (unsigned int howLong);
void delayMicroseconds (unsigned int howLong);
void delay (unsigned int howLong);
unsigned int millis (void);
unsigned int micros (void);
