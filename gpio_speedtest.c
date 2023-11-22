#include "wiringPi.h"
#include <stdio.h>
#include <sys/resource.h>
#include <errno.h>

#define PIN 7
#define GPIO_BANK 3

void setprio() {
  id_t pid = getpid();
  int ret = setpriority(PRIO_PROCESS, pid, 10000);
  if (ret < 0) {
    fprintf(stderr, "Failed to set prio: %d\n", errno);
  }
}

int main() {


  setprio();
  if (wiringPiSetup(GPIO_BANK)) {
    fprintf(stderr, "Failed to configure GPIO\n");
  }

  pinMode(PIN, OUTPUT);

  uint32_t count = 4e10;
  while (count) {
    digitalWrite(PIN, LOW);
    digitalWrite(PIN, HIGH);
    count--;
  }
  cleanup();
}