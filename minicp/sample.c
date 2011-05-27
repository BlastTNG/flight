#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include "mcplib.h"

static void cleanup(int signo)
{
  printf("Stopping mcplib sample on signal %d\n", signo);
  BB_stop();

  signal(signo, SIG_DFL);
  raise(signo);
}

int main(int argc, char* argv[])
{
  unsigned char dig[6];
  unsigned int ana[4];
  int /*i = 10,*/ l=5;
  int digital = 0;

  signal(SIGHUP, cleanup);
  signal(SIGINT, cleanup);
  signal(SIGTERM, cleanup);

  BB_setOutfile(stdout);
  BB_useFramefile(1);
  BB_start();

  while(1) {
    BB_receive();

    //get digital inputs from cards 2 and 3
    dig[0] = BB_getDigital(1, 0);
    dig[1] = BB_getDigital(1, 1);
    dig[2] = BB_getDigital(1, 2);
    dig[3] = BB_getDigital(2, 0);
    dig[4] = BB_getDigital(2, 1);
    dig[5] = BB_getDigital(2, 2);

    //get the first analog input from each card
    ana[0] = BB_getAnalog(0, 0);
    ana[1] = BB_getAnalog(1, 0);
    ana[2] = BB_getAnalog(2, 0);
    ana[3] = BB_getAnalog(3, 0);

/*
    if (i <= 0) {
      printf("\nFrame %x\n", BB_getIndex());
      printf("adc2: d1 %d d2 %d d3 %d\n", dig[0], dig[1], dig[2]);
      printf("adc3: d1 %d d2 %d d3 %d\n", dig[3], dig[4], dig[5]);
      printf("digital output value: %x\n", digital);
      printf("a00 input: adc1 %x adc2 %x adc3 %x adc4 %x\n", 
	  ana[0], ana[1], ana[2], ana[3]);
      i = 10;
    } else i--;
*/

    //send a counter to digital outputs on cards 1 and 4
    BB_setDigital(1, 0, digital);
    BB_setDigital(1, 1, digital);
    BB_setDigital(1, 2, digital);
    BB_setDigital(3, 0, digital);
    BB_setDigital(3, 1, digital);
    BB_setDigital(3, 2, digital);
    if (l <= 0) {
      digital++;
      l = 5;
    } else l--;

    BB_send();

    usleep(10000);
  }

  return 0;
}

