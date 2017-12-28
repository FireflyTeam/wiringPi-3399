#include "wiringPi.h"
//#include "softPwm.h"
#include <stdio.h>
#include <stdlib.h>

int main ()
{ 
  int i,j=14;
  if(wiringPiSetup() == -1)
  {
    exit(1);
  }
  pinMode(j ,1);
  for(;;)
  {  
    /*for(i=0;i<1024;++i)
    {
     pwmWrite(j,i);
     delay(1);
    }
   
    for(i=1023;i>=0;--i)
    {
     pwmWrite(j,i);
     delay(1);
    }*/
//    digitalWrite(j,0);
    softPwmCreate(j,0,100);
    softPwmWrite(j,50);
    delay(500);
  }
  return 0;
}
