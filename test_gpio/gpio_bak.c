#include "wiringPi.h"
#include <stdio.h>
#include <stdlib.h>
int main()
{ 
  int i=10,j=0,z=0;
  if( wiringPiSetup() == -1)
  {
    exit(1);
  }
  pinMode(i,OUTPUT);
  
  for(;;)
  {
    
      digitalWrite(i,HIGH);
      delay(500);
      j = digitalRead(i);
      printf("\n %d is %d ",i,j);


      digitalWrite(i,LOW);
      delay(500); 
      j = digitalRead(i);
      printf("\n %d is %d ",i,j);
    
  }


  return 0;
}
