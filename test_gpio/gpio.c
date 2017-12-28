#include "wiringPi.h"
#include <stdio.h>
int main()
{ 
  int a,b,c,i=13,j=14;
  wiringPiSetup();
  pinMode(i,OUTPUT);
  pinMode(j,INPUT);
  
  for(;;)
  {  
      digitalWrite(i,0);
      a=digitalRead(i);
      printf("13 is %d ",a);     
      
      c = digitalRead(j);
      printf("14 is %d  ",c);
      delay(1000);
      
      digitalWrite(i,1);
      b=digitalRead(i);
      printf("13 is %d  ",b); 

      c = digitalRead(j);
      printf("14  is %d  ",c);
      delay(1000);
  }


  return 0;
}
