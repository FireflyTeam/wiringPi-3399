#include "wiringPi.h"
//#include "wiringpiI2C.h"
#include <stdlib.h>
#include <stdio.h>
int main()
{ 
   int buf[10],i,reg = 0x02,value = 0x1234;
   const int devId = 0x1c;
   wiringPiSetup();
   int fd = wiringPiI2CSetup(devId);
   printf("fd is : %d \n",fd);  
   if(fd < 0)
   {
     printf("Error setup I2C device %x \n ",devId);
     exit(1);
   }
   
/*   for(i=0;i<10;i++)
   {  
     wiringPiI2CReadReg8(fd,devId);
     buf[i] =  wiringPiI2CReadReg8(fd , 0x02)<<8|wiringPiI2CReadReg8(fd , 0x03);
     printf("buf[%d] = %x \n ",i, buf[i]);
   }*/
  // wiringPiI2CWrite(fd,0x20);
    wiringPiI2CWriteReg16(fd,reg,value);
    delay(500);
    wiringPiI2CWriteReg16(fd,reg,value);
    buf[0] =  wiringPiI2CReadReg16(fd,reg);
    printf("buf = %02x\n",buf[0]);
   return 0; 
}
