 /*
 * lcd.c:
 *	Text-based LCD driver.
 *	This is designed to drive the parallel interface LCD drivers
 *	based in the Hitachi HD44780U controller and compatables.
 *
 * Copyright (c) 2012 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include <wiringPi.h>
#include <lcd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>

//unsigned char EncoderRead()
//{
//
//}
int Cnt = 0;
unsigned char CurVal;
unsigned char PrevVal;
int mcnt=0;


/*
 * myInterrupt:
 *********************************************************************************
 */

void EncInterrupt1 (void)
{
  char ValP5,ValC5,ValC4,ValP4,lcnt=0;
  mcnt=0;
  //  delay(1);mcnt++;
  ValP4=digitalRead(4);
  delay(1);mcnt++;
  ValC4=digitalRead(4);
  while(((ValC4!=0)||(ValP4!=0))&&(lcnt<10))
    {
    ValP4=ValC4;
    delay(1);mcnt++;
    lcnt++;
    ValC4=digitalRead(4);
    }

  //if((ValC4==0)&&(ValP4==0)&&(lcnt<=10)){
  if((ValC4==0)&&(ValP4==0)){
     ValP5=digitalRead(5);
     delay(1);mcnt++;
     ValC5=digitalRead(5);
     lcnt=0;
     while((ValC5!=ValP5)&&(lcnt<10))
       {
	 ValP5=ValC5;
	 delay(1);mcnt++;
	 ValC5=digitalRead(5);
	 lcnt++;
       }
     if(ValC5==0) Cnt++;
     else Cnt--;
   
    }
   // delay(5);
  }

//************************************************
void EncInterrupt2 (void)
{
  char ValP5,ValC5,ValC4,ValP4,lcnt=0;
  mcnt=0;
  
    ValP5=digitalRead(5);
     delay(1);mcnt++;
     ValC5=digitalRead(5);
     lcnt=0;
     while((ValC5!=ValP5)&&(lcnt<10))
       {
	 ValP5=ValC5;
	 delay(1);mcnt++;
	 ValC5=digitalRead(5);
	 lcnt++;
       }
     if(ValC5==0) Cnt++;
     else Cnt--;
   
    }


//************************************************
int main (void)
{
  int i, j,k ;
  int fd1, fd2 ; 
  int fd;
 
 int z;
  char buf[32];


  char message1 [256] ;
  char message2 [256] ;
  char buf1 [30] ;
  char buf2 [30] ;
  //  unsigned char CurVal;
  //  unsigned char PrevVal;
  //  int Cnt=0;

  struct tm *t ;
  time_t tim ;

  struct ifreq ifr;
  fd=socket(AF_INET, SOCK_DGRAM,0);
  ifr.ifr_addr.sa_family = AF_INET;
//  strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);
  strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ-1);
  ioctl(fd, SIOCGIFADDR, &ifr);
  close(fd);
  printf("%s\n",inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));		

  printf ("RPi IP test 2\n") ;

  z = gethostname(buf,sizeof buf);
  if(z==-1){
	printf(stderr, "%s: gethostname(2)\n", strerror(errno));
	exit(1);
  }
  printf("host name = '%s'\n",buf);



  z = getnameinfo(buf,sizeof buf);
  if(z==-1){
	printf(stderr, "%s: getnameinfo(2)\n", strerror(errno));
	exit(1);
  }
  printf("name info = '%s'\n",buf);



  if (wiringPiSetup () == -1)
    exit (1) ;

  if (wiringPiISR (4, INT_EDGE_FALLING, &EncInterrupt1) == -1)
    exit (1);
  //{
  //    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
  //    return 1 ;
  //  }


  fd1 = lcdInit (2, 16, 4, 11, 10, 0,1,2,3,0,0,0,0) ;

  if (fd1 == -1)
  {
    printf ("lcdInit 1 failed\n") ;
    return 1 ;
  }

pinMode (4, INPUT) ; 	// Pin 4 EncA  
pinMode (5, INPUT) ; 	// Pin 5 EncB
pinMode (6, INPUT) ; 	// Pin 6 EncSw
CurVal = digitalRead(5);
PrevVal = CurVal;

//  sleep (1) ;

  lcdPosition (fd1, 0, 0) ; lcdPuts (fd1, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
//  lcdPosition (fd1, 0, 0) ; lcdPuts (fd1, "IP test");
//  sleep (2) ;
k=1;

  while(!k)
  {
    i = 0 ;
    j = 0 ;
     sprintf (buf2, "Button Pressed") ;

    while(!k)
    {

      //      CurVal = (digitalRead(4)<<1)|digitalRead(5);



      //      tim = time (NULL) ;
      //      t = localtime (&tim) ;

      //      sprintf (buf1, "    %02d:%02d:%02d    ", t->tm_hour, t->tm_min, t->tm_sec) ;
      sprintf (buf1, "    %04d/%04d      ", Cnt,mcnt);
      if (digitalRead (6) == 1){
	lcdPosition (fd1, 0, 1) ;
	lcdPuts (fd1, buf1);
      }
      else{
	Cnt = 0;
	lcdPosition (fd1, 0, 1) ;
	lcdPuts (fd1,buf2);
      }
 //     sprintf (buf1, "%02d/%02d/%02d", t->tm_mday, t->tm_mon + 1, t->tm_year+1900) ;
 //     lcdPosition (fd1, 0, 3) ;
 //     lcdPuts (fd1, buf1) ;

      delay (1) ;
    }
  }

  return 0 ;
}
