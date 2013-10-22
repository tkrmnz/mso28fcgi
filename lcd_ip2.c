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

int Cnt = 0;
unsigned char CurVal;
unsigned char PrevVal;
int mcnt=0;
char message[100];

/*
 * myInterrupt:
 *********************************************************************************
 */

void EncInterrupt1 (void)
{
  char ValP5,ValC5,ValC4,ValP4,lcnt=0;

  mcnt=0;
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

//---------------------------------
void EncInterrupt2 (void)
{
  char ValP5,ValC5,lcnt=0;

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
//-----------------------------------------------
void scrollMessage(int lcd, int line, int width)
{
  char buf[32];
  static int position = 0;
  strncpy (buf, &message[position], width);
  buf[width]=0;
  lcdPosition(lcd, 0,line);
  lcdPuts(lcd, buf);
  if(++position == (strlen(message)-width))
    position = 0;
}

//-----------------------------------------------
int main (void)
{
  int k ;
  int fd1;
  int fd;
  FILE *pp;
  FILE *fp;

  char wip[16];
  char cBuf[32];
  int z;

  char buf[32];

  char buf1 [30] ;
  char buf2 [30] ;
  char urlstr[30];
  char wname[32];
  char ethstr[32];

  struct ifreq efr;
  struct ifreq wfr;

  fd=socket(AF_INET, SOCK_DGRAM,0);
  efr.ifr_addr.sa_family = AF_INET;
  strncpy(efr.ifr_name, "eth0", IFNAMSIZ-1);
  ioctl(fd, SIOCGIFADDR, &efr);
  close(fd);

  fd=socket(AF_INET, SOCK_DGRAM,0);
  wfr.ifr_addr.sa_family = AF_INET;
  strncpy(wfr.ifr_name, "wlan0", IFNAMSIZ-1);
  ioctl(fd, SIOCGIFADDR, &wfr);
  close(fd);

  strncpy(wip,inet_ntoa(((struct sockaddr_in *)&wfr.ifr_addr)->sin_addr),12);

  printf("%s\n",inet_ntoa(((struct sockaddr_in *)&efr.ifr_addr)->sin_addr));
  printf("%s\n",inet_ntoa(((struct sockaddr_in *)&wfr.ifr_addr)->sin_addr));
  sprintf(cBuf,"nslookup %s localhost\n",wip);
  pp = popen(cBuf,"r");
  if(pp != NULL){
    while (1){
     char *line;
     char lbuf[1000];
     int i =0;

     line = fgets(buf, sizeof lbuf, pp);
     if(line==NULL) break;
     else {
	if(strlen(line) >1){


	while(i<strlen(line)){
		if((line[i]=='n')&&(line[i+1]=='a')&&(line[i+2]=='m')&&(line[i+3]=='e'))
	    		{
			strcpy(&wname,&line[i+7]);
			printf("%s\n",wname);
			break;
			}
		else  i++;

		}
	}//if >1
    }//else null
  }//while(1)
}//pp
  pclose(pp);


  strcpy(wname+(strlen(wname)-1),"  ");//padding spaces
  sprintf(urlstr,"      URL=      ");

  printf ("RPi IP test 2\n") ;

  z = gethostname(buf,sizeof buf);
  if(z==-1){
	printf(stderr, "%s: gethostname(2)\n", strerror(errno));
	exit(1);
  }
  printf("host name = '%s'\n",buf);
  sprintf (buf1, "Wifi AP=%s", &buf);
  sprintf (ethstr, "  Ethernet IP   ");



  if (wiringPiSetup () == -1)
    exit (1) ;

  if (wiringPiISR (4, INT_EDGE_FALLING, &EncInterrupt1) == -1)
    exit (1);


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

  k=0;

  while(!k)
  {
    if(((Cnt/2)*2)==Cnt)
    {
      lcdPosition (fd1, 0, 0) ; lcdPuts (fd1, buf1);

 //     lcdPosition (fd1, 0, 1) ; lcdPuts (fd1, urlstr);
      lcdPosition (fd1, 0, 1) ; lcdPuts (fd1, wname);
//  lcdPosition (fd1, 0, 1) ; lcdPuts (fd1, inet_ntoa(((struct sockaddr_in *)&wfr.ifr_addr)->sin_addr));

    }
    else
    {

    lcdPosition (fd1, 0, 0) ; lcdPuts (fd1, ethstr);
    lcdPosition (fd1, 0, 1) ; lcdPuts (fd1, inet_ntoa(((struct sockaddr_in *)&efr.ifr_addr)->sin_addr));
    }
      delay (1) ;
  }

  return 0 ;
}
