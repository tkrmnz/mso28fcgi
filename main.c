/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * main.c
 * Copyright (C) 2012 John Yeh <tnkrmnz@gmail.com>

 * 
 * mso28cgi is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * mso28cgi is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <aio.h>
#include <errno.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcgi_config.h>
#include <fcgi_stdio.h>
#include <time.h>

//#define BAUDRATE B460800
#define BAUDRATE B921600
#define TRUE 1
#define FALSE 0

#define FIELD_LEN 250 /* how long each name or value can be */
#define NV_PAIRS 200  /* how many name=value pairs we can process */

typedef struct name_value_st {
    char name[FIELD_LEN + 1];
    char value[FIELD_LEN + 1];
} name_value;

name_value name_val_pairs[NV_PAIRS];

static int Get_input(void);
static void send_error(char *error_text);
static void load_nv_pair(char *tmp_buffer, int nv_entry_number_to_load);
static char x2c(char *what);
static void unescape_url(char *url);


static void ResetADC();
static void portconfig(int *, int len);
static unsigned char BufTransLower(unsigned char Addr, unsigned char Val);
static unsigned char BufTransUpper(unsigned char Addr, unsigned char Val);
static unsigned int SerBufInit(unsigned int StartVal);
//static void GetSerPort();
static void GetSerPort28();
static unsigned char SPI_DevSel(unsigned char Dev);
static void SPI_CH(unsigned char Dev);
static void SPI_CL(unsigned char Dev);
static void SPI_Start(unsigned char Dev);
static void SPI_Stop(unsigned char Dev);
static void SPI_Mem_Out(unsigned char mode);
//static void SPI_Write(unsigned char Dev, unsigned char Addr, unsigned char Data);
static unsigned char CheckTriggerStatus();
static unsigned char CheckTriggerStatusRaw();
static void ReadBuffer();
//static void ReadBuffer2();
static void SPI_Read_Buf_Page(unsigned char Dev, unsigned char *Buf, unsigned char Page);
static void LEDOff();
static void ParseEprom(unsigned char *Buf);
static void PrintMsoParam();
static void ReadMsoParam();
static void WriteMsoParam();
static void InitSettings();
//static void ReadMsoSettings();
static void ReadMsoSettings2();
static void WriteMsoSettings();
static int WriteMsoData();
static void ReadQueryString();
static void VoltageConvert();
static double CalcVoltageFromRawValue(unsigned short pt, double vbit, int ProbeAttn);
static double GetVbit(unsigned char Chan);
static void ConfigureHardwareMSO();
static unsigned short CalcOffsetRawValueFromVoltage(double volts,unsigned char Chan);
static void DAC_Out_SPI();
static void ClkRate_Out(unsigned char MSB, unsigned char LSB);
static char CalcRateMSBLSB(int nsRate);
static void ConfigureThresholdLevel_SPI();
static void ConfigureTriggerHardware();
static void ForceCapture();
//static void WriteTest();
static void PrintMSOSettings();
static long RotateSid(long sidT);


char * line=NULL;
unsigned char HexBuf[4096];
unsigned char DSOCtlBase;
unsigned int BufTransCnt;
int fd_w, code;
struct termios oldtio;
struct termios newtio;
unsigned int serialNumber, modelNumber, msoType;
double vbitDiv10[2],OffsetVBitDiv10[2],vbit[2],OffsetVBit[2];
unsigned short OffsetCenterValDiv10[2],OffsetCenterVal[2];
unsigned char MBuf[128],FBuf[128];
unsigned char ACDCM;
unsigned char ClkRateMSB, ClkRateLSB;
int sampleInterval;
unsigned char SlowMode;
unsigned char LogFam;
int ProbeAttn[2];
float VGND = 511.0;
float ADCMAX = 1023.0;
unsigned short OffsetDacVal[2];
int vDiv[2];
char TrigLAWord[9];
//double OffsetDbl[2];
int OffsetDbl[2];
unsigned char ACDCMode[2];
int TrigLevel[3];
unsigned char TrigSlope[3];
const unsigned char SLOPE_RISING = 0;
const unsigned char SLOPE_FALLING = 1;
int TrigPosition;
char TrigWdtmp[37];
char TrigSPIWd[37];
char TrigI2CWd[37];
unsigned char TrigSPIMode = 0;
unsigned int TrigWidth[3];
unsigned char TrigModeDSO[3];

unsigned short AnalogDataA[1024];
unsigned short AnalogDataB[1024];
unsigned char LogicData[1024];
float AnalogVoltageDataA[1024];
float AnalogVoltageDataB[1024];
unsigned char SetChanged=0;

unsigned char SerTrigWdX[8];
unsigned char TrigChSelX;
long sid=0,CurrSid=0,PrevSid=0;
unsigned char RBWait;
unsigned char MsoBusy=0;
long sidA=0,sidB=0,sidC=0,sidD=0,sidE=0;

//float AnalogDataFA[1024];
//float AnalogDataFB[1024];




typedef enum {
    TRIG_CHAN_DSO,
	TRIG_CHAN_DSO1,
	TRIG_CHAN_LA,
	TRIG_CHAN_SER_I2C,
	TRIG_CHAN_SER_SPI,
	nMaxTrigChanDefines
} TrigChanDefines ;

TrigChanDefines TrigChan;

typedef enum  {
	TRIG_DSO_EDGE,
	TRIG_DSO_GE,
	TRIG_DSO_LT,
	nMaxTrigModedefines
} TrigModeDefines;
/*
TrigModeDefines TrigModeDSO[0] = TRIG_DSO_EDGE;
TrigModeDefines TrigModeDSO[1] = TRIG_DSO_EDGE;
*/

//#define _POSIX_SOURCE 1 // POSIX compliant source 
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE; 

//-----------------------------------------------------
void GetSerPort28()
{
	DIR *dp;
	char CurDir[FILENAME_MAX];
	struct dirent *entry;
	char *dir = "/dev";
	char *sdir = NULL;
	char * sdirnm = NULL;
	int newlen;

	getcwd(CurDir, sizeof(CurDir));//get current working directory

	if((dp = opendir(dir)))
	{
		chdir(dir);//cd to dir
		while((entry = readdir(dp)))
		{//search for MSO-28
			if((strcmp(".",entry->d_name) == 0)||
				(strcmp("..",entry->d_name) == 0))
				continue;
			sdir = entry->d_name;
			if(strlen(sdir)>6)
			{
				if((sdir[0] == 'M') && (sdir[1] == 'S') && (sdir[2] == 'O')
				   && (sdir[3] == '-') && (sdir[4] == '2') && (sdir[5] == '8'))
				{
				newlen = 5 + strlen(sdir) + 2;
				sdirnm = (char *) calloc(newlen,sizeof(char));
				strcat(sdirnm,"/dev/");
				strcat(sdirnm,sdir);
				strcpy(line,sdirnm);
				free(sdirnm);
			   }//if MSO-28
			}//if Len>6
		}//readdir

	chdir("..");
	closedir(dp);
	chdir(CurDir);//Restore working directory

	}//opendir ok
	else
	{
		fprintf(stderr,"cannot open directory: %s\n", dir);
//		printf("cannot open directory: %s\n", dir);
	}//opendir ng

}
//-----------------------------------------------------
/*
void GetSerPort()
{
	FILE *fp;
//	char * line = NULL;
	size_t len = 0;
	//ssize_t read;

	// get MSO device handle from msoif.txt
	fp = fopen("msoif.txt", "r");
	if (fp == NULL)
	{
	 printf("Missing msoif.txt\n");
	 exit(0);
	}

	getline(&line, &len, fp);
	line[strlen(line) - 1] = 0; //removes last character
    fclose(fp);
}*/
//-----------------------------------------------------

//Configure serial port
void portconfig(int *device, int len)
{
	struct termios newtio;
	int cod = 0;

	newtio.c_cflag = 0;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	cod=tcsetattr(*device,TCSANOW,&newtio);
	if(cod<0)
	{
		 perror("Setting attributes failed!");
		 exit(-1);
	}
	cfsetispeed(&newtio, BAUDRATE);
	cfsetospeed(&newtio, BAUDRATE);

//	newtio.c_cflag |= (CRTSCTS | CS8 | CLOCAL | CREAD );//exp
	newtio.c_cflag |= (CS8 | CLOCAL | CREAD );//exp
	
	newtio.c_lflag = 0;

	newtio.c_iflag |= IGNPAR;
//	newtio.c_iflag |= IGNBRK;
//	newtio.c_iflag |= IGNCR;

	newtio.c_oflag = 0;

/*	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= (CRTSCTS | CS8 | CLOCAL | CREAD);//exp

	newtio.c_lflag |= NOFLSH;
	newtio.c_lflag |= FLUSHO;
	newtio.c_lflag 	&= ~(ICANON | ECHO | ECHOE| ECHOK | ISIG);

	newtio.c_iflag &= ~INPCK;
	newtio.c_iflag &= ~IXON;
	newtio.c_iflag &= ~IXOFF;
	newtio.c_iflag |= IGNBRK;
	newtio.c_iflag |= IGNCR;

	newtio.c_oflag &= ~OPOST;
*/
	newtio.c_cc[VTIME]    = 0;   // inter-character timer unused 
	newtio.c_cc[VMIN]     = len;   // blocking read until 5 chars received 


	tcflush(*device,TCIOFLUSH);//flushes untransmitted output
//	tcflush(*device,TCOFLUSH);//flushes untransmitted output
	cod=tcsetattr(*device,TCSANOW,&newtio);	//changes occurs immediately.
//	cod=tcsetattr(*device,TCSAFLUSH,&newtio);	//changes occurs immediately.
	
	if(cod<0)
	{
		 perror("Setting attributes failed!");
		 exit(-1);
	}

}
//-----------------------------------------------------

unsigned char BufTransLower(unsigned char Addr, unsigned char Val)
{
	unsigned char i;
	i = Val & 0x3f;
	if ((i & 0x20) != 0x20)
	    i |= 0x40;
	return i;
}

//-----------------------------------------------------

unsigned char BufTransUpper(unsigned char Addr, unsigned char Val)
{
	unsigned char i, j, k;
	i = Addr & 0x0f;
	j = Val & 0xc0;
	j = j >> 2;
	k = i | j;
	if ((k & 0x20) != 0x20)
	    k |= 0x40;
	return k;
}
//-----------------------------------------------------
unsigned int SerBufInit(unsigned int StartVal)
{
	//Preload @LDS~
	HexBuf[StartVal] = 0x40;
    HexBuf[StartVal+1] = 0x4C;
    HexBuf[StartVal+2] = 0x44;
    HexBuf[StartVal+3] = 0x53;
    HexBuf[StartVal+4] = 0x7E;

	return StartVal + 5;
}

//-----------------------------------------------------
//returns trigstat with open/close fd_w
unsigned char CheckTriggerStatus()
{
//	char * rdbuf = NULL;
//	size_t len = 0;
	int lcnt = 0;
	int res;
//	int resw,resw2;
	unsigned char buf[10];
	int bytes;

//	printf("%s\n",line);
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action

	if(fd_w < 0)
	{
		printf("Serial error TS\n");
		return fd_w;
	}
	else
	{
		portconfig(&fd_w,1);

//		tcflush(fd_w, TCIOFLUSH);

		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x02, 0x00);
		HexBuf[BufTransCnt++] = BufTransLower(0x02, 0x00);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		usleep(5);



		STOP = FALSE;
		while (STOP==FALSE){       // loop for input 
			ioctl(fd_w, FIONREAD, &bytes);
			if(bytes >= 1) {
				res = read(fd_w,buf,bytes);
				STOP = TRUE;
			}
			else
			{
				lcnt++;
//				write(fd_w,HexBuf,BufTransCnt);  
//				usleep(1000);
			}
				usleep(5);
		}

 		tcflush(fd_w, TCIOFLUSH);

//		res = read(fd_w,buf,1);   // returns after 5 chars have been input 
//		printf("nByte=%d val=%x cnt=%d resw=%d resw2=%d\n",res,buf[0],lcnt,resw,resw2);
//		printf("nByte=%d val=%x cnt=%d\n",res,buf[0],lcnt);

		close(fd_w);
		return buf[0];
	}
}
//-----------------------------------------------------
//returns trigstat NO open/close fd_w to save time, 
unsigned char CheckTriggerStatusRaw()
{
	int res;
	unsigned char buf[10];
	int lcnt = 0;
	int bytes;

	DSOCtlBase = 0x10;
	BufTransCnt = 0;
	BufTransCnt = SerBufInit(BufTransCnt);
	HexBuf[BufTransCnt++] = BufTransUpper(0x02, 0x00);
	HexBuf[BufTransCnt++] = BufTransLower(0x02, 0x00);
	HexBuf[BufTransCnt++] = 0x7E;
	write(fd_w,HexBuf,BufTransCnt);
	usleep(5);

/*
    tcgetattr(fd_w,&oldtio); // save current port settings
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME]    = 0;   // inter-character timer unused
	newtio.c_cc[VMIN]     = 1;   // blocking read until 5 chars received
//	tcflush(fd_w, TCIFLUSH);
	tcsetattr(fd_w,TCSANOW,&newtio);
	write(fd_w,HexBuf,BufTransCnt);  //experimental
*/

		STOP = FALSE;
		while (STOP==FALSE){       // loop for input 
			ioctl(fd_w, FIONREAD, &bytes);
			if(bytes >= 1) {
				res = read(fd_w,buf,bytes);
				STOP = TRUE;
			}
			else
			{
				lcnt++;
//				write(fd_w,HexBuf,BufTransCnt);  
//				usleep(1000);
			}
			usleep(5);
		}

 		tcflush(fd_w, TCIOFLUSH);

	/*	STOP = FALSE;
	while (STOP==FALSE){       // loop for input
		if((lcnt ==10)&&(bytes == 0)) write(fd_w,HexBuf,BufTransCnt);
		ioctl(fd_w, FIONREAD, &bytes);
		if(bytes >= 1) {
			res = read(fd_w,buf,bytes);
			buf[res]=0;               // so we can printf...
			STOP = TRUE;
		}
		else
			usleep(1000);
		lcnt++;
	}
*/
/*
	STOP = FALSE;
	while (STOP==FALSE)
	{       // loop for input
	  res = read(fd_w,buf,255);   // returns after 5 chars have been input
	  buf[res]=0;               // so we can printf...
//	  printf(":%s:%d\n", buf, res);
//	  printf(":%c:%x\n", buf[0], buf[0]);
	  if (buf[0]!=0x00) STOP=TRUE;
	}

*/
//	tcsetattr(fd_w,TCSANOW,&oldtio);
	return buf[0];
}

//-----------------------------------------------------
void ReadBuffer()
{
//	int res;
	unsigned char bbb[4200];
	int	Cnt;
	int BytesToRead, AdjAddr;
	unsigned short ii, jj, mm, kk, nn, pp;
	int bytes;
	int lcnt = 0;


	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error RB\n");
	}
	else
	{
		portconfig(&fd_w,4090);
		tcflush(fd_w, TCIOFLUSH);

		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x01, 0x00);
		HexBuf[BufTransCnt++] = BufTransLower(0x01, 0x00);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		usleep(5);

		lcnt = 0;
		STOP = FALSE;
		while ((STOP==FALSE)&&(lcnt<2000)){       // loop for input 
			ioctl(fd_w, FIONREAD, &bytes);
			if(bytes >= 4095) {
				BytesToRead = read(fd_w,bbb,bytes);
				STOP = TRUE;
			}
			else{
//				if((lcnt ==10)&&(bytes == 0)) write(fd_w,HexBuf,BufTransCnt);  
//				else usleep(1000);
//				usleep(10);
			}
			usleep(5);
			lcnt++;
		}

		tcflush(fd_w, TCIOFLUSH);

		close(fd_w);


		AdjAddr = 0;
//		BytesToRead = 4096;
//		BytesToRead = res;
	int CntPos;
	int CntAdj;
        for (Cnt = AdjAddr; Cnt < (BytesToRead / 4); Cnt++)//x4
        {
            CntPos= Cnt*4;
	    CntAdj=Cnt-AdjAddr;
	    ii = bbb[CntPos];              //dso 1
            jj = (bbb[CntPos + 1] & 0x03) << 8; //dso1

            kk = (bbb[CntPos + 1] & 0xfc) >> 2; //dso2
            mm = (bbb[CntPos + 2] & 0x0f) << 6; //dso2

            nn = (bbb[CntPos + 2] & 0xf0) >> 4; //la
            pp = (bbb[CntPos + 3] & 0x0f) << 4; //la

            AnalogDataA[CntAdj] = jj | ii;
            AnalogDataB[CntAdj] = mm | kk;
            LogicData[CntAdj] = nn | pp;
        }

   //     for (Cnt = AdjAddr; Cnt < (BytesToRead / 4); Cnt++)//x4
//		printf("%X %X %X\n",AnalogDataA[Cnt],AnalogDataB[Cnt],LogicData[Cnt]); 

//		return buf[0];
	}
}

//-----------------------------------------------------
/*void ReadBuffer2()
{
//	int res;
	unsigned char bbb[4200];
	int	Cnt;
	int BytesToRead, AdjAddr;
	unsigned short ii, jj, mm, kk, nn, pp;
	int bytes;
	int lcnt = 0;
	FILE *fp;
	char sBuf[512];


	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error RB\n");
	}
	else
	{
		portconfig(&fd_w,4090);
		tcflush(fd_w, TCIOFLUSH);

		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x01, 0x00);
		HexBuf[BufTransCnt++] = BufTransLower(0x01, 0x00);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		usleep(5);

		lcnt = 0;
		STOP = FALSE;
		while ((STOP==FALSE)&&(lcnt<2000)){       // loop for input 
			ioctl(fd_w, FIONREAD, &bytes);
			if(bytes >= 4095) {
				BytesToRead = read(fd_w,bbb,bytes);
				STOP = TRUE;
			}
			else{
//				if((lcnt ==10)&&(bytes == 0)) write(fd_w,HexBuf,BufTransCnt);  
//				else usleep(1000);
//				usleep(10);
			}
			usleep(5);
			lcnt++;
		}

		tcflush(fd_w, TCIOFLUSH);

		close(fd_w);


		AdjAddr = 0;
//		BytesToRead = 4096;
//		BytesToRead = res;
	int CntPos;
	int CntAdj;

    double VbitA,VbitB;
    int PAtnA,PAtnB;	

    VbitA=GetVbit(0);
    VbitB=GetVbit(1);
    PAtnA=ProbeAttn[0];
    PAtnB=ProbeAttn[1];


	

	sprintf(sBuf,"tmp/msodata%d.csv",int(CurrSid));
	fp = fopen(sBuf, "wb");

//  	fp = fopen("msodata.csv", "wb");
	if(fp)
	{
      for (Cnt = AdjAddr; Cnt < (BytesToRead / 4); Cnt++)//x4
        {
            CntPos= Cnt*4;
	    CntAdj=Cnt-AdjAddr;
	    ii = bbb[CntPos];              //dso 1
            jj = (bbb[CntPos + 1] & 0x03) << 8; //dso1

            kk = (bbb[CntPos + 1] & 0xfc) >> 2; //dso2
            mm = (bbb[CntPos + 2] & 0x0f) << 6; //dso2

            nn = (bbb[CntPos + 2] & 0xf0) >> 4; //la
            pp = (bbb[CntPos + 3] & 0x0f) << 4; //la

//            AnalogDataA[CntAdj] = jj | ii;
//           AnalogDataB[CntAdj] = mm | kk;
            AnalogVoltageDataA[CntAdj] =
                CalcVoltageFromRawValue( (jj | ii),
                VbitA,
                PAtnA);

            AnalogVoltageDataB[CntAdj] =
                CalcVoltageFromRawValue((mm | kk),
                VbitB,
                PAtnB);

			
			LogicData[CntAdj] = nn | pp;
			fprintf(fp,"%f\t%f\t%d\n",AnalogVoltageDataA[CntAdj],AnalogVoltageDataB[CntAdj],LogicData[CntAdj]); 
			
        }
		fclose(fp);
	}

   //     for (Cnt = AdjAddr; Cnt < (BytesToRead / 4); Cnt++)//x4
//		printf("%X %X %X\n",AnalogDataA[Cnt],AnalogDataB[Cnt],LogicData[Cnt]); 

//		return buf[0];
	}
}*/
//-----------------------------------------------------
//Issues Reset to ADC and puts the MSO in ready to be arm mode
void ResetADC()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error Rst ADC\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x40));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x40));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
char CalcRateMSBLSB(int nsRate)
{
    char ret = 0;
    SlowMode = 0x00;
    switch (nsRate)
    {
        case 1: //RIS mode 1GSa
        case 5: ClkRateMSB = 0x02; ClkRateLSB = 0x05; break;//Tstg = "200Msa/S";
        case 10: ClkRateMSB = 0x01; ClkRateLSB = 0x03; break;//Tstg = "100Msa/S";
        case 20: ClkRateMSB = 0x03; ClkRateLSB = 0x02; break;//Tstg = "50Msa/S";
        case 50: ClkRateMSB = 0x03; ClkRateLSB = 0x08; break;//Tstg = "20Msa/S" 3;
        case 100: ClkRateMSB = 0x03; ClkRateLSB = 0x12; break;//Tstg = "10Msa/S" 8;
        case 200: ClkRateMSB = 0x03; ClkRateLSB = 0x26; break;//Tstg = "5Msa/S" 18;
        case 500: ClkRateMSB = 0x03; ClkRateLSB = 0x62; break;//Tstg = "2Msa/S" 48;
        case 1000: ClkRateMSB = 0x03; ClkRateLSB = 0xc6; break;//Tstg = "1Msa/S" 98; 
        case 2000: ClkRateMSB = 0x07; ClkRateLSB = 0x8e; break;//Tstg = "500Ksa/S" 198;
        case 5000: ClkRateMSB = 0x0f; ClkRateLSB = 0xe6; break;//Tstg = "200Ksa/S" 498; 
        case 10000: ClkRateMSB = 0x1f; ClkRateLSB = 0xce; break;//Tstg = "100Ksa/S" 998;
        case 20000: ClkRateMSB = 0x3f; ClkRateLSB = 0x9e; break;//Tstg = "50Ksa/S" 1998; 
        case 50000: ClkRateMSB = 0x9f; ClkRateLSB = 0x0e; break;//Tstg = "20Ksa/S" 4998;
        //0322612 slow rate
		case 100000: ClkRateMSB = 0x03; ClkRateLSB = 0xc6; SlowMode = 0x20; break;//Tstg = "10Ksa/S" 9998;
        case 200000: ClkRateMSB = 0x07; ClkRateLSB = 0x8e; SlowMode = 0x20; break;//Tstg = "5Ksa/S" 199;  
        case 500000: ClkRateMSB = 0x0f; ClkRateLSB = 0xe6; SlowMode = 0x20; break;//Tstg = "2Ksa/S" 499;
        case 1000000: ClkRateMSB = 0x1f; ClkRateLSB = 0xce; SlowMode = 0x20; break;//Tstg = "1Ksa/S" 999;
        case 2000000: ClkRateMSB = 0x3f; ClkRateLSB = 0x9e; SlowMode = 0x20; break;//Tstg = "500sa/S" 1999; 
        case 5000000: ClkRateMSB = 0x9f; ClkRateLSB = 0x0e; SlowMode = 0x20; break;//Tstg = "20sa/S" 4999;
 //       case 10000000: ClkRateMSB = 0x9f; ClkRateLSB = 0x0f; SlowMode = 0x20; break;//Tstg = "10sa/S" 9999;
        default:
            ret = -1;
            break;
    }
    return ret;
}
//-----------------------------------------------------
void ClkRate_Out(unsigned char MSB, unsigned char LSB)
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error ClkRate\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x09, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x09, MSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0a, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x0a, LSB);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}

//-----------------------------------------------------
/*void DAC_Out(unsigned short DacVal)
{
	unsigned char MSB, LSB;
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error DAC Out\n");
	}
	else
	{
		portconfig(&fd_w);
		DSOCtlBase = 0x10;
		LSB = (unsigned char)(DacVal & 0x00ff);
		MSB = (unsigned char)((DacVal & 0xff00) >> 8);
		//DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0c, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x0c, MSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0d, LSB);//DAC LSB
		HexBuf[BufTransCnt++] = BufTransLower(0x0d, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x20));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x20));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase & 0xdf));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase & 0xdf));                                           
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
*/
//-----------------------------------------------------
void SPI_Out_DAC(unsigned char DAC, unsigned char MSB, unsigned char LSB)
{
    unsigned char SPItmp = 0x00;
    unsigned char i, j;

	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error SPI DAC\n");
	}
	else
	{
		portconfig(&fd_w,1);
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		if (DAC == 0x02) j = (SPItmp | 0x10);
		else if (DAC == 0x01) j = (SPItmp | 0x08);
		else j = SPItmp;

		HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
		HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
		for (i = 0; i <= 7; i++)
		{
		    if ((MSB & (0x80 >> i)) != 0x00) j |= 0x02;
		    else j &= 0xfd;
		    j ^= 0x01;
		    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
		    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
		    j ^= 0x01;
		    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
		    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
		}

		for (i = 0; i <= 7; i++)
		{
		    if ((LSB & (0x80 >> i)) != 0x00) j |= 0x02;
		    else j &= 0xfd;
		    j ^= 0x01;
		    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
		    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
		    j ^= 0x01;
		    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
		    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
		}
		j = SPItmp;
		HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
		HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
void DAC_Out_SPI()
{
    unsigned char MSB, LSB;

    LSB = OffsetDacVal[0] & 0x00ff;
    MSB = (OffsetDacVal[0] & 0xff00) >> 8;
    MSB |= 0x80;

    //LSB = 0x8a;
    //MSB = 0x85;//ChA offset = 0V 0x58a 0x80 selects ChA
    SPI_Out_DAC(2, MSB, LSB);//ChA

    LSB = OffsetDacVal[1] & 0x00ff;
    MSB = (OffsetDacVal[1] & 0xff00) >> 8;
    MSB |= 0x40;

    //LSB = 0x80;
    //MSB = 0x45;//ChB offset = 0V 0x580 0x40 selects ChB
    SPI_Out_DAC(2, MSB, LSB); //ChB

//    LSB = 0xc0;
//    MSB = 0x03f;//LA Threshold 
//    SPI_Out_DAC(1, MSB, LSB);//Logic Threshold


}        //set trigger value


//-----------------------------------------------------
void ConfigureThresholdLevel_SPI()
{
	unsigned char MSB, LSB;	
	unsigned short tmp = 0x3fc0;
    switch (LogFam)
    {
        case 0: tmp = 0x1740; break;    //1.2V Logic 0x8600
        case 1: tmp = 0x1d00; break;    //1.5V Logic 0x8770
        case 2: tmp = 0x22c0; break;    //1.8V Logic 0x88ff
        case 3: tmp = 0x3080; break;    //2.5V Logic 0x8c70
        case 4: tmp = 0x3a40; break;    //3.0V Logic	0x8eff
        default:
        case 5: tmp = 0x3fc0; break;    //3.3V Logic family   0x8fff
    }

    LSB = tmp & 0x00ff;
    MSB = (tmp & 0xff00) >> 8;
    SPI_Out_DAC(1, MSB, LSB);//Logic Threshold

}
//-----------------------------------------------------
double GetOffsetVBit(unsigned char Chan)
{
    if (vDiv[Chan] > 50)
		return OffsetVBitDiv10[Chan];
    else
		return OffsetVBit[Chan];
}
//-----------------------------------------------------

double GetOffsetCenterVal(unsigned char Chan)
{
    if (vDiv[Chan] > 50)
        return OffsetCenterValDiv10[Chan];
    else
        return OffsetCenterVal[Chan];
}
//-----------------------------------------------------
double GetVbit(unsigned char Chan)
{
    if (vDiv[Chan] > 50)
        return vbitDiv10[Chan];
    else
        return vbit[Chan];
}
//-----------------------------------------------------
unsigned short CalcOffsetRawValueFromVoltage(double volts,unsigned char Chan)
{ //don't need to adjust for probe atten
    unsigned short DacVal;
    int DacTmp;

	DacTmp = (int)(GetOffsetCenterVal(Chan) - ((volts / ProbeAttn[Chan]) / GetOffsetVBit(Chan)));
	
	if (DacTmp < 0) DacVal = 0x0000;
    else if (DacTmp > 0x0fff) DacVal = 0x0fff;
    else DacVal = (DacTmp);

    return (unsigned short) DacVal;
}
/*
for(i=0;i<2;i++){
		printf("vbit[%d]=%f\n",i,vbit[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetVBit[%d]=%f\n",i,OffsetVBit[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetCenterVal[%d]=0x%x\n",i,OffsetCenterVal[i]);
	}
	printf("\n");
	for(i=0;i<2;i++){
		printf("vbitDiv10[%d]=%f\n",i,vbitDiv10[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetVBitDiv10[%d]=%f\n",i,OffsetVBitDiv10[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetCenterValDiv10[%d]=0x%x\n",i,OffsetCenterValDiv10[i]);
	}

}
*/
 
//-----------------------------------------------------
			
unsigned short CalcRawValueFromVoltage(double val, unsigned char Chan)
{
    return (unsigned short)(512 - (unsigned short)((val/ProbeAttn[Chan]) / GetVbit(Chan)));
}

//-----------------------------------------------------
double CalcVoltageFromRawValue(unsigned short pt, double vbit, int ProbeAttn)
{
	double ret;
    double Vtmp, Vtmp2; 
    Vtmp = pt;
    Vtmp2 = (1023.0 - Vtmp) - VGND;
    ret = Vtmp2 * vbit * ProbeAttn;
    return ret;
}

//-----------------------------------------------------
void VoltageConvert()
{
    int ii; //, PageSize = 5;
    double VbitA,VbitB;
    int PAtnA,PAtnB;	

    VbitA=GetVbit(0);
    VbitB=GetVbit(1);
    PAtnA=ProbeAttn[0];
    PAtnB=ProbeAttn[1];


    for (ii = 0; ii < 1024; ii++)
        {
            AnalogVoltageDataA[ii] =
                CalcVoltageFromRawValue(AnalogDataA[ii],
                VbitA,
                PAtnA);
//                GetVbit(0),
//                ProbeAttn[0]);

            AnalogVoltageDataB[ii] =
                CalcVoltageFromRawValue(AnalogDataB[ii],
                VbitB,
                PAtnB);
//                GetVbit(1),
//                ProbeAttn[1]);
        }
}
//-----------------------------------------------------
void ResetFSM()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error RstFSM\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}

//-----------------------------------------------------
void ForceCapture()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error ForceCap\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x08));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x08));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}


//-----------------------------------------------------
void ArmMSO()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error ArmMSO\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x02));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x02));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase));
		HexBuf[BufTransCnt++] = 0x7E;

		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
/*
	vDiv[0];		vDiv[1];
	ProbeAttn[0];	ProbeAttn[1];
	ACDCMode[0];	ACDCMode[1];
	OffsetDbl[0];	OffsetDbl[1];
	LogFam;
	sampleInterval;
	TrigPosition;
	TrigLevel[0];	TrigLevel[1];
	TrigSlope[0];	TrigSlope[1];	TrigSlope[2];
	TrigLAWord;		TrigSPIWd;		TrigI2CWd;
	TrigSPIMode;
	TrigWidth;
	TrigModeDSO;
	TrigChan;

typedef enum {
    TRIG_CHAN_DSO,
	TRIG_CHAN_DSO1,
	TRIG_CHAN_LA,
	TRIG_CHAN_SER_I2C,
	TRIG_CHAN_SER_SPI,
	nMaxTrigChanDefines
} TrigChanDefines ;

TrigChanDefines TrigChan;

typedef enum  {
	TRIG_DSO_EDGE,
	TRIG_DSO_GE,
	TRIG_DSO_LT,
	nMaxTrigModedefines
} TrigModeDefines;
TrigModeDefines TrigModeDSO = TRIG_DSO_EDGE;
			*/


//-----------------------------------------------------
void ConfigureTriggerHardware()
{
    unsigned char MSB, LSB; //, SPIDataSource;
    unsigned short TrigVal[2];
    unsigned char TrigChSel=0;
    int ii;
    unsigned char logTrigWd = 0, logTrigIgn = 0;
    char tmpTrigWord[8]; // = "XXXXXXXX";

	strcpy(tmpTrigWord,TrigLAWord);
    for (ii = 0; ii < 8; ii++)
    {
        if (tmpTrigWord[ii] == '1')
            logTrigWd |= (0x01 << ii);
        if (tmpTrigWord[ii] == 'x' || tmpTrigWord[ii] == 'X')
            logTrigIgn |= (0x01 << ii);
    }
	

	TrigVal[0] = CalcRawValueFromVoltage(
        TrigLevel[TRIG_CHAN_DSO] + OffsetDbl[TRIG_CHAN_DSO], 0);
    TrigVal[1] = CalcRawValueFromVoltage(
        TrigLevel[TRIG_CHAN_DSO1] + OffsetDbl[TRIG_CHAN_DSO1], 1);


    if (TrigChan == TRIG_CHAN_DSO)
        TrigChSel = 0x00;
    else if (TrigChan == TRIG_CHAN_DSO1)
        TrigChSel = 0x01;
    else if (TrigChan == TRIG_CHAN_LA)
        TrigChSel = 0x02;
    else if (TrigChan == TRIG_CHAN_SER_I2C)
        TrigChSel = 0x05;
    else if (TrigChan == TRIG_CHAN_SER_SPI)
        TrigChSel = 0x04;


    // default TRIG_DSO_EDGE
    TrigVal[0] &= 0x9fff;
    TrigVal[1] &= 0x9fff;

    if(TrigModeDSO[0] == 1)//TRIG_DSO_GE
		TrigVal[0] |= 0x4000;
    else if(TrigModeDSO[0] == 2)//TRIG_DSO_LT
		TrigVal[0] |= 0x2000;
    
    
    if(TrigModeDSO[1] == 1)//TRIG_DSO_GE
		TrigVal[1] |= 0x4000;
    else if(TrigModeDSO[1] == 2)//TRIG_DSO_LT
		TrigVal[1] |= 0x2000;
     
    
    
    
	if (TrigSlope[0] == SLOPE_FALLING) TrigVal[0] |= 0x0400;
    else TrigVal[0] &= 0xfbff;

    if (TrigSlope[1] == SLOPE_FALLING) TrigVal[1] |= 0x0400;
    else TrigVal[1] &= 0xfbff;

    TrigVal[0] |= 0x0000;//ChVal load selector
    TrigVal[1] |= 0x0800;//ChVal load selector


	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
    	printf("Serial error TrgHdwr\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);

		//Width first, TrigVal second

		//---------  Ch0 trig settings
		ii = (unsigned char)(TrigWidth[0] / sampleInterval);
		if (ii > 3)
		    ii -= 3;
		else
		    ii = 3;

		HexBuf[BufTransCnt++] = BufTransUpper(0x0b, ii);
		HexBuf[BufTransCnt++] = BufTransLower(0x0b, ii);

		LSB = (unsigned char)(TrigVal[0] & 0x00ff);
		MSB = (unsigned char)((TrigVal[0] & 0xff00) >> 8);
		HexBuf[BufTransCnt++] = BufTransUpper(0x03, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x03, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x04, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x04, MSB);

		//---------  Ch1 trig settings
		ii = (unsigned char)(TrigWidth[1] / sampleInterval);
		if (ii > 3)
		    ii -= 3;
		else
		    ii = 3;

		HexBuf[BufTransCnt++] = BufTransUpper(0x0b, ii);
		HexBuf[BufTransCnt++] = BufTransLower(0x0b, ii);

		LSB = (unsigned char)(TrigVal[1] & 0x00ff);
		MSB = (unsigned char)((TrigVal[1] & 0xff00) >> 8);
		HexBuf[BufTransCnt++] = BufTransUpper(0x03, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x03, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x04, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x04, MSB);



//		if (TrigSlope[TrigChan] == SLOPE_FALLING) TrigChSel |= 0x08;
		if (TrigSlope[2] == SLOPE_FALLING) TrigChSel |= 0x08;
		else TrigChSel &= 0xf7;

		if (ACDCMode[0] != 0x00) //ChA1 AC/DC
		    TrigChSel |= 0x10;
		else
		    TrigChSel &= 0xef;

		if (ACDCMode[1] != 0x00)//ChA2 AC/DC
		    TrigChSel |= 0x20;
		else
		    TrigChSel &= 0xdf;

		if (vDiv[0] > 50) //ChA1 Div1/Div10
		    TrigChSel &= 0xbf;
		else
		    TrigChSel |= 0x40;

		if (vDiv[1] > 50)//ChA2 Div1/Div10
		    TrigChSel &= 0x7f;
		else
		    TrigChSel |= 0x80;

		double a;
		a = vDiv[0];

		HexBuf[BufTransCnt++] = BufTransUpper(0x05, TrigChSel);//park TrigValSelector
		HexBuf[BufTransCnt++] = BufTransLower(0x05, TrigChSel);

		HexBuf[BufTransCnt++] = BufTransUpper(0x0c, logTrigWd);
		HexBuf[BufTransCnt++] = BufTransLower(0x0c, logTrigWd);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0d, logTrigIgn);
		HexBuf[BufTransCnt++] = BufTransLower(0x0d, logTrigIgn);

		HexBuf[BufTransCnt++] = BufTransUpper(0x0f, (0x02 | SlowMode));//Alt Page 2, SerTrig
		HexBuf[BufTransCnt++] = BufTransLower(0x0f, (0x02 | SlowMode));

		int Dex = 0;
		unsigned char SerTrigWd[4];
		unsigned char SerIgnWd[4];
		char chtmp;
		
    if (TrigChan == TRIG_CHAN_SER_SPI){
		for (ii = 0; ii < 4; ii++){
		    SerTrigWd[ii] = 0;
		    SerIgnWd[ii] = 0;
		    for (Dex = 0; Dex < 8; Dex++){
				chtmp = TrigSPIWd[((3-ii)*8)+Dex];
				if (chtmp == '1')
		            SerTrigWd[ii] |= (0x01 << Dex);
				else if((chtmp =='X')||(chtmp == 'x'))
		            SerIgnWd[ii] |= (0x01 << Dex);
			}
		}//SPI Parsing
		
		while(SerIgnWd[0] == 0xff){
			SerTrigWd[0] = SerTrigWd[1];
			SerTrigWd[1] = SerTrigWd[2];
			SerTrigWd[2] = SerTrigWd[3];
			SerTrigWd[3] = 0x00;
			SerIgnWd[0] = SerIgnWd[1];
			SerIgnWd[1] = SerIgnWd[2];
			SerIgnWd[2] = SerIgnWd[3];
			SerIgnWd[3] = 0xff;

		}//SPI trigword position swap
	}//SPI
	else if (TrigChan == TRIG_CHAN_SER_I2C){
		for (ii = 0; ii < 4; ii++){
		    SerTrigWd[ii] = 0;
		    SerIgnWd[ii] = 0;
		    for (Dex = 0; Dex < 8; Dex++){
				chtmp = TrigI2CWd[((3-ii)*8)+Dex];
				if (chtmp == '1')
		            SerTrigWd[ii] |= (0x01 << Dex);
				else if((chtmp =='X')||(chtmp == 'x'))
		            SerIgnWd[ii] |= (0x01 << Dex);
			}
		}//I2C Parsing
		
		while(SerIgnWd[0] == 0xff){
			SerTrigWd[0] = SerTrigWd[1];
			SerTrigWd[1] = SerTrigWd[2];
			SerTrigWd[2] = SerTrigWd[3];
			SerTrigWd[3] = 0x00;
			SerIgnWd[0] = SerIgnWd[1];
			SerIgnWd[1] = SerIgnWd[2];
			SerIgnWd[2] = SerIgnWd[3];
			SerIgnWd[3] = 0xff;

		}//I2C trigword position swap
			

		}//I2C
		

		//Need to add I2C trigwd


		//serial trigger word [17] .. [24]  - must translate from menu
		HexBuf[BufTransCnt++] = BufTransUpper(0x00, SerTrigWd[3]);//SerTrigWd1
		HexBuf[BufTransCnt++] = BufTransLower(0x00, SerTrigWd[3]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x01, SerTrigWd[2]);//
		HexBuf[BufTransCnt++] = BufTransLower(0x01, SerTrigWd[2]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x02, SerTrigWd[1]);
		HexBuf[BufTransCnt++] = BufTransLower(0x02, SerTrigWd[1]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x03, SerTrigWd[0]);
		HexBuf[BufTransCnt++] = BufTransLower(0x03, SerTrigWd[0]);

		//serial trigger word [25] .. [32] ingnore bits X   - must translate from menu
		HexBuf[BufTransCnt++] = BufTransUpper(0x04, SerIgnWd[3]);
		HexBuf[BufTransCnt++] = BufTransLower(0x04, SerIgnWd[3]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x05, SerIgnWd[2]);
		HexBuf[BufTransCnt++] = BufTransLower(0x05, SerIgnWd[2]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x06, SerIgnWd[1]);
		HexBuf[BufTransCnt++] = BufTransLower(0x06, SerIgnWd[1]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x07, SerIgnWd[0]);//SerTrigIgn1
		HexBuf[BufTransCnt++] = BufTransLower(0x07, SerIgnWd[0]);

		HexBuf[BufTransCnt++] = BufTransUpper(0x08, TrigSPIMode);//bit 0,1 = SPI mode
		HexBuf[BufTransCnt++] = BufTransLower(0x08, TrigSPIMode);

		HexBuf[BufTransCnt++] = BufTransUpper(0x0f, (0x00 | SlowMode));
		HexBuf[BufTransCnt++] = BufTransLower(0x0f, (0x00 | SlowMode));

		//To turn on probe cal signal set (0x0f, 0x10)

		HexBuf[BufTransCnt++] = 0x7E;

		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
/*
		SerTrigWdX[0] = SerTrigWd[0];
		SerTrigWdX[1] = SerIgnWd[0];
		SerTrigWdX[2] = SerTrigWd[1];
		SerTrigWdX[3] = SerIgnWd[1];
		SerTrigWdX[4] = SerTrigWd[2];
		SerTrigWdX[5] = SerIgnWd[2];
		SerTrigWdX[6] = SerTrigWd[3];
		SerTrigWdX[7] = SerIgnWd[3];
		TrigChSelX = TrigChSel;
		WriteTest();		
*/
		
	}//fd_w
}
//-----------------------------------------------------
void ConfigureTriggerPosition()
{
    unsigned short LocalTrigPos;
	unsigned char MSB, LSB;

	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error TrigPos\n");
	}
	else
	{
        if (TrigPosition >= 0)
        {
            LocalTrigPos = TrigPosition;
            LocalTrigPos &= 0X7fff; //turn off holdoff counter
        }
        else
        {
            LocalTrigPos = TrigPosition * -1;
            LocalTrigPos |= 0X8000; //turn on holdoff counter
        }
        if (sampleInterval == 5) LocalTrigPos += 11;//6
        else if (sampleInterval == 10) LocalTrigPos += 9;//6
        else if (sampleInterval == 20) LocalTrigPos += 9;
        else LocalTrigPos += 3;

        LSB = LocalTrigPos & 0x00ff;
        MSB = (LocalTrigPos & 0xff00) >> 8;

        portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x07, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x07, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x08, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x08, MSB);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}//-----------------------------------------------------

void ConfigureHardwareMSO()
{
//    unsigned char MSB = 0, LSB = 0;
//    Boolean TimerStat = false;
//    if (MSO1800.frmDSO.timer1.Enabled)
//    {
//        TimerStat = false;
//        MSO1800.frmDSO.timer1.Stop();
//    }
//        byte tmpGoStat = vars.GoStat;
        CalcRateMSBLSB(sampleInterval);
        ClkRate_Out(ClkRateMSB, ClkRateLSB);

        OffsetDacVal[0] = CalcOffsetRawValueFromVoltage(OffsetDbl[0], 0);
        OffsetDacVal[1] = CalcOffsetRawValueFromVoltage(OffsetDbl[1], 1);

        DAC_Out_SPI();
		ConfigureThresholdLevel_SPI();

        ConfigureTriggerHardware();
		ConfigureTriggerPosition();
        //SerialWrite(SerialP1, timer1, RateBuf + DACBuf1 + DACBuf2 + TrigBuf + TrigPosBuf, vars.DSOD[UnitNum].HardwareStatus);
    
//    vars.DSOD[UnitNum].HardwareNeedsConfiguringMSO = false;
//    vars.DSOD[UnitNum].HardwareNeedsConfiguringDAC = false;

//    if (TimerStat)
//    {
//        MSO1800.frmDSO.timer1.Start();
//    }

}



//-----------------------------------------------------
//turns MSO off, and put it in low power stby mode
void LEDOff()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error LED off\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase &= 0xef;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}

//-----------------------------------------------------
//selects between SPI config Mem and threshold ADC
unsigned char SPI_DevSel(unsigned char Dev)
{
	unsigned char SPItmp = 0x00;
    unsigned char j;

	if (Dev == 0x02) j = SPItmp | 0x20;//SPI Mem
    else if (Dev == 0x01) j = SPItmp | 0x04;//SPI ADC
    else j = SPItmp;

	return j;

}
//-----------------------------------------------------
//SPI clock H
void SPI_CH(unsigned char Dev)
{
    unsigned char j;

	DSOCtlBase = 0x10;
	BufTransCnt = 0;
	BufTransCnt = SerBufInit(BufTransCnt);
	j = SPI_DevSel(Dev);
    j ^= 0x01;
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C CLK high
    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
	HexBuf[BufTransCnt++] = 0x7E;
	write(fd_w,HexBuf,BufTransCnt);  
}
//-----------------------------------------------------
//SPI clock L
void SPI_CL(unsigned char Dev)
{
    unsigned char j;

	DSOCtlBase = 0x10;
	BufTransCnt = 0;
	BufTransCnt = SerBufInit(BufTransCnt);
	j = SPI_DevSel(Dev);
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C CLK high
    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
	HexBuf[BufTransCnt++] = 0x7E;
	write(fd_w,HexBuf,BufTransCnt);  
}
//-----------------------------------------------------
//SPI transfer Start
void SPI_Start(unsigned char Dev)
{
    unsigned char j;

	DSOCtlBase = 0x10;
	BufTransCnt = 0;
	BufTransCnt = SerBufInit(BufTransCnt);
	j = SPI_DevSel(Dev);
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, 0x00);//Pre start CS high
    HexBuf[BufTransCnt++] = BufTransLower(0x06, 0x00);
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C CS low
    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
	HexBuf[BufTransCnt++] = 0x7E;
	write(fd_w,HexBuf,BufTransCnt);  
}
//-----------------------------------------------------
//SPI transfer stop
void SPI_Stop(unsigned char Dev)
{
    unsigned char j;

	DSOCtlBase = 0x10;
	BufTransCnt = 0;
	BufTransCnt = SerBufInit(BufTransCnt);
	j = SPI_DevSel(Dev);
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C CS low
    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, 0x00);//Pre start CS high
    HexBuf[BufTransCnt++] = BufTransLower(0x06, 0x00);
	HexBuf[BufTransCnt++] = 0x7E;
	write(fd_w,HexBuf,BufTransCnt);  
}
//-----------------------------------------------------
//SPI data out
void SPI_Mem_Out(unsigned char val)
{
	unsigned char SPItmp = 0x20;
    unsigned char i,j;

	DSOCtlBase = 0x10;
	BufTransCnt = 0;
	BufTransCnt = SerBufInit(BufTransCnt);
	j = SPItmp;
    for (i = 0; i <= 7; i++)
    {
        if ((val & (0x80 >> i)) != 0x00) j |= 0x02;
        else j &= 0xfd;
        HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
        HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
        j ^= 0x01;
		HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//I2C Ctl
        HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
        j ^= 0x01;
    }
    j &= 0xfd;
    HexBuf[BufTransCnt++] = BufTransUpper(0x06, j);//Data L, Clk L
    HexBuf[BufTransCnt++] = BufTransLower(0x06, j);
    HexBuf[BufTransCnt++] = 0x7E;
	write(fd_w,HexBuf,BufTransCnt);  
}

//-----------------------------------------------------
/*//SPI Write
void SPI_Write(unsigned char Dev, unsigned char Addr, unsigned char Data)
{
    unsigned char mode;

    SPI_Start(Dev);
    mode = 0x06; //WREN
    SPI_Mem_Out(mode);
    SPI_Stop(Dev);

    SPI_Start(Dev);
    mode = 0x02; //Write
    SPI_Mem_Out(mode);
    SPI_Mem_Out(Addr);
    SPI_Mem_Out(Data);
    SPI_Stop(Dev);
}
*/
//-----------------------------------------------------
//SPI read byte 
unsigned char SPI_Read_Gut82(unsigned char Dev)
{
    unsigned char val1, ret = 0x00;
	
	SPI_CH(Dev);

	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x80;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x40;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x20;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x10;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x08;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x04;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x02;
    SPI_CL(Dev);

    SPI_CH(Dev);
	val1 = CheckTriggerStatusRaw() & 0X40;
    if (val1 != 0x00) ret |= 0x01;
    SPI_CL(Dev);

    return ret;

}
//-----------------------------------------------------
//SPI read 16 byte page
void SPI_Read_Buf_Page(unsigned char Dev, unsigned char *Buf, unsigned char Page)
{
    unsigned char mode;
    unsigned char Addr, Cnt;//Data
    Cnt = 0;

	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error SPI read\n");
	}
	else
	{
		portconfig(&fd_w,1);
		SPI_Start(Dev);
		mode = 0x03; //Read
		SPI_Mem_Out(mode);
		Addr = Page * 16;
		SPI_Mem_Out(Addr);
		for (Cnt = 0; Cnt <= 15; Cnt++)
		{
		    Buf[Addr + Cnt] = SPI_Read_Gut82(Dev);
		}
		SPI_Stop(Dev);
		close(fd_w);
	}
}

//-----------------------------------------------------
//Parse SPI EEProm data into MSO calibration parameters
void ParseEprom(unsigned char *Buf)
{
    int Chan, BufPos;
	unsigned short tmp;

	msoType = Buf[0];
	modelNumber = Buf[1];
	serialNumber = ((Buf[3] << 8) + Buf[2]);
    BufPos = 4;
    for (Chan = 0; Chan < 2; Chan++)
    {
        tmp = (Buf[BufPos + 1] << 8) + Buf[BufPos];
        vbitDiv10[Chan] = (double)tmp / 10000.0;
        BufPos += 2;
    }

    for (Chan = 0; Chan < 2; Chan++)
    {
        tmp = (Buf[BufPos + 1] << 8) + Buf[BufPos];
        OffsetVBitDiv10[Chan] = (double)tmp / 10000.0;
        BufPos += 2;
    }

    for (Chan = 0; Chan < 2; Chan++)
    {
        tmp = (Buf[BufPos + 1] << 8) + Buf[BufPos];
        OffsetCenterValDiv10[Chan] = tmp;
        BufPos += 2;
    }
    for (Chan = 0; Chan < 2; Chan++)
    {
        tmp = (Buf[BufPos + 1] << 8) + Buf[BufPos];
        vbit[Chan] = (double)tmp / 10000.0;
        BufPos += 2;
    }

    for (Chan = 0; Chan < 2; Chan++)
    {
        tmp = (Buf[BufPos + 1] << 8) + Buf[BufPos];
        OffsetVBit[Chan] = (double)tmp / 10000.0;
        BufPos += 2;
    }

    for (Chan = 0; Chan < 2; Chan++)
    {
        tmp = (Buf[BufPos + 1] << 8) + Buf[BufPos];
        OffsetCenterVal[Chan] = tmp;
        BufPos += 2;
    }
	

}
//-----------------------------------------------------
//Screen dump of MSO calibration parameters
void PrintMsoParam()
{
	int i,j;

	for(j=0;j<2;j++){
		for(i=0;i<16;i++) printf("%d ",FBuf[(j*16)+i]);
		printf("\n");
	}
		
	printf("MSO Type= %d\n",msoType);
	printf("Model Number= %d\n",modelNumber);
	printf("SN= %d\n",serialNumber);
	printf("\n");
	for(i=0;i<2;i++){
		printf("vbit[%d]=%f\n",i,vbit[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetVBit[%d]=%f\n",i,OffsetVBit[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetCenterVal[%d]=0x%x\n",i,OffsetCenterVal[i]);
	}
	printf("\n");
	for(i=0;i<2;i++){
		printf("vbitDiv10[%d]=%f\n",i,vbitDiv10[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetVBitDiv10[%d]=%f\n",i,OffsetVBitDiv10[i]);
	}
	for(i=0;i<2;i++){
		printf("OffsetCenterValDiv10[%d]=0x%x\n",i,OffsetCenterValDiv10[i]);
	}
	printf("\n");
}
//-----------------------------------------------------
//write msocfg.bin file
void WriteMsoParam()
{
	FILE *fp;
	/* open the file */
	fp = fopen("msocfg.bin", "wb");
	if(fp)
	{
		fwrite(MBuf,sizeof(MBuf[0]),32,fp);
		fclose(fp);
	}
}

//-----------------------------------------------------
//Read msocfg.bin file 
void ReadMsoParam()
{
	FILE *fp;
	/* open the file */
	fp = fopen("msocfg.bin", "rb");
	if(fp)
	{
		fread(FBuf,sizeof(FBuf[0]),32,fp);
		fclose(fp);
	}
}
//-----------------------------------------------------
//Read msodata.csv file 
void ReadMsoData()
{
	FILE *fp;
	/* open the file */
	fp = fopen("msocfg.bin", "rb");
	if(fp)
	{
		fread(FBuf,sizeof(FBuf[0]),32,fp);
		fclose(fp);
	}
}
//--------------------------------
void send_error(char *error_text)
{
    printf("Content-type: text/plain\r\n");
    printf("\r\n");
    printf("Woops:- %s\r\n", error_text);
}
//-----------------------------------------------------
/* this routine borrowed from the examples that come with the NCSA server */
char x2c(char *what)
{
    register char digit;

    digit = (what[0] >= 'A' ? ((what[0] & 0xdf) - 'A')+10 : (what[0] - '0'));
    digit *= 16;
    digit += (what[1] >= 'A' ? ((what[1] & 0xdf) - 'A')+10 : (what[1] - '0'));
    return(digit);
}


//--------------------------------

/* this routine borrowed from the examples that come with the NCSA server */
void unescape_url(char *url)
{
    int x,y;

    for (x=0,y=0; url[y]; ++x,++y) 
	{
        if ((url[x] = url[y]) == '%')
		{
            url[x] = x2c(&url[y+1]);
            y += 2;
        }
    }
    url[x] = '\0';
}
//-----------------------------------------------------

/* Assumes name_val_pairs array is currently full of NULL characters */
void load_nv_pair(char *tmp_buffer, int nv_entry)
{
    int chars_processed = 0;
    char *src_char_ptr;
    char *dest_char_ptr;

    /* get the part before the '=' sign */
    src_char_ptr = tmp_buffer;
    dest_char_ptr = name_val_pairs[nv_entry].name;
    while(*src_char_ptr && *src_char_ptr != '=' &&
          chars_processed < FIELD_LEN)
	{
        /* Change a '+' to a ' ' */
        if (*src_char_ptr == '+')
            *dest_char_ptr = ' ';
        else
            *dest_char_ptr = *src_char_ptr;
        dest_char_ptr++;
        src_char_ptr++;
        chars_processed++;
    }

    /* skip the '=' character */
    if (*src_char_ptr == '=')
	{
        /* get the part after the '=' sign */
        src_char_ptr++;
        dest_char_ptr = name_val_pairs[nv_entry].value;
        chars_processed = 0;
        while(*src_char_ptr && *src_char_ptr != '=' &&
               chars_processed < FIELD_LEN)
		{
            /* Change a '+' to a ' ' */
            if (*src_char_ptr == '+')
                *dest_char_ptr = ' ';
            else
                *dest_char_ptr = *src_char_ptr;
            dest_char_ptr++;
            src_char_ptr++;
            chars_processed++;
        }
    }

    /* Now need to decode %XX characters from the two fields */
    unescape_url(name_val_pairs[nv_entry].name);
    unescape_url(name_val_pairs[nv_entry].value);
}
//-----------------------------------------------------
int Get_input(void)
{
    int nv_entry_number = 0;
    int got_data = 0;
    char *ip_data = 0;
    int ip_length = 0;
    char tmp_buffer[(FIELD_LEN * 2) + 2];
    int tmp_offset = 0;
    char *tmp_char_ptr;
    int chars_processed = 0;

    tmp_char_ptr = getenv("REQUEST_METHOD");
    if (tmp_char_ptr)
	{
        if (strcmp(tmp_char_ptr, "POST") == 0)
		{
            tmp_char_ptr = getenv("CONTENT_LENGTH");
            if (tmp_char_ptr)
			{
                ip_length = atoi(tmp_char_ptr);
                ip_data = malloc(ip_length + 1); /* allow for NULL character */
                if (fread(ip_data, 1, ip_length, stdin) != ip_length)
				{
                    send_error("Bad read from stdin");
                    return(0);
                }
                ip_data[ip_length] = '\0';
                got_data = 1;        
            }
        }
    }

    tmp_char_ptr = getenv("REQUEST_METHOD");
    if (tmp_char_ptr)
	{
        if (strcmp(getenv("REQUEST_METHOD"), "GET") == 0)
		{
            tmp_char_ptr = getenv("QUERY_STRING");
            if (tmp_char_ptr) 
			{
                ip_length = strlen(tmp_char_ptr);
                ip_data = malloc(ip_length + 1); /* allow for NULL character */
                strcpy(ip_data, getenv("QUERY_STRING"));
                ip_data[ip_length] = '\0';
                got_data = 1;
            }
        }
    }

    if (!got_data) 
	{
        send_error("No data received");
        return(0);
    }

    if (ip_length <= 0)
	{
        send_error("Input length not > 0");
        return(0);
    }
	
//	printf("QUERY_STRING 2= ");
//	printf("%s\n",tmp_char_ptr);

    memset(name_val_pairs, '\0', sizeof(name_val_pairs));
    tmp_char_ptr = ip_data;
    while (chars_processed <= ip_length && nv_entry_number < NV_PAIRS)
	{
        /* copy a single name=value pair to a tmp buffer */
        tmp_offset = 0;
        while (*tmp_char_ptr && *tmp_char_ptr != '&' &&
               tmp_offset < FIELD_LEN)
		{
            tmp_buffer[tmp_offset] = *tmp_char_ptr;
            tmp_offset++;
            tmp_char_ptr++;
            chars_processed++;
        }
        tmp_buffer[tmp_offset] = '\0';

        /* decode and load the pair */
        load_nv_pair(tmp_buffer, nv_entry_number);

        /* move on to the next name=value pair */
        tmp_char_ptr++;
        nv_entry_number++;
    }
//	printf("NV= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);
	

	return(1);
}



//------------------------------------------------
void InitSettings()
{
	vDiv[0] = 500;			vDiv[1] = 500;
	//vDiv
	ProbeAttn[0] = 10;  	ProbeAttn[1] = 10;
	//ProbeAttn
	ACDCMode[0] = 0;		ACDCMode[1] = 0;
	//ACDCMode
	OffsetDbl[0] = 0;	   OffsetDbl[1] = 0;
	//OffsetDbl
	LogFam = 5;
	//LogFam
	sampleInterval = 10;
	//sampleInterval
	TrigLevel[0] = 0;	   	TrigLevel[1] = 0;
	//TrigLevel
//	printf("nv3.1= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);

	TrigSlope[0] = SLOPE_RISING;
	TrigSlope[1] = SLOPE_RISING;
	TrigSlope[2] = SLOPE_RISING;
	//TrigSlope	
	TrigLAWord[0] = 0;
	strcat(TrigLAWord,"XXXXXXXX");
//	printf("nv3.2= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);
	//TrigLAWord
	TrigPosition = 500;
	//TrigPosition
	TrigSPIWd[0] = 0;
	strcat(TrigSPIWd,"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
	//TrigSPIWd
	TrigI2CWd[0] = 0;
	strcat(TrigI2CWd,"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
//	printf("nv3.3= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);

	//TrigI2CWd
	TrigSPIMode = 0;
	//TrigSPIMode
	TrigWidth[0] = 0;
	TrigWidth[1] = 0;
	//TrigWidth
//	TrigModeDSO[0] = TRIG_DSO_EDGE;
//	TrigModeDSO[1] = TRIG_DSO_EDGE;
	TrigModeDSO[0] = 0;
	TrigModeDSO[1] = 0;
	//TrigModeDSO
	TrigChan = TRIG_CHAN_DSO;
	//TrigChan
}
//----------------------------------
//write settings.bin file
void WriteMsoSettings()
{
	FILE *fp;
	/* open the file */
	fp = fopen("msoset.txt", "wb");
	if(fp)
	{
		fprintf(fp,"vDiv0,%d\n",vDiv[0]);
		fprintf(fp,"vDiv1,%d\n",vDiv[1]);
		fprintf(fp,"ProbeAttn0,%d\n",ProbeAttn[0]);
		fprintf(fp,"ProbeAttn1,%d\n",ProbeAttn[1]);
		fprintf(fp,"ACDCMode0,%d\n",ACDCMode[0]);
		fprintf(fp,"ACDCMode1,%d\n",ACDCMode[1]);
		fprintf(fp,"OffsetDbl0,%d\n",OffsetDbl[0]);
		fprintf(fp,"OffsetDbl1,%d\n",OffsetDbl[1]);
		fprintf(fp,"LogFam,%d\n",LogFam);
		fprintf(fp,"sampleInterval,%d\n",sampleInterval);
		fprintf(fp,"TrigLevel0,%d\n",TrigLevel[0]);
		fprintf(fp,"TrigLevel1,%d\n",TrigLevel[1]);
		fprintf(fp,"TrigSlope0,%d\n",TrigSlope[0]);
		fprintf(fp,"TrigSlope1,%d\n",TrigSlope[1]);
		fprintf(fp,"TrigLAWord,%s\n",TrigLAWord);
		fprintf(fp,"TrigLASlope,%d\n",TrigSlope[2]);
		fprintf(fp,"TrigPosition,%d\n",TrigPosition);
		fprintf(fp,"TrigSPIWd,%s\n",TrigSPIWd);
		fprintf(fp,"TrigI2CWd,%s\n",TrigI2CWd);
		fprintf(fp,"TrigSPIMode,%d\n",TrigSPIMode);
		fprintf(fp,"TrigWidth0,%d\n",TrigWidth[0]);
		fprintf(fp,"TrigWidth1,%d\n",TrigWidth[1]);
		fprintf(fp,"TrigModeDSO0,%d\n",TrigModeDSO[0]);
		fprintf(fp,"TrigModeDSO1,%d\n",TrigModeDSO[1]);
		fprintf(fp,"TrigChan,%d\n",TrigChan);
		fclose(fp);
	}
}
//-----------------------------------
/*void WriteTest()
{
	FILE *fp;
	// open the file 
	fp = fopen("testout.txt", "wb");
	if(fp)
	{
		fprintf(fp,"SerTrigWdX, %X %X %X %X %X %X %X %X\n ",SerTrigWdX[0],SerTrigWdX[1],SerTrigWdX[2],SerTrigWdX[3],SerTrigWdX[4],SerTrigWdX[5],SerTrigWdX[6],SerTrigWdX[7]);
		fprintf(fp,"TrigSPIMode, %X\n",TrigSPIMode);
		fprintf(fp,"TrigChSelX, %X\n",TrigChSelX);
		fprintf(fp,"SlowMode, %X\n",SlowMode);
		
		fclose(fp);
	}
}*/
//-----------------------------------
/*void ReadMsoSettings()
{
	char fBuf[512],	vBuf[512],pBuf[512];
	FILE *fp;
	int ii,jj;

	// open the file 
	fp = fopen("msoset.txt", "rb");
	if(fp){
		fBuf[0]=0;
		while(fscanf(fp,"%s",&fBuf[0])>0){
			pBuf[0]=0;
			vBuf[0]=0;
			ii = 0;
			jj = 0;
			while(fBuf[ii]!=','){
				pBuf[ii]=fBuf[ii];
				ii++;
			}
			pBuf[ii]=0;
			ii++;
			while(fBuf[ii]!=0){
				vBuf[jj]=fBuf[ii];
				ii++;
				jj++;
			}
			vBuf[jj]=0;
			if(strcmp(pBuf,"TrigI2CWd")==0) {
				TrigI2CWd[0]=0;
				strncat(TrigI2CWd,vBuf,32);
			}
			if(strcmp(pBuf,"TrigSPIWd")==0) {
				TrigSPIWd[0]=0; 
				strncat(TrigSPIWd,vBuf,32);
			}
			if(strcmp(pBuf,"vDiv0")==0) vDiv[0]=atoi(vBuf);
			if(strcmp(pBuf,"vDiv1")==0) vDiv[1]=atoi(vBuf);
			if(strcmp(pBuf,"ProbeAttn0")==0) ProbeAttn[0]=atoi(vBuf);
			if(strcmp(pBuf,"ProbeAttn1")==0) ProbeAttn[1]=atoi(vBuf);
			if(strcmp(pBuf,"ACDCMode0")==0) ACDCMode[0]=atoi(vBuf);
			if(strcmp(pBuf,"ACDCMode1")==0) ACDCMode[1]=atoi(vBuf);
			if(strcmp(pBuf,"OffsetDbl0")==0) OffsetDbl[0]=atoi(vBuf);
			if(strcmp(pBuf,"OffsetDbl1")==0) OffsetDbl[1]=atoi(vBuf);
			if(strcmp(pBuf,"LogFam")==0) LogFam=atoi(vBuf);
			if(strcmp(pBuf,"sampleInterval")==0) sampleInterval=atoi(vBuf);
			if(strcmp(pBuf,"TrigLevel0")==0) TrigLevel[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigLevel1")==0) TrigLevel[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigSlope0")==0) TrigSlope[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigSlope1")==0) TrigSlope[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigLAWord")==0) {TrigLAWord[0]=0; strncat(TrigLAWord,vBuf,8);}
			if(strcmp(pBuf,"TrigLASlope")==0) TrigSlope[2]=atoi(vBuf);
			if(strcmp(pBuf,"TrigPosition")==0) TrigPosition=atoi(vBuf);
			if(strcmp(pBuf,"TrigSPIMode")==0) TrigSPIMode=atoi(vBuf);
			if(strcmp(pBuf,"TrigWidth0")==0) TrigWidth[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigWidth1")==0) TrigWidth[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigModeDSO0")==0) TrigModeDSO[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigModeDSO1")==0) TrigModeDSO[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigChan")==0) TrigChan=atoi(vBuf);
			fBuf[0]=0;
//			fscanf(fp,"%s",&fBuf);
		}
		fclose(fp);
	}
}*/
//-----------------------------------
void ReadMsoSettings2()
{
//	char fBuf[512],	vBuf[512],pBuf[512];
	char *fBuf;
	char vBuf[512],pBuf[512],tBuf[512];
	FILE *fp;
	int ii,jj;
//	fBuf=malloc(512*sizeof(char));
	int n,i;

	/* open the file */
//	printf("test1\n");
	fp = fopen("msoset.txt", "rb");
	if(fp){
//		fBuf[0]=0;
		i=0;
		fBuf = fgets(tBuf,512,fp);
		while(fBuf){
			n = strlen(fBuf);
//			printf("%d, len = %d\n",i,n);
	//		printf("%s",fBuf);
		//			while(fscanf(fp,"%s",&fBuf[0])>0){

			pBuf[0]=0;
			vBuf[0]=0;
			ii = 0;
			jj = 0;
			while(fBuf[ii]!=','){
				pBuf[ii]=fBuf[ii];
				ii++;
			}
			pBuf[ii]=0;
			ii++;
			while(fBuf[ii]!=0){
				vBuf[jj]=fBuf[ii];
				ii++;
				jj++;
			}
			vBuf[jj]=0;
			if(strcmp(pBuf,"TrigI2CWd")==0) {
				TrigI2CWd[0]=0;
				strncat(TrigI2CWd,vBuf,32);
			}
			if(strcmp(pBuf,"TrigSPIWd")==0) {
				TrigSPIWd[0]=0; 
				strncat(TrigSPIWd,vBuf,32);
			}
			if(strcmp(pBuf,"vDiv0")==0) vDiv[0]=atoi(vBuf);
			if(strcmp(pBuf,"vDiv1")==0) vDiv[1]=atoi(vBuf);
			if(strcmp(pBuf,"ProbeAttn0")==0) ProbeAttn[0]=atoi(vBuf);
			if(strcmp(pBuf,"ProbeAttn1")==0) ProbeAttn[1]=atoi(vBuf);
			if(strcmp(pBuf,"ACDCMode0")==0) ACDCMode[0]=atoi(vBuf);
			if(strcmp(pBuf,"ACDCMode1")==0) ACDCMode[1]=atoi(vBuf);
			if(strcmp(pBuf,"OffsetDbl0")==0) OffsetDbl[0]=atoi(vBuf);
			if(strcmp(pBuf,"OffsetDbl1")==0) OffsetDbl[1]=atoi(vBuf);
			if(strcmp(pBuf,"LogFam")==0) LogFam=atoi(vBuf);
			if(strcmp(pBuf,"sampleInterval")==0) sampleInterval=atoi(vBuf);
			if(strcmp(pBuf,"TrigLevel0")==0) TrigLevel[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigLevel1")==0) TrigLevel[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigSlope0")==0) TrigSlope[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigSlope1")==0) TrigSlope[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigLAWord")==0) {TrigLAWord[0]=0; strncat(TrigLAWord,vBuf,8);}
			if(strcmp(pBuf,"TrigLASlope")==0) TrigSlope[2]=atoi(vBuf);
			if(strcmp(pBuf,"TrigPosition")==0) TrigPosition=atoi(vBuf);
			if(strcmp(pBuf,"TrigSPIMode")==0) TrigSPIMode=atoi(vBuf);
			if(strcmp(pBuf,"TrigWidth0")==0) TrigWidth[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigWidth1")==0) TrigWidth[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigModeDSO0")==0) TrigModeDSO[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigModeDSO1")==0) TrigModeDSO[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigChan")==0) TrigChan=atoi(vBuf);
			fBuf[0]=0;
//			fscanf(fp,"%s",&fBuf);
		//}
			fBuf = fgets(tBuf,512,fp);
			i++;
		}


		fclose(fp);
	}//fp
}
//-----------------------------------
void ReadQueryString()
{
	int nv_entry_number = 0;
	unsigned char i;
	unsigned char s;
	unsigned char wd=0;
//	int ret;
//	FILE *fp;
	char sBuf[512];
	long sidTmp;

//	clock_t start,end,end2;

//	int Cnt;

//	printf("NV= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);

	while(name_val_pairs[nv_entry_number].name[0] != '\0')
	{
		if(strcmp(name_val_pairs[nv_entry_number].name,"sid")==0)
			{
				sid= atol(name_val_pairs[nv_entry_number].value);
			/*	printf("%d\n\n",sid);
				if(wd){
					fp = fopen("msodata.csv", "ab");
					if(fp)
					{
						fprintf(fp,"sid = %d\n",sid);
						fclose(fp);
						ret = 1;
					}
					else ret = 0;
					sprintf(sBuf,"gzip -f msodata.csv\n");
					system(sBuf);
					

				}*/

			}

			
//		printf("Name=%s, Value=%s\n\n",
//               name_val_pairs[nv_entry_number].name,
//               name_val_pairs[nv_entry_number].value);
		if(strcmp(name_val_pairs[nv_entry_number].name,"VDIV0")==0){
			vDiv[0] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"VDIV1")==0){
			vDiv[1] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//vDiv
		if(strcmp(name_val_pairs[nv_entry_number].name,"PATTN0")==0){
			ProbeAttn[0] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}

		if(strcmp(name_val_pairs[nv_entry_number].name,"PATTN1")==0){
			ProbeAttn[1] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}

		//ProbeAttn
		if(strcmp(name_val_pairs[nv_entry_number].name,"ACDC0")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"DC")==0)
				ACDCMode[0] = 1;
			else
				ACDCMode[0] = 0;
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"ACDC1")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"DC")==0)
				ACDCMode[1] = 1;
			else
				ACDCMode[1] = 0;
			SetChanged++;
		}
		//ACDCMode
		if(strcmp(name_val_pairs[nv_entry_number].name,"VOFF0")==0){
			OffsetDbl[0] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"VOFF1")==0){
			OffsetDbl[1] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//OffsetDbl
		if(strcmp(name_val_pairs[nv_entry_number].name,"LAFM")==0){
			LogFam = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//LogFam
		if(strcmp(name_val_pairs[nv_entry_number].name,"TSAMP")==0){
			sampleInterval = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//sampleInterval
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGV0")==0){
			TrigLevel[0] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGV1")==0){
			TrigLevel[1] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigLevel
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRSLP0")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"R")==0)
				TrigSlope[0] = SLOPE_RISING;
			else
				TrigSlope[0] = SLOPE_FALLING;
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRSLP1")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"R")==0)
				TrigSlope[1] = SLOPE_RISING;
			else
				TrigSlope[1] = SLOPE_FALLING;
			SetChanged++;
		}
		//TrigSlope	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRLAWD")==0){
			strncpy(TrigLAWord,name_val_pairs[nv_entry_number].value,8);
			SetChanged++;
		}
		//TrigLAWord	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRLASLP")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"T")==0)
				TrigSlope[2] = SLOPE_RISING;
			else
				TrigSlope[2] = SLOPE_FALLING;
			SetChanged++;
		}
		//Logic trig slope
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRPOS")==0){
//			TrigPosition = atof(name_val_pairs[nv_entry_number].value);
			TrigPosition = atoi(name_val_pairs[nv_entry_number].value);
//			TrigPosition = 555;
			SetChanged++;
		}
		//TrigPosition

		if(strcmp(name_val_pairs[nv_entry_number].name,"TRSPIWD")==0){
			strncpy(TrigWdtmp,name_val_pairs[nv_entry_number].value,32);
			strncpy(TrigSPIWd,TrigWdtmp,8);
			strncpy(&TrigSPIWd[8],&TrigWdtmp[8],8);
			strncpy(&TrigSPIWd[16],&TrigWdtmp[16],8);
			strncpy(&TrigSPIWd[24],&TrigWdtmp[24],8);
			TrigSPIWd[32]=0;
			SetChanged++;
		}
		//TrigSPIWd	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRI2CWD")==0){
			strncpy(TrigWdtmp,name_val_pairs[nv_entry_number].value,32);
			strncpy(TrigI2CWd,TrigWdtmp,8);
			strncpy(&TrigI2CWd[8],&TrigWdtmp[8],8);
			strncpy(&TrigI2CWd[16],&TrigWdtmp[16],8);
			strncpy(&TrigI2CWd[24],&TrigWdtmp[24],8);
			TrigI2CWd[32]=0;
			SetChanged++;
		}
		//TrigI2CWd	


		if(strcmp(name_val_pairs[nv_entry_number].name,"SPIMODE")==0){
			TrigSPIMode = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigSPIMode
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGWD0")==0){
			TrigWidth[0] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGWD1")==0){
			TrigWidth[1] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigWidth
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRMODE0")==0)
		{
			TrigModeDSO[0] = atoi(name_val_pairs[nv_entry_number].value);

//			if(strcmp(name_val_pairs[nv_entry_number].value,"GE")==0)
//				TrigModeDSO[0] = TRIG_DSO_GE;
//			else if(strcmp(name_val_pairs[nv_entry_number].value,"LT")==0)
//				TrigModeDSO[0] = TRIG_DSO_LT;
//			else
//				TrigModeDSO[0] = TRIG_DSO_EDGE;//ED
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRMODE1")==0)
		{
			TrigModeDSO[1] = atoi(name_val_pairs[nv_entry_number].value);
//			if(strcmp(name_val_pairs[nv_entry_number].value,"GE")==0)
//				TrigModeDSO[1] = TRIG_DSO_GE;
//			else if(strcmp(name_val_pairs[nv_entry_number].value,"LT")==0)
//				TrigModeDSO[1] = TRIG_DSO_LT;
//			else
//				TrigModeDSO[1] = TRIG_DSO_EDGE;//ED
			SetChanged++;
		}
		//TrigModeDSO	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRGCH")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"A0")==0)
				TrigChan = TRIG_CHAN_DSO;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"A1")==0)
				TrigChan = TRIG_CHAN_DSO1;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"LA")==0)
				TrigChan = TRIG_CHAN_LA;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"I2C")==0)
				TrigChan = TRIG_CHAN_SER_I2C;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"SPI")==0)
				TrigChan = TRIG_CHAN_SER_SPI;
			else
				TrigChan = TRIG_CHAN_DSO;
			SetChanged++;
		}
		//TrigChan
		if(strcmp(name_val_pairs[nv_entry_number].name,"w")==0)
		{
			s = atoi(name_val_pairs[nv_entry_number].value);
			usleep(s*10000);
		}
		//wait

		if(strcmp(name_val_pairs[nv_entry_number].name,"i")==0)
		{
//			j=sscanf(Parms[i+1],"%c",&report);
			GetSerPort28();
			if(strcmp(name_val_pairs[nv_entry_number].value,"I")==0)
			{
				ResetADC();
				SPI_Read_Buf_Page(2, &MBuf[0], 0);
				SPI_Read_Buf_Page(2, &MBuf[0], 1);
				WriteMsoParam();
				LEDOff();
				MsoBusy=0;
//				for(i=0;i<32;i++){
//					printf("%x ",MBuf[i]);
//				}
//				printf("\n");	

				printf("P_Rdy\n%d\n",sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"C")==0
			        ||strcmp(name_val_pairs[nv_entry_number].value,"c")==0)
			{
				ResetADC();
				ReadMsoParam();
				ParseEprom(&FBuf[0]);
				sprintf(sBuf,"rm tmp/msodata*\n");
				system(sBuf);
				MsoBusy=0;

				if(strcmp(name_val_pairs[nv_entry_number].value,"c")==0){
					PrintMsoParam();
					printf("%d\n",sid);
				}
				else
					printf("Connected\n%d\n",sid);	

			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"a")==0)
			{
				if(!MsoBusy){
					//				ResetFSM();
					PrevSid=CurrSid;
					MsoBusy=1;
					sidTmp = RotateSid(PrevSid);
					if(sidTmp!=0){
						sprintf(sBuf,"rm tmp/msodata%d.csv\n",sidTmp);
						system(sBuf);
					}

					ArmMSO();
					CurrSid=sid;
	//				printf("Armed\n\n");	
					printf("%d\n%d\n",28,CurrSid);
				}
				else printf("%d\n%d\n",36,CurrSid);

			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"A")==0)
			{
				if(!MsoBusy){
					PrevSid=CurrSid;
					MsoBusy=1;
					sidTmp = RotateSid(PrevSid);
					if(sidTmp!=0){
						sprintf(sBuf,"rm tmp/msodata%d.csv\n",sidTmp);
						system(sBuf);
					}
					if(SetChanged){
						WriteMsoSettings();//write to msoset.txt
						SetChanged = 0;
					}
					ReadMsoParam();
					ParseEprom(&FBuf[0]);
					InitSettings();
					ReadMsoSettings2();//read back from msoset.txt
					ConfigureHardwareMSO();
					CurrSid=sid;
					printf("%d\n%d\n",27,CurrSid);	
					ArmMSO();
	//				printf("Setup & Armed\n\n");	
				}
				else printf("%d\n%d\n",37,CurrSid);	

			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"M")==0)
			{
				ReadMsoParam();
				ParseEprom(&FBuf[0]);
				InitSettings();
				ReadMsoSettings2();//read back from msoset.txt
				ConfigureHardwareMSO();
//				printf("MSO Setup\n\n");	
				printf("%d\n%d\n",26,sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"F")==0)
			{
				ResetFSM();
//				printf("ResetFSM\n\n");	
				MsoBusy=0;
				printf("%d\n%d\n",30,sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"X")==0)
			{
				ForceCapture();
//				printf("ForceCap\n\n");	
				MsoBusy=0;
				printf("%d\n%d\n",30,sid);	
			}

			else if(strcmp(name_val_pairs[nv_entry_number].value,"Q")==0)
			{
				LEDOff();
//				printf("Off\n\n");	
	
				printf("Bye\n%d\n",sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"p")==0)
			{
				ReadMsoParam();
				ParseEprom(&FBuf[0]);
				PrintMsoParam();
				InitSettings();
				ReadMsoSettings2();//read back from msoset.txt
				PrintMSOSettings();	
				printf("%d\n",sid);
			}
			else if((strcmp(name_val_pairs[nv_entry_number].value,"T")==0)||
			        (strcmp(name_val_pairs[nv_entry_number].value,"t")==0))
			{
//start = clock();
				i = CheckTriggerStatus();
				i&=0x1f;
//				printf("TrigStat = %x\n\n",i);	
				if(strcmp(name_val_pairs[nv_entry_number].value,"T")==0)
					printf("%d\n%d\n",i,sid);
				else{
					if(i==22){
						if(sid==CurrSid){
//						ReadMsoParam();
//						ParseEprom(&FBuf[0]);
//						ReadBuffer2();
						ReadBuffer();
						VoltageConvert();
//						while(!WriteMsoData());

//						end2 = clock();
						WriteMsoData();
//printf("fileWrite time was: %f\n", (end2 - end) / (double)CLOCKS_PER_SEC);
//end = clock();
//printf("BeadBuf time was: %f\n", (end - start) / (double)CLOCKS_PER_SEC);
						wd =1;
						MsoBusy=0;
						printf("%d\n%d\n",29,CurrSid);
						}
						else printf("%d\n%d\n",35,CurrSid);
					}//22 waiting for Readback
					else printf("%d\n%d\n",i,sid);
				}//"t"

			} //"t or T"
			else if((strcmp(name_val_pairs[nv_entry_number].value,"B")==0)||
				(strcmp(name_val_pairs[nv_entry_number].value,"b")==0))
			{
				ReadMsoParam();
				ParseEprom(&FBuf[0]);
				ReadBuffer();
//				printf("ReadBuf = %x\n\n",HexBuf[0]);
//		        for (Cnt = 0; Cnt < (4096 / 4); Cnt++)//x4
//					printf("%X %X %X\n",AnalogDataA[Cnt],AnalogDataB[Cnt],LogicData[Cnt]); 

//				printf("ReadBuf\n\n");	

				if(strcmp(name_val_pairs[nv_entry_number].value,"b")==0){
					for(i=0;i<2;i++) printf("vbit[%d]=%f\n",i,vbit[i]);
					for(i=0;i<2;i++) printf("OffsetVBit[%d]=%f\n",i,OffsetVBit[i]);
					for(i=0;i<2;i++) printf("OffsetCenterVal[%d]=0x%x\n",i,OffsetCenterVal[i]);
					printf("\n");
					for(i=0;i<2;i++) printf("vbitDiv10[%d]=%f\n",i,vbitDiv10[i]);
					for(i=0;i<2;i++) printf("OffsetVBitDiv10[%d]=%f\n",i,OffsetVBitDiv10[i]);
					for(i=0;i<2;i++) printf("OffsetCenterValDiv10[%d]=0x%x\n",i,OffsetCenterValDiv10[i]);
				}

				VoltageConvert();

//		        for (Cnt = 0; Cnt < ((4096 / 4)-1); Cnt++)//x4
//					printf("%f %f\n",AnalogVoltageDataA[Cnt],AnalogVoltageDataB[Cnt]); 

				WriteMsoData();
				MsoBusy=0;
//				printf("%d\n%d\n",29,sid);	

			}
		} 
	   nv_entry_number++;
	}

}
//-----------------------------------

int WriteMsoData()
{
	FILE *fp;
	int Cnt;
	int ret;
	char sBuf[512];
	
	/* open the file */
	sprintf(sBuf,"tmp/msodata%d.csv",CurrSid);
	fp = fopen(sBuf, "wb");
//	fp = fopen("msodata.csv", "wb");
	if(fp)
	{

        for (Cnt = 0; Cnt < 1023; Cnt++)
			fprintf(fp,"%f\t%f\t%d\n",AnalogVoltageDataA[Cnt],AnalogVoltageDataB[Cnt],LogicData[Cnt]); 
		
//		fprintf(fp,"sid = %d\n",sid);
		fclose(fp);
		ret = 1;
	}
	else ret = 0;

//	sprintf(sBuf,"cp msodata.csv msodata%d.csv\n",CurrSid);
//	system(sBuf);

//	sprintf(sBuf,"gzip -f msodata.csv\n");
//	system(sBuf);
	printf("%d\n%d\n",29,CurrSid);



	return ret;

}
//-----------------------------------
static long RotateSid(long sidT)
{
	long Ret;
	Ret = sidE;
	sidE=sidD;
	sidD=sidC;
	sidC=sidB;
	sidB=sidA;
	sidA=sidT;
	return Ret;
}
//-----------------------------------
	
void PrintMSOSettings()
{
	printf("vDiv0 = %d, vDiv1 = %d\n",vDiv[0], vDiv[1]);
	printf("ProbeAttn0 = %d, ProbeAttn1 = %d\n",ProbeAttn[0], ProbeAttn[1]);
	printf("ACDCMode0 = %d, ACDCMode1 = %d\n",ACDCMode[0], ACDCMode[1]);
	printf("OffsetDbl0 = %d, OffsetDbl1 = %d\n",OffsetDbl[0], OffsetDbl[1]);
	printf("LogFam = %d\n",LogFam);
	printf("sampleInterval = %d\n",sampleInterval);
	printf("TrigLevel0 = %d, TrigLevel1 = %d\n",TrigLevel[0], TrigLevel[1]);
	printf("TrigSlope0 = %d, TrigSlope1 = %d\n",TrigSlope[0], TrigSlope[1]);
	printf("TrigLAWord = %s\n",TrigLAWord);
	printf("TrigLASlope = %d\n",TrigSlope[2]);
	printf("TrigPosition = %d\n",TrigPosition);
	printf("TrigSPIWd = %s\n",TrigSPIWd);
	printf("TrigI2CWd = %s\n",TrigI2CWd);
	printf("TrigSPIMode = %d\n",TrigSPIMode);
	printf("TrigWidth0 = %d, TrigWidth1 = %d\n",TrigWidth[0], TrigWidth[1]);
	printf("TrigModeDSO0 = %d, TrigModeDSO1 = %d\n",TrigModeDSO[0],TrigModeDSO[1]);
	printf("TrigChan = %d\n",TrigChan);

}
//===================================================
/*	char *pEnvPtr;

	pEnvPtr= getenv ("REQUEST_METHOD"); 
	printf ("REQUEST_METHOD= "); 
	   if (!pEnvPtr) 
		  printf ("<NULL-POINTER>\n"); 
	   else 
		  printf ("%s\n", pEnvPtr); 

	pEnvPtr= getenv ("QUERY_STRING"); 
	printf ("QUERY_STRING= "); 
	   if (!pEnvPtr) 
		  printf ("<NULL-POINTER>\n"); 
	   else 
		  printf ("%s\n", pEnvPtr); 
*/
//===================================================
int main(int Parm_Count, char* Parms[])
{
	line = (char *) malloc(18);
	InitSettings(); 
	ReadMsoSettings2();//read back from msoset.txt

		while (FCGI_Accept() >= 0) {
//		char *contentLength = getenv("CONTENT_LENGTH");
//		int len;


		if (!Get_input())
		{
        		exit(EXIT_FAILURE);
		}
			printf("Content-Type: text/plain\n");
			printf("Cache-Control: no-cache,no-store\n");
			printf("Status: 200 OK\n\n");


//			printf("Information decoded was:\r\n\r\n");
			SetChanged = 0;
	//		InitSettings();
		//	printf("nv3= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);
	//		ReadMsoSettings();//read back from msoset.txt
		//	printf("nv4= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);
			ReadQueryString();
		//	printf("\n\nEnviron\n");
		//	PrintMSOSettings();

			if(SetChanged)	WriteMsoSettings();//write to msoset.txt


			printf("\n\n");
   } // while 
   
	
//	printf("\r\n");
//    exit(EXIT_SUCCESS);
	free(line);
    return 0;
}

