/*
 * PiDX.cpp
 * Raspberry Pi based SSB digital transceiver controller
 *---------------------------------------------------------------------
 * This program operates as a controller for a Raspberry Pi to control
 * a Pixie transceiver hardware.
 * Project at http://www.github.com/lu7did/PiDX
 *---------------------------------------------------------------------
 *
 * Created by Dr. Pedro E. Colla (lu7did@gmail.com)
 * Code excerpts from several packages:
 *    Adafruit's python code for CharLCDPlate
 *    tune.cpp from rpitx package by Evariste Courjaud F5OEO
 *    sendiq.cpp from rpitx package (also) by Evariste Coujaud (F5EOE)
 *    wiringPi library (git clone git://git.drogon.net/wiringPi)
 *    iambic-keyer (https://github.com/n1gp/iambic-keyer)
 *    log.c logging facility by  rxi https://github.com/rxi/log.c
 *    minIni configuration management package by Compuphase https://github.com/compuphase/minIni/tree/master/dev
 * Also libraries
 *    librpitx by  Evariste Courjaud (F5EOE)
 *    TinyALSA sound card library
* Ideas
 *    Hans Summers (G0UPL) QDX transceiver User Manual
 *    Adam Rong (BD6CR), D4D transceiver
 * ---------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

/* Firmware configuration */
#define DEBUG 0


/*--- Standard libraries ---*/

#include "stdio.h"
#include <unistd.h>
#include <cstring>
#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <cstdlib> // for std::rand() and std::srand()
#include <ctime>   // for std::time()
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <time.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

/*--- Specific libraries ---*/

#include "/home/pi/librpitx/src/librpitx.h"
#include "/home/pi/librpitx/src/librpitx.h"

/*--- General purpose definitions ---*/

#define PTT_ON 1
#define PTT_OFF 0
typedef unsigned char byte;
typedef bool boolean;

#define RUN 0b00000001

//==*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                         FIRMWARE CONFIGURATION
//==*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#define GPIO04 4
#define GPIO12 12

//==*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                         FIRMWARE IDENTIFICATION
//==*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
const char   *PROGRAMID="PiDX";
const char   *PROG_VERSION="1.0";
const char   *PROG_BUILD="00";
const char   *COPYRIGHT="(c) LU7DID,LU7DZ 2019,2020";

//--------------------------------------------------------------------------------------------------
// PiDX
//--------------------------------------------------------------------------------------------------

byte MSW=0;
int  TVOX=0;

iqdmasync *iqtest=NULL;

//--------------------------[System Word Handler]---------------------------------------------------
// getSSW Return status according with the setting of the argument bit onto the SW
//--------------------------------------------------------------------------------------------------
bool getWord (unsigned char SysWord, unsigned char v) {

  return SysWord & v;

}
//--------------------------------------------------------------------------------------------------
// setWord Sets a given bit of a status word 
//--------------------------------------------------------------------------------------------------
void setWord(unsigned char* SysWord,unsigned char v, bool val) {

  *SysWord = ~v & *SysWord;
  if (val == true) {
    *SysWord = *SysWord | v;
  }

}



//--------------------------------------------------------------------------------------------------
// timer_exec 
// timer management controller
//--------------------------------------------------------------------------------------------------
void timer_exec()
{

  if (TVOX!=0) {
     TVOX--;
     if(TVOX==0) {
       //iqtest->stop();
       //delete(iqtest);
       //iqtest=NULL;
       //setWord(&MSW,VOX,false);
       printf("VOX turned off\n");
     }
  }
}

//---------------------------------------------------------------------------
// Timer handler function
//---------------------------------------------------------------------------
void timer_start(std::function<void(void)> func, unsigned int interval)
{
  std::thread([func, interval]()
  {
    while (getWord(MSW,RUN)==true)
    {
      auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
      func();
      std::this_thread::sleep_until(x);
    }
  }).detach();
}


//---------------------------------------------------------------------------------
// Print usage
//---------------------------------------------------------------------------------
void print_usage(void)
{
fprintf(stderr,"%s %s [%s]\n\
Usage: [-i File Input][-s Samplerate][-l] [-f Frequency] [-h Harmonic number] \n\
-i            path to File Input \n\
-s            SampleRate 10000-250000 \n\
-f float      central frequency Hz(50 kHz to 1500 MHz),\n\
-l            loop mode for file input\n\
-h            Use harmonic number n\n\
-t            IQ type (i16 default) {i16,u8,float,double}\n\
-?            help (this help).\n\
\n",PROGRAMID,PROG_VERSION,PROG_BUILD);


} /* end function print_usage */

#ifdef Pi4D


void SimpleTestFileIQ(uint64_t Freq)
{
	
}

//---------------------------------------------------------------------------
// Capture and manage SIG
//---------------------------------------------------------------------------
static void terminate(int num)
{
    setWord(&MSW,RUN,false);
    fprintf(stderr,"Caught signal - Terminating %x\n",num);

}
#endif

//----------------------------------------------------------------------------
// main execution
//----------------------------------------------------------------------------
int main(int argc, char* argv[])
{

/*--- Define master timer ---*/

        timer_start(timer_exec,100);
	int a;
	int anyargs = 1;

/*--- Process arguments ---*/

	while(1)
	{
		a = getopt(argc, argv, "h:");
	
		if(a == -1) 
		{
			if(anyargs) break;
			else a='h'; //print usage and exit
		}
		anyargs = 1;	

		switch(a)
		{
		case 'i': // File name
			//FileName = optarg;
			break;
		case 'f': // Frequency
			//SetFrequency = atof(optarg);
			break;
		case 's': // SampleRate (Only needeed in IQ mode)
			//SampleRate = atoi(optarg);

#ifdef Pi4D
			if(SampleRate>MAX_SAMPLERATE) 
			{
				for(int i=2;i<12;i++) //Max 10 times samplerate
				{
					if(SampleRate/i<MAX_SAMPLERATE) 
					{
						SampleRate=SampleRate/i;
						Decimation=i;
						break;
					}
				}
				if(Decimation==1)
				{
					 fprintf(stderr,"SampleRate too high : >%d sample/s",10*MAX_SAMPLERATE);
					 exit(1);
				}
				else
				{
					fprintf(stderr,"Warning samplerate too high, decimation by %d will be performed",Decimation);
				}
			};
			break;
#endif 

		case 'h': // help
			//Harmonic=atoi(optarg);
			break;
		case 'l': // loop mode
			//loop_mode_flag = true;
			break;
		case 't': // inout type
			//if(strcmp(optarg,"i16")==0) InputType=typeiq_i16;
			//if(strcmp(optarg,"u8")==0) InputType=typeiq_u8;
			//if(strcmp(optarg,"float")==0) InputType=typeiq_float;
			//if(strcmp(optarg,"double")==0) InputType=typeiq_double;
			break;
		case -1:
        	break;
		case '?':
			print_usage();
			exit(1);
			break;
		default:
			print_usage();
			exit(1);
			break;
		}/* end switch a */
	}/* end while getopt() */





#ifdef Pi4D

	float SetFrequency=7074000;
	float SampleRate=12000;
	char* FileName=NULL;
	int Harmonic=1;
	enum {typeiq_i16,typeiq_u8,typeiq_float,typeiq_double};
	int InputType=typeiq_i16;
	int Decimation=1;


	if(FileName==NULL) {fprintf(stderr,"Need an input\n");exit(1);}

	for (int i = 0; i < 64; i++) {
           struct sigaction sa;
           std::memset(&sa, 0, sizeof(sa));
           sa.sa_handler = terminate;
           sigaction(i, &sa, NULL);
        }

        setWord(&MSW,RUN,true);
        setWord(&MSW,VOX,false);

//----------- Initialize GPIO interface

        //if(gpioInitialise()<0) {
        //  printf("Cannot initialize GPIO\n");
        //  return -1;
        //}

        //gpioSetPullUpDown(GPIO12,PI_PUD_UP);
        //usleep(100000);

#endif

        printf("%s %s [%s]\n",PROGRAMID,PROG_VERSION,PROG_BUILD);

#ifdef Pi4D

	FILE *iqfile=NULL;
	if(strcmp(FileName,"-")==0)
		iqfile=fopen("/dev/stdin","rb");
	else
		iqfile=fopen(FileName	,"rb");
	if (iqfile==NULL) 
	{
	   printf("input file issue\n");
	   exit(0);
	}

	int SR=12000;
	int FifoSize=IQBURST*4;

	//iqdmasync iqtest(SetFrequency,SampleRate,14,FifoSize,MODE_IQ);
	//iqtest.print_clock_tree();
	//iqtest.SetPLLMasterLoop(5,6,0);

	std::complex<float> CIQBuffer[IQBURST];	

	while(getWord(MSW,RUN)==true)
	{
	   int CplxSampleNumber=0;
           switch(InputType)
	   {
//*---------------------------------------------------------------------------------------------------------
//* Float Queue
//*---------------------------------------------------------------------------------------------------------
	   case typeiq_float:
	   {
		static float IQBuffer[IQBURST*2];
		int nbread=fread(IQBuffer,sizeof(float),IQBURST*2,iqfile);
                float s=0.0;
		if(nbread>0)
		{
           	   for(int i=0;i<nbread/2;i++)
		   {
		     if(i%Decimation==0) {
		       if (getWord(MSW,VOX)==true) {
 		           CIQBuffer[CplxSampleNumber++]=std::complex<float>(IQBuffer[i*2],IQBuffer[i*2+1]);
		       } else {
			   CIQBuffer[CplxSampleNumber++]=std::complex<float>(1.0,1.0);
	               }
 	  	       float Ai=(IQBuffer[i*2]*IQBuffer[i*2])+(IQBuffer[i*2+1]*IQBuffer[i*2+1]);
		       Ai=log10(Ai);
		       Ai=10.0*Ai;
		       s=s+Ai;
		     }
		   }
   		   s=s/(1.0*nbread/2);
		   if (s>=-25.0) {
            	      if(getWord(MSW,VOX)==false) {
		         printf("Vox Activated Avg(%f)\n",s);
     			 iqtest=new iqdmasync(SetFrequency,SampleRate,14,FifoSize,MODE_IQ);
      	 		 iqtest->SetPLLMasterLoop(5,6,0);
                         usleep(10000);

		      }
		      Tvox=15;
                      //gpioWrite(GPIO12,PTT_ON);
		      setWord(&MSW,VOX,true);
	   	   }
		} else 	{
		   printf("End of file\n");
                   setWord(&MSW,VOX,false);
		   setWord(&MSW,RUN,false);
		}
	        }
		break;	
	      }
   	      if (getWord(MSW,VOX)==true) {
                  iqtest->SetIQSamples(CIQBuffer,CplxSampleNumber,Harmonic);
              }
	}

        if (getWord(MSW,VOX)==false) {
           gpioWrite(GPIO12,PTT_OFF);
        }

        iqtest->stop();
        delete(iqtest);
        iqtest=NULL;

#endif

}

