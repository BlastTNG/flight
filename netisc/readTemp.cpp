#include "readTemp.h"    

// Parameters that are loaded from the configuration file

//const char __tempInput_Name[]="temp.cfg";
//char __tempOutput_Name[80];

int __tempControl = 1;               // thermometer that controls heater
unsigned int __tempsleeptime = 100;
float __tempSetLimit = 3.0;
float __tempOffset = 0.10;
float __temppressuregain;
float __temppressureoffset;

float __tempgain = 103.3;           // gain is not 100 because of filtering

int __tempUDStat = 0;
int __tempULStat = 0;
    
int __tempHeaterStat = 0;

// Variables for digital out to control heater. 
// Meant to work with Measurement Computing USB-1608FS
int __tempBoardNum = 0;
int __tempPortNum = AUXPORT;
int __tempDataValue = 0;

// set stuff up
// return 1 for success, 0 for failure

int tempSetup( int in_tempControl, unsigned int in_tempsleeptime, 
	       float in_tempSetLimit, float in_tempOffset, 
	       float in_temppressuregain, float in_temppressureoffset ) {
  __tempControl = in_tempControl;
  __tempsleeptime = in_tempsleeptime;
  __tempSetLimit = in_tempSetLimit;
  __tempOffset = in_tempOffset;
  __temppressuregain = in_temppressuregain;
  __temppressureoffset = in_temppressureoffset;
  
  
  /*
    FILE *tempconfig;
    char linein[80];
    
    // Try reading settings file
    if((tempconfig = fopen(__tempInput_Name,"r")) ==NULL) {
    printf("No settings file!"); 
    return 0;
    }
    
    fgets(linein,80,tempconfig); sscanf(linein,"%s",&__tempOutput_Name[0]);    
    fgets(linein,80,tempconfig); sscanf(linein,"%i",&__tempControl);
    fgets(linein,80,tempconfig); sscanf(linein,"%i",&__tempsleeptime);
    fgets(linein,80,tempconfig); sscanf(linein,"%f",&__tempSetLimit); 
    fgets(linein,80,tempconfig); sscanf(linein,"%f",&__tempOffset);
    fgets(linein,80,tempconfig); sscanf(linein,"%f",&__temppressuregain);
    fgets(linein,80,tempconfig); sscanf(linein,"%f",&__temppressureoffset);
    __tempSetLimit=__tempSetLimit/100.;
    __tempOffset=__tempOffset/100.;
    
    fclose(tempconfig);
  */
  
  // Declare UL Revision Level
  float RevLevel = (float)CURRENTREVNUM;
  __tempUDStat = cbDeclareRevision(&RevLevel);
  
  /* Initiate error handling
     Parameters:
     PRINTALL :all warnings and errors encountered will be printed
     DONTPRINT: suppresses error messages so that card will continue when
     error clears.
     DONTSTOP :program will continue even if error occurs.
     Note that STOPALL and STOPFATAL are only effective in 
     Windows applications, not Console applications. 
  */
  cbErrHandling (DONTPRINT, DONTSTOP);

  // Configure the port for output
  __tempULStat = cbDConfigPort (__tempBoardNum, __tempPortNum, DIGITALOUT);
  
  // Turn off relay that controls heater for camera*/
  __tempULStat = cbDOut(__tempBoardNum, __tempPortNum, __tempDataValue);
  
  return 1;
}


// Read / log sensors, set relay on the heater as necessary
// display contains the readout (should already have space allocated!)

void tempDoStuff( double *temp1, double *temp2, double *temp3, double *temp4, double *pressure, int *heaterOn ) { 
  time_t curTime=0;
  int i;
  int Chan=1;
  int Gain = BIP10VOLTS;
  int LowChannel = 0;
  int HighChannel = 4;
  WORD DataValue[5];
  float EngUnits[5];
  //FILE *tempsout;
  
  // Open file for writing
  //tempsout=fopen(__tempOutput_Name,"a");
  
  // Put a sleep here to avoid reading the sensors too frequently
  Sleep(__tempsleeptime);
  
  /* collect the sample from the channels */
  /*Parameters:
    __tempBoardNum    :number used by CB.CFG to describe this board
    Chan        :input channel number
    Gain        :gain for the board in __tempBoardNum
    DataValue   :value collected from Chan */
  
  Chan = LowChannel-1;
  while (Chan++ < HighChannel) {
    __tempUDStat = cbAIn (__tempBoardNum, Chan, Gain, &DataValue[Chan]);
    __tempUDStat = cbToEngUnits (__tempBoardNum, Gain, DataValue[Chan], 
				 &EngUnits[Chan]);
  }
  
  if(EngUnits[__tempControl]*__tempgain < __tempSetLimit) {

    /* Value to write to digital port to turn on relay */
    __tempDataValue = 64;
    
    printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&& heater on: %i, %lf < %lf\n",
	   __tempControl,EngUnits[__tempControl]*__tempgain, __tempSetLimit);

    // set the heater flag
    //*heaterOn = 1;
    __tempHeaterStat = 1;
  } else if (EngUnits[__tempControl]*__tempgain > 
	     (__tempSetLimit+__tempOffset)) {

    /* Turn off relay...if neither condition is met, do nothing */
    __tempDataValue = 0;
    
    printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&& heater off: %i, %lf > %lf\n",
	   __tempControl,EngUnits[__tempControl]*__tempgain, 
	   __tempSetLimit+__tempOffset);
    //*heaterOn = 0;
    __tempHeaterStat = 0;
  }
        
  *heaterOn = __tempHeaterStat;
  __tempULStat = cbDOut(__tempBoardNum, __tempPortNum, __tempDataValue);
  
  i=-1;
  curTime=time(NULL);    /* Get timestamp*/                
    //float thegain, theoffset;
    
    //while (i++ < HighChannel)     /* Write data*/
    //{
    //    if( i < 4 ) 
    //    {
    //        thegain = 100.;
    //        theoffset = 0.;
    //    } 
    //    else
    //    {
    //        thegain = __temppressuregain;
    //        theoffset = __temppressureoffset;
    //    }
    
    //fprintf(tempsout,"%d %6u %.1f ",i, DataValue[i],EngUnits[i]*thegain + 
    // theoffset);
    //}
    
    *temp1 = EngUnits[0]*__tempgain;
    *temp2 = EngUnits[1]*__tempgain;
    *temp3 = EngUnits[2]*__tempgain;
    *temp4 = EngUnits[3]*__tempgain;
    *pressure = EngUnits[4]*__temppressuregain + __temppressureoffset;
    
    //fprintf(tempsout,"%f ",__tempAEngUnits);
    //fprintf(tempsout,asctime(gmtime(&curTime)));  /*write timestamp*/
    
    /* Close files */
    //fclose(tempsout);
}

// Turn off the heaters on shutdown
void tempShutdown( void ) {
  /* Turn off relay */
  
  __tempDataValue = 0;
    
  printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&& heater off\n");
    
  __tempULStat = cbDOut(__tempBoardNum, __tempPortNum, __tempDataValue);
}
