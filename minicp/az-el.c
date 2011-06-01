/*******************************************************************

az-el.c -- mcplib code to drive the Spider test cryostat az-el mount 
           to do scans for beam mapping. Controls the az and el Cool 
           Muscle stepper motors. 
           
Author:    Jamil A. Shariff
           BallAst Group, Univeristy of Toronto

Updated:   May 26, 2011

Note:      CM = Cool Muscle (stepper motor type)
           CML = Cool Muscle Language (ASCII-based, to command motors)
********************************************************************/

#include <stdio.h>	
#include <stdlib.h>
#include <sys/time.h>	
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <unistd.h>	
#include <fcntl.h>	
#include <termios.h>	
#include <errno.h>

#include "blast.h"	  
#include "command_struct.h"
#include "tx.h"

//#define CM_DEBUG        // uncomment to see debug info
#define COMMAND_SIZE 15   // largest expected command string size 
#define MAX_CHARS 1000    // max number of response bytes

#define AZ_DEVICE "/dev/ttyUSB0"
#define EL_DEVICE "/dev/ttyUSB1"

#define AZ_GEAR_RATIO 222 
#define CM_PULSES 50000   // Cool Muscle pulses per rotation
#define SPEED_UNIT 10     // pulses/s in one speed unit
#define ACCEL_UNIT 1000   // pulses/s^2 in one accel unit
#define IN_TO_MM 25.4
#define ROT_PER_INCH 5    // lin. actuator rotations per inch of travel
#define EL_GEAR_RATIO 70  
#define A 1029.68         // distance from cryo axis to lin. act. axis (mm)
#define B 350.0           // length of rocker arm in mm
#define D 740.79          // actuator length in mm (fully retracted)
#define C 117.65          // angle in degrees relevant to geometry of system
                          // = 180 - 37 - 25.35
#define PI 3.14159265
#define CNTS_PER_DEG 72000.0/360.0
#define CM_PER_ENC 50000.0/72000.0
#define EL_MIN -10.0
#define EL_MAX 89.0

/* CMInfoStruct: contains info on state of serial comms  with a Cool Muscle
   (also axis encoder reference position)  */

struct CMInfoStruct {

  int fd;                 // file descriptor
  int open;               // 0 = closed, 1 = open
  int init;               // 0 = uninitialized, 1 = initialized
  int closing;            // 1 if in the process of closing
//int reset;              // 1 to reset serial connection
//unsigned int err_count; // tally of serial comm. errors 
  int ref;                // position of encoder (counts) after an az_el_set 
                          // command

  char motorstr[3];       // name of motor

} azinfo, elinfo;

static pthread_t azelcomm_id;     // gets assigned a thread ID

void nameThread(const char*);   // mcplib.c

void startAzEl();
void endAzEl();
void WriteAzEl();

static void* azelComm(void* arg); // serial thread routine

static void open_cm(char *dev_name, struct CMInfoStruct *cminfo);
static void close_cm(struct CMInfoStruct *cminfo);
static void checkpos_cm(struct CMInfoStruct *cminfo);
static void init_cm(struct CMInfoStruct *cminfo);
static void slew_cm(int accel, int speed, int position, struct CMInfoStruct 
                    *cminfo);

static void AzElScan();
static void allstop_cm(); 
static void goto_cm();
static void raster_cm(); 

static int drive_cm(int accel, int speed, int position, struct CMInfoStruct 
                    *cminfo);
static int read_cm(struct CMInfoStruct *cminfo, int read_flag);
static int write_cm(struct CMInfoStruct *cminfo, char *cmd, int length, 
                    const char *cmd_desc); 

static double calc_dx(double theta, double dtheta);
static double xoftheta(double theta); 
static double dxdtheta(double theta);

/* start/end serial threads */

void startAzEl()
{
  pthread_create(&azelcomm_id, NULL, (void*)&azelComm, NULL);
}

void endAzEl()
{
 
  int i = 0;
  azinfo.closing = 1;
  elinfo.closing = 1; 
  
  while(azinfo.open == 1 && elinfo.open == 1 && i++ < 100) usleep(10000);

}

/* drive_cm moves a Cool Muscle stepper motor by the specified distance 
   at the specified speed and acceleration. */

int drive_cm(int accel, int speed, int distance, struct CMInfoStruct* cminfo)
{

  int acount=0, scount=0, dcount=0;
  
  char *execframe = "[1\r";              // the command to execute the CML
                                         // program bank for motion

  char accelframe[COMMAND_SIZE];         // string for acceleration value
  char speedframe[COMMAND_SIZE];         // string for speed value
  char distframe[COMMAND_SIZE];          // string for position value

  char *program = "B1\rA1,S1,P1+\rEND\r";// small CML program bank to do 
                                         // relative move

  /* read acceleration, speed and distance values into strings */
  
  acount = sprintf(accelframe, "A1=%i\r", accel);
  scount = sprintf(speedframe, "S1=%i\r", speed);
  dcount = sprintf(distframe, "P1=%i\r", distance);

  #ifdef CM_DEBUG
  bprintf(info, "%s motor commands:\n", cminfo->motorstr);
  bprintf(info,"The acceleration string written was: %s", accelframe);
  bprintf(info,"The acceleration character count was: %i", acount);
  bprintf(info,"The speed string written was: %", speedframe);
  bprintf(info,"The speed character count was: %i", scount);
  bprintf(info,"The position string written was: %s", posframe);
  bprintf(info,"The position character count was: %i", dcount);
  #endif

  /* write motion commands */

  #ifdef CM_DEBUG
  bprintf(info,"Writing acceleration CML command to %s motor.", 
          cminfo->motorstr);
  #endif

  if ((write_cm(cminfo, accelframe, acount, " acceleration CML command")) < 0) {
    return -1;
  }

  #ifdef CM_DEBUG
  bprintf(info,"Writing speed CML command to %s motor.", cminfo->motorstr);
  #endif

  if ((write_cm(cminfo, speedframe, scount, " speed CML command")) < 0) {
    return -2;
  }

  #ifdef CM_DEBUG
  bprintf(info,"Writing position CML command to %s motor.", cminfo->motorstr);
  #endif

  if ( (write_cm(cminfo, distframe, dcount, " distance CML command")) < 0 ) {
    return -3;
  }

  #ifdef CM_DEBUG
  bprintf(info,"Writing CML motion program to %s motor.", cminfo->motorstr);
  #endif

  if ( (write_cm(cminfo, program, strlen(program), " CML program")) < 0 ) {
    return -4;
  }

  #ifdef CM_DEBUG
  bprintf(info,"Writing excecute motion CML command to %s motor.", 
          cminfo->motorstr);
  #endif

  if ( (write_cm(cminfo, execframe, strlen(execframe), 
        " execute motion CML command")) < 0 ) {
    return -5;
  }

  return 0;
}

/* slew_cm uses drive_cm to move a Cool Muscle to a specified position
   and does error checking */

void slew_cm(int accel, int speed, int position, struct CMInfoStruct *cminfo)
{
  int motion_commanded = 0;
  int response;

  while (!motion_commanded) {

    response = drive_cm(accel, speed, position, cminfo);

    if (response == 0){
      motion_commanded = 1;
    } else {
      bprintf(info, "Commanding motion of %s motor failed: retrying...", 
              cminfo->motorstr);
      sleep(1);
    }
    /* TODO - sjb: flight code should restart connection after too many 
                   failures*/
  }
}

/* goto_cm goes to a specific (az,el) position by slewing in each axis */

void goto_cm()
{  

  int az_accel, az_speed, az_dest, el_accel, el_speed, el_dest;

  static struct BiPhaseStruct* elEncAddr;
  static struct BiPhaseStruct* azEncAddr;

  static int firsttime = 1;

  double az_enc;      // current az encoder reading (cnts)
  double az_enc_dest; // az encoder reading at destination angle
  double el_enc;      // current el encoder reading (cnts)
  double el;          // current elevation
  double dext;        // change in lin. act. extension from current
                      // angle to destination angle
  double dextdtheta;  // derivative of lin. act. extension w.r.t. elevation

  if (firsttime) {
    elEncAddr = GetBiPhaseAddr("adc1_enc_el");
    azEncAddr = GetBiPhaseAddr("adc1_enc_az");
    firsttime = 0;
  }

  /* azimuth calibrations */

  az_enc = ReadData(azEncAddr);

  az_enc_dest = (CommandData.az_el.az - CommandData.az_el.az_ref)*CNTS_PER_DEG
                 + azinfo.ref;

  az_dest = (int)((az_enc_dest - az_enc)*AZ_GEAR_RATIO*CM_PER_ENC);
		  
  az_speed = ((int)(CommandData.az_el.az_speed)*AZ_GEAR_RATIO*CM_PULSES)
                    /(360*SPEED_UNIT);

  az_accel = ((int)(CommandData.az_el.az_accel)*AZ_GEAR_RATIO*CM_PULSES)
                    /(360*ACCEL_UNIT);

  /* elevation calibrations */

  el_enc = ReadData(elEncAddr);

  el = (el_enc - elinfo.ref)/CNTS_PER_DEG + CommandData.az_el.el_ref;

  dext = calc_dx(el, (CommandData.az_el.el - el));

  dextdtheta = dxdtheta(el); // not ideal to use current angle for something 
                             // that is a continuous func. of theta

  el_dest =  (int)(((dext)/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES);

  el_speed = (int)(((dextdtheta*CommandData.az_el.el_speed/IN_TO_MM)
                     *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/SPEED_UNIT);

  el_accel = (int)(((dextdtheta*CommandData.az_el.el_accel/IN_TO_MM)
	*ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/ACCEL_UNIT);

  bprintf(info, "el_dest = %i", el_dest);
  bprintf(info, "el_speed = %i", el_speed);
  bprintf(info, "el_accel = %i", el_accel);

  slew_cm(az_accel, az_speed, az_dest, &azinfo);
  slew_cm(el_accel, el_speed, el_dest, &elinfo);
  if (az_speed != 0) 
    checkpos_cm(&azinfo);
 
  if (el_speed !=0)
    checkpos_cm(&elinfo);
}

/* raster_cm performs a raster scan with the commanded parameters using a 
   sequence of slew_cm calls */

void raster_cm() 
{ 

  int az_width, az_speed, az_accel, el_start, el_speed, el_accel, 
      step_size; 

  int step_count;           // keep track of el steps                
  int pos_arg;              // argument to pass as az position value
  int N_steps;              // number of elevation steps

  double el_enc, el, ext_low, ext, dext, dextdtheta, el_high, el_low;

  static struct BiPhaseStruct* elEncAddr;
  static struct BiPhaseStruct* azEncAddr;

  static int firsttime = 1;

  if (firsttime) {
    elEncAddr = GetBiPhaseAddr("adc1_enc_el");
    azEncAddr = GetBiPhaseAddr("adc1_enc_az");
    firsttime = 0;
  }

  /* azimuth calibrations */

  az_width = ((int)(CommandData.az_el.az_width)*AZ_GEAR_RATIO*CM_PULSES)
                    /360;

  az_speed = ((int)(CommandData.az_el.az_speed)*AZ_GEAR_RATIO*CM_PULSES)
                    /(360*SPEED_UNIT);

  az_accel = ((int)(CommandData.az_el.az_accel)*AZ_GEAR_RATIO*CM_PULSES)
                    /(360*ACCEL_UNIT);

  /* elevation calibrations */
 

  el_low = CommandData.az_el.el - CommandData.az_el.el_height/2.0;
  el_high = CommandData.az_el.el + CommandData.az_el.el_height/2.0;

  el_low = (el_low < EL_MIN) ? EL_MIN : el_low;
  el_high = (el_high > EL_MAX) ? EL_MAX : el_high;

  el_enc = ReadData(elEncAddr);

  el = ((el_enc - elinfo.ref)/CNTS_PER_DEG) + CommandData.az_el.el_ref;

  dextdtheta = dxdtheta(el); // uses current elevation angle (okay for small
                             // step size?)

  ext_low = xoftheta(el_low);

  ext = xoftheta(el);
  
  el_start = (int)(((ext_low - ext)/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO
		   *CM_PULSES);

  el_speed = (int)(((dextdtheta*CommandData.az_el.el_speed/IN_TO_MM)
	            *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/SPEED_UNIT);

  el_accel = (int)(((dextdtheta*CommandData.az_el.el_accel/IN_TO_MM)
       	            *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/ACCEL_UNIT);

  N_steps = (el_high - el_low)/CommandData.az_el.el_step;
 
  bprintf(info, "Performing raster scan...");
  
  /* move in azimuth to one end of the scan range */
  
  bprintf(info, "Moving to end of az scan range...");    
  slew_cm(az_accel, az_speed, -(az_width/2), &azinfo);

  /* check to see if it got there */
  checkpos_cm(&azinfo);

  /* move in elevation to starting elevation */
  bprintf(info, "Moving to starting elevation...");    
  slew_cm(el_accel, el_speed, el_start, &elinfo);

  /* check to see if it got there */
  checkpos_cm(&elinfo);
  
  /* start the raster scan */

  step_count = 0;
  
  while (step_count < N_steps) {
   
    step_count++;   

    pos_arg = ( ((step_count - 1) % 2) == 0 ) ? az_width : -az_width;
   
    /* scan across in azimuth */
   
    bprintf(info, "Scanning in azimuth...");
    slew_cm(az_accel, az_speed, pos_arg, &azinfo);
    checkpos_cm(&azinfo);
     
    /* step in elevation */
    
    bprintf(info,"Elevation Step: %i", step_count);  
    bprintf(info, "Stepping in elevation...");

    el_enc = ReadData(elEncAddr);

    el = ((el_enc - elinfo.ref)/CNTS_PER_DEG) + CommandData.az_el.el_ref;

    dextdtheta = dxdtheta(el); // uses current elevation angle (okay for small
                               // step size?)

    dext = calc_dx(el, CommandData.az_el.el_step);

    step_size = (int)(((dext)/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO
		   *CM_PULSES);

    el_speed = (int)(((dextdtheta*CommandData.az_el.el_speed/IN_TO_MM)
	              *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/SPEED_UNIT);

    el_accel = (int)(((dextdtheta*CommandData.az_el.el_accel/IN_TO_MM)
         	      *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/ACCEL_UNIT);

    if (step_size > 0) {
      slew_cm(el_accel, el_speed, step_size, &elinfo);
      checkpos_cm(&elinfo);  
    }

    /* TODO - sjb: XY stage also did "el" scan with "az" steps. Do we want 
       this? */
  }

}

/* init_cm sets a Cool Muscle's internal parameters */

void init_cm(struct CMInfoStruct* cminfo) 
{

  char* KH_query = "?90\r%87\r";
  
  char* resolution = "K37=30\r";   // sets resolution to 50,000 pulses
                                   // per rotation and the speed unit to
				   // 10 pulse/s

  char* password = "W=924\r";      // unlocks the H parameters for writing
                                   // (these are gains for controller design)
  char* H0 = "H0=100\r";
  char* H1 = "H1=46\r";
  char* H2 = "H2=-2427\r";
  char* H3 = "H3=204\r";
  char* H4 = "H4=250\r";
  char* H5 = "H5=65\r";
  char* H6 = "H6=4\r";
  char* H7 = "H7=10\r";

  tcflush(cminfo->fd, TCIOFLUSH);

  /* write internal parameters */
  
  bprintf(info,"Writing %s motor internal parameters", cminfo->motorstr);
    
  if ( (write_cm(cminfo, resolution, strlen(resolution), 
                 " resolution parameter K37")) < 0) {
    cminfo->init = 0;
    return;
  }

  if ( (write_cm(cminfo, password, strlen(password), 
                 " password to access H parameters")) < 0) {
    cminfo->init = 0;
    return;
  }

  if ( strcmp(cminfo->motorstr, "az") == 0 ) {
    bprintf(info, "This is the az motor ==> writing H parameter values.");
     
    if ((write_cm(cminfo, H0, strlen(H0), " parameter H0")) < 0) {
      cminfo->init = 0;
      return;
    }

    if ((write_cm(cminfo, H1, strlen(H1), " parameter H1")) < 0) {
      cminfo->init = 0;
      return;
    }

    if ((write_cm(cminfo, H2, strlen(H2), " parameter H2")) < 0) {
      cminfo->init = 0;
      return;
    }

    if ((write_cm(cminfo, H3, strlen(H3), " parameter H3")) < 0) {
      cminfo->init = 0;
      return;
    }

    if ((write_cm(cminfo, H4, strlen(H4), " parameter H4")) < 0) {
      cminfo->init = 0;
      return;
    }
    
    if ((write_cm(cminfo, H5, strlen(H5), " parameter H5")) < 0) {
      cminfo->init = 0;
      return;
    }
 
    if ((write_cm(cminfo, H6, strlen(H6), " parameter H6")) < 0) {
      cminfo->init = 0;
      return;
    }

    if ((write_cm(cminfo, H7, strlen(H7), " parameter H7")) < 0) {
      cminfo->init = 0;
      return;
    }
  }
     
  /* query the motor to see what the K and H paramater values are */

  bprintf(info,"Querying %s motor K & H parameter values.", cminfo->motorstr);
  if ((write_cm(cminfo, KH_query, strlen(KH_query), 
       " query to check K & H parameters")) < 0) {
    cminfo->init = 0;
    return;
  } else if (read_cm(cminfo, 1) <= 0 ) {
    /* right now the program doesn't care whether a response is received */
    bprintf(err,"Failed to receive the queried %s motor K & H parameter values"
            , cminfo->motorstr);
  }

  /* TODO - sjb: do you want to check the K/H values you queried? */

  cminfo->init = 1;   

}

/* write_cm sends a command to a Cool Muscle stepper motor, and checks for 
   errors in writing the bytes */

int write_cm(struct CMInfoStruct *cminfo, char *cmd, int length, 
             const char *cmd_desc) 
{

  int n_bytes;

  n_bytes = write(cminfo->fd, cmd, length);

  if (n_bytes < 0) {
    berror(err, "Failed to write %s.", cmd_desc);
    return -1;
  } else if (n_bytes != length) {
    bprintf(err,"Wrote incorrect number of bytes for %s", cmd_desc);
    return -2;
  } else return 0;

} 

/* checkpos_cm checks if a motor reached the commanded position */

void checkpos_cm(struct CMInfoStruct *cminfo) 
{

  int in_position = 0;

  bprintf(info,"Waiting for in-position signal from %s motor...\n", 
          cminfo->motorstr);

  while (!in_position) {
    if ( read_cm(cminfo, 0) > 0 ) {
      in_position = 1;
    } else {
      bprintf(err, "Didn't receive %s motor in-position signal. Retrying...", 
              cminfo->motorstr);
    }
  }

}

/* allstop_cm sends disable and pause commands to both motors */

void allstop_cm()
{

  char *disable = "*\r";
  char *enable = "*1\r";

  int stopped = 0;

  while (!stopped){
    if (write_cm(&elinfo,disable,strlen(disable)," disable CML command")==0 
        && write_cm(&azinfo,disable,strlen(disable)," disable CML command")
        ==0) {

      stopped = 1;
    }
  }

  write_cm(&elinfo, enable, strlen(enable), " enable CML command");
  write_cm(&azinfo, enable, strlen(enable), " enable CML command");

}

/* read_cm waits for a response from a Cool Muscle to a query */

int read_cm(struct CMInfoStruct *cminfo, int read_flag) 
{

  char rxchar = 0;
  char status[10];          // store received chars to check for
                            // in-position message "Ux.1=8"
  char msg[4];
  int i, stat;

  unsigned int store = 0;   // 1 if received chars are to be stored
  int bytes_received = 0;   // total # of bytes received
  int read_stat, timeout, data_avail, count=0;
  struct timeval tv;
  
  timeout = read_flag ? 1 : 300;    
                                    
  fd_set rfds;

  tv.tv_sec = timeout;
  tv.tv_usec = 0;

  FD_ZERO(&rfds);
  FD_SET(cminfo->fd, &rfds);

  #ifdef CM_DEBUG
  bprintf(info, "Waiting for response from %s motor...", cminfo->motorstr);
  #endif

  /* wait for a response */

  data_avail = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
  
  if (!data_avail) { // select() returns zero if the timeout is reached
    bytes_received = 0;
    bprintf(err,"Communication with %s motor timed out.\n\n", cminfo->motorstr);
  } else if (data_avail < 0) {
    bprintf(err,"%s motor: select() call has returned an error.\n", 
            cminfo->motorstr);
    bytes_received = 0;
    data_avail = 0;
  }
        
  FD_ZERO(&rfds);
  FD_SET(cminfo->fd, &rfds);

  while (data_avail) {
  
    /* if no character at buffer, wait for timeout before
       accepting end of response. */

    if(select(FD_SETSIZE, &rfds, NULL, NULL, &tv)>0) {
      
      read_stat = read(cminfo->fd, &rxchar, 1);
      
      if (read_stat < 0) {
        bytes_received = 0;
        data_avail = 0;          // break out of while loop
      } else {
        rxchar = rxchar & 0xFF;  
        bytes_received++;
        #ifdef CM_DEBUG
	bprintf(info, "%c", rxchar);
	#endif
        if (rxchar == 'U') {  
	  store = 1;
	}
	if (store) {
	  status[count++] = rxchar;
	  if (rxchar == '\r') {
	    store = 0;
	    bprintf(info, "status message: %s", status);
            for (i = 5; i < strlen(status)-1; i++) {
              msg[i-5] = status[i];
	    }
            stat = atoi(msg);
            bprintf(info, "stat = %i", stat);
            switch(stat) {
	      case 0:
                bprintf(err, "%s motor is still running!",cminfo->motorstr);
		data_avail = 0;
		break;
	     
	      case 1:
                bprintf(err, "%s motor position error overflow!",
		        cminfo->motorstr);
	         data_avail = 0;	
		break;

	      case 2:             
                bprintf(err, "%s motor over regen voltage limit!",
		        cminfo->motorstr);
		data_avail = 0;
		break;
 
	      case 4: 
                bprintf(err, "%s motor over load/current!",cminfo->motorstr);
		data_avail = 0;
		break;

	      case 8: 
                bprintf(info, "%s motor is in-position.",cminfo->motorstr);
		data_avail = 0;
		break;

	      case 16: 
                bprintf(err, "%s motor is disabled!",cminfo->motorstr);
		data_avail = 0;
		break;

	      case 32: 
                bprintf(err, "%s motor torque limit reached!",cminfo->motorstr);
		data_avail = 0;
		break;

	      case 128:	
                bprintf(err, "%s motor over temperature limit!",
		        cminfo->motorstr);
		data_avail = 0;
		break;

	      case 256:         
                bprintf(err, "%s motor push mode timeout not reached!",
		        cminfo->motorstr);
		data_avail = 0;
		break;

	      case 512: 
                bprintf(err, "%s motor emergency stop!",cminfo->motorstr);
		data_avail = 0;
		break;

	      default:  
                bprintf(err, "Could not parse %s motor status!",
		        cminfo->motorstr);
		data_avail = 0;
	    }
	  }
        }
      }
      if (bytes_received >= MAX_CHARS) {
        bytes_received = 0;
        data_avail = 0; 
      }
      
    } else { 
      data_avail = 0;
    }
  }
  
  return bytes_received;
}

/* open_cm sets up a serial port for communication with a Cool Muscle stepper 
   motor */

void open_cm(char *dev_name, struct CMInfoStruct *cminfo)
{

  int fd;
  struct termios settings;

  bprintf(info, "Attempting to open %s motor at %s\n", 
          cminfo->motorstr, dev_name);

  /*for some reason, setting O_NDELAY below was necessary to prevent this 
    process from waiting for open() */

  if( (fd = open(dev_name, O_RDWR | O_NDELAY | O_NOCTTY)) < 0) {
    berror(err, "Error opening %s motor device %s", cminfo->motorstr, dev_name);
    cminfo->open = 0;
    return;
  } else { 
    bprintf(info, "%s motor port %s is now open\n", cminfo->motorstr, dev_name);
    cminfo->fd = fd;
  }

  if(tcgetattr(fd, &settings) < 0) {
    berror(err, "tcgetattr failed");
    cminfo->open = 0;
    return;
  }

  /* clear Character size; set no parity bits; set 1 stop bit */
  settings.c_cflag &= ~(CSTOPB | CSIZE | PARENB);

  /* set 8 data bits; set local port; enable receiver */
  settings.c_cflag |= (CS8 | CLOCAL | CREAD);

  /* disable all software flow control */
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);

  /* disable output processing (raw output) */
  settings.c_oflag &= ~OPOST;

  /* disable input processing (raw input) */
  settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  if(cfsetospeed(&settings, B38400) < 0) {	
    berror(err, "Error setting output baud rate");
    cminfo->open = 0;
    return;  
  }

  if(cfsetispeed(&settings, B38400) < 0) {	
    berror(err, "Error setting input baud rate");
    cminfo->open = 0;
    return;      
  }

  if(tcsetattr(fd, TCSANOW, &settings) < 0) {
    berror(err,"tcsetattr failed");
    cminfo->open = 0;
    return;
  }

  cminfo->open = 1;
}

/* close_cm closes serial port communications with a Cool Muscle stepper motor 
 */

void close_cm(struct CMInfoStruct* cminfo)
{

  bprintf(info,"Closing connection to %s motor.\n", cminfo->motorstr);

  if (cminfo->open == 0) {
    bprintf(info, "%s motor port is already closed!", cminfo->motorstr);
  } else if ( (close(cminfo->fd)) > 0 ) {
    cminfo->open = 0;
    bprintf(info, "Connection to %s motor is now closed.", cminfo->motorstr);
  } else berror(err, "Failed to close motor serial port");
}

/* calcdx returns the change in lin. act. extension (mm) given a step in 
   elevation angle (deg) */

double calc_dx(double theta, double dtheta)
{

  // dtheta: el step size, comes from user
  // theta: read the current elevation encoder value into this variable

  double x;      // current extension of linear actuator in mm
  double x1;     // extension of lin. act. at the new angle
  double theta1; // new angle after elevation step occurs
  double dx;     // lin. act. thrust required to achieve dtheta 

  x = xoftheta(theta);

  theta1 = theta + dtheta;

  x1 = xoftheta(theta1);

  dx = x1 - x;

  return dx;

}

/* xoftheta returns the extension of the lin. actuator (in mm) given the 
  telescope elevation angle (deg) */

double xoftheta(double theta) 
{

  double x;

  x = sqrt( pow(A,2) + pow(B,2) - 2*A*B*cos((C-theta)*PI/180.0) ) - D;

  return x;
} 

/* returns derivative of lin. act. extension. w.r.t. elevation angle (mm/deg) */

double dxdtheta(double theta)
{
  double deriv;

  deriv = -(A*B*sin((C - theta)*PI/180.0))/(sqrt(pow(A,2) + pow(B,2) 
           - 2*A*B*cos((C-theta)*PI/180.0)));

  deriv = (deriv < 0) ? -deriv*(PI/180.0) : deriv*(PI/180.0);

  return deriv;  // want to return a speed (always +ve). Direction is taken
                 // care of by position value (also conversions from mm/rad
		 // to mm/deg)
}

/* serial threads */

void* azelComm(void* arg) 
{

  int ser_attempts = 0;  // number of attempts to open port
  int init_attempts = 0; // number of attempts to initialize motor

  /* initialize values in the motor info structs */

  azinfo.fd = 0;
  azinfo.open = 0;
  azinfo.init = 0;
  azinfo.closing = 0;
  azinfo.ref = 0;
  strncpy(azinfo.motorstr, "az", 3);

  elinfo.fd = 0;
  elinfo.open = 0;
  elinfo.init = 0;
  elinfo.closing = 0;
  elinfo.ref = 0;
  strncpy(elinfo.motorstr, "el", 3);

  nameThread("AzEl");
  bprintf(startup, "Starting serial thread for motors.");
 
  /* try to open the ports */

  while (azinfo.open == 0) {

    open_cm(AZ_DEVICE, &azinfo);
    
    if (ser_attempts == 10) {
      bputs(err, "Unable to open az motor port after 10 attempts.\n");
    }
    
    ser_attempts++;

    if (azinfo.open == 1) {
      bprintf(info, "Opened the az motor port on attempt %i", ser_attempts);
    } else sleep(1);	     
  }

  ser_attempts = 0;

  while (elinfo.open == 0) {

    open_cm(EL_DEVICE, &elinfo);
    
    if (ser_attempts == 10) {
      bputs(err, "Unable to open el motor port after 10 attempts.\n");
    }
    
    ser_attempts++;

    if (elinfo.open == 1) {
      bprintf(info, "Opened the el motor port on attempt %i", ser_attempts);
    } else sleep(1);	     
  }

  /* initialize motor internal parameters */

  bprintf(info, "Initializing az motor");  

  while (azinfo.init == 0) {

    init_cm(&azinfo);

    if (init_attempts == 10) {
      bputs(err, "Could not initalize the az motor after 10 attempts.\n");
    } 
    
    init_attempts++;
   
    if (azinfo.init == 1) {
      bprintf(info, "Initialized the az motor on attempt %i", init_attempts);
    } else sleep(1);
  }

  init_attempts = 0;

  bprintf(info, "Initializing el motor");  

  while (elinfo.init == 0) {

    init_cm(&elinfo);

    if (init_attempts == 10) {
      bputs(err, "Could not initalize the el motor after 10 attempts.\n");
    } 
    
    init_attempts++;
   
    if (elinfo.init == 1) {
      bprintf(info, "Initialized the el motor on attempt %i", init_attempts);
    } else sleep(1);
  }

  while (1) {

    /* TODO: NEED SOME KIND OF CHECKING OF ERROR STATES HERE */

    if (azinfo.closing == 1) {
      close_cm(&azinfo);
      usleep(10000);
    } else if (elinfo.closing == 1) {
      close_cm(&elinfo);
      usleep(10000);
    } else {

      AzElScan();

      usleep(10000);

      /* TODO - sjb: probably want, "too many errors, reconnecting logic here */
    }

  }
  
  return NULL;
}

/* AzElScan: do whatever has been commanded */

void AzElScan()
{

  static struct BiPhaseStruct* elEncAddr;
  static struct BiPhaseStruct* azEncAddr;

  static int firsttime = 1;

  if (firsttime) {

    elEncAddr = GetBiPhaseAddr("adc1_enc_el");
    azEncAddr = GetBiPhaseAddr("adc1_enc_az");

    firsttime = 0;
  }

  switch(CommandData.az_el.mode) {

    case AzElNone:
      break;

    case AzElDisable:
      allstop_cm();
      CommandData.az_el.mode = AzElNone;
      break;

    case AzElGoto:
      if (!CommandData.az_el.cmd_disable) {   
        goto_cm();
        CommandData.az_el.mode = AzElNone;
      }
      break;

    case AzElRaster:
      if (!CommandData.az_el.cmd_disable) {   
        raster_cm();       
        CommandData.az_el.mode = AzElNone;
      }
      break;

    case AzElSet:
      azinfo.ref = ReadData(azEncAddr);
      elinfo.ref = ReadData(elEncAddr);
      break;

    default:
      bputs(err, "Invalid scan mode specified\n");
  }

}

/* WriteAzEl writes relevant data to frame */

void WriteAzEl()
{

  static struct NiosStruct* azWidthAddr;
  static struct NiosStruct* azVelAddr;
  static struct NiosStruct* elVelAddr;
  static struct NiosStruct* azAccelAddr;
  static struct NiosStruct* elAccelAddr;
  static struct NiosStruct* elStepAddr;
  static struct NiosStruct* elHeightAddr;
  static struct NiosStruct* elGotoAddr;
  static struct NiosStruct* azGotoAddr;
  static struct NiosStruct* azStartAddr;
  static struct NiosStruct* elStartAddr;

  static int firsttime = 1;
  
  if (firsttime) {

    azWidthAddr = GetNiosAddr("width_az");
    azVelAddr = GetNiosAddr("v_az");
    elVelAddr = GetNiosAddr("v_el");
    azAccelAddr = GetNiosAddr("a_az");
    elAccelAddr = GetNiosAddr("a_el");
    elStepAddr = GetNiosAddr("step_el");
    elHeightAddr = GetNiosAddr("height_el");
    elGotoAddr = GetNiosAddr("el");
    azGotoAddr = GetNiosAddr("az");
    azStartAddr = GetNiosAddr("az_ref");
    elStartAddr = GetNiosAddr("el_ref");
    
    firsttime = 0;
  }


  WriteData(azWidthAddr,(CommandData.az_el.az_width)*(65535.0/180.0),
            NIOS_QUEUE);  

  WriteData(azVelAddr,(CommandData.az_el.az_speed)*(65535.0/10.0), NIOS_QUEUE);

  WriteData(elVelAddr, (CommandData.az_el.el_speed)*(65535.0/5.0), NIOS_QUEUE);

  WriteData(azAccelAddr,(CommandData.az_el.az_accel)*(65535.0/5.0),NIOS_QUEUE);

  WriteData(elAccelAddr,(CommandData.az_el.el_accel)*(65535.0/5.0),NIOS_QUEUE);

  WriteData(elStepAddr,(CommandData.az_el.el_step)*(65535.0/5.0), NIOS_QUEUE);

  WriteData(elHeightAddr,(CommandData.az_el.el_height)*(65535.0/99.0), 
            NIOS_QUEUE);

  WriteData(elGotoAddr,((CommandData.az_el.el)+10.0)*(65535.0/99.0),
            NIOS_QUEUE);  

  WriteData(azGotoAddr,((CommandData.az_el.az)+90.0)*(65535.0/180.0),
            NIOS_QUEUE);

  WriteData(azStartAddr,((CommandData.az_el.az_ref)+90.0)*(65535.0/180.0), 
            NIOS_QUEUE);

  WriteData(elStartAddr,((CommandData.az_el.el_ref)+10.0)*(65535.0/99.0), 
            NIOS_QUEUE);
}
