/*******************************************************************

az-el.c -- mcplib code to drive the Spider test cryostat az-el mount 
           to do scans for beam mapping. Controls the az and el Cool 
           Muscle stepper motors. 
           
Author:    Jamil A. Shariff
           BallAst Group, Univeristy of Toronto

Updated:   May 26, 2011

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
#define SPEED_UNIT 1      // pulses/s in one speed unit
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


/* CMInfoStruct: contains info on state of serial comms with a Cool Muscle */

struct CMInfoStruct {

  int fd;                 // file descriptor
  int open;               // 0 = closed, 1 = open
  int init;               // 0 = uninitialized, 1 = initialized
  int closing;            // 1 if in the process of closing
//int reset;              // 1 to reset serial connection
//unsigned int err_count; // tally of serial comm. errors 
  char motorstr[3];       // name of motor

} azinfo, elinfo;

static pthread_t azcomm_id;     // gets assigned a thread ID
static pthread_t elcomm_id;

void nameThread(const char*);   // mcplib.c

void startAzEl();
void endAzEl();
void AzElScan();

static void* azComm(void* arg); // serial thread routine
static void* elComm(void* arg);

static void open_cm(char *dev_name, struct CMInfoStruct *cminfo);
static void close_cm(struct CMInfoStruct *cminfo);
static void checkpos_cm(struct CMInfoStruct *cminfo);
static void init_cm(struct CMInfoStruct *cminfo);
static void allstop_cm(struct CMInfoStruct *azinfoptr, struct CMInfoStruct 
                       *elinfoptr); 
static void slew_cm(int accel, int speed, int position, struct CMInfoStruct 
                    *cminfo);
static void goto_cm(int az_accel, int el_accel, int az_speed, int el_speed, 
                    int az, int el, struct CMInfoStruct *azinfoptr, struct 
                    CMInfoStruct *elinfoptr);

static void raster_cm(int az_width, int az_speed, int el_speed, int az_accel, 
		      int el_accel, int step_size, int el_min, int el_max, 
                      struct CMInfoStruct *azinfoptr, struct CMInfoStruct 
                      *elinfoptr); 

static int drive_cm(int accel, int speed, int position, struct CMInfoStruct 
                    *cminfo);
static int read_cm(struct CMInfoStruct *cminfo, int read_flag);
static int write_cm(struct CMInfoStruct *cminfo, char *cmd, int length, 
                    const char *cmd_desc); 

static double calc_dx(double theta, double dtheta);
static double xoftheta(double theta); 

/* start/end serial threads */

void startAzEl()
{
  pthread_create(&azcomm_id, NULL, &azComm, NULL);
  pthread_create(&elcomm_id, NULL, &elComm, NULL);
}

void endAzEl()
{
 
  int i = 0;
  azinfo.closing = 1;
  elinfo.closing = 1; 
  
  while(azinfo.open == 1 && elinfo.open == 1 && i++ < 100) usleep(10000);

}

/* drive_cm moves a Cool Muscle stepper motor to the specified position 
   at the specified speed and acceleration. */

int drive_cm(int accel, int speed, int position, struct CMInfoStruct* cminfo)
{

  int acount=0, scount=0, pcount=0;
  
  char *execframe = "^\r";            // the command to execute motion

  char accelframe[COMMAND_SIZE];      // string for acceleration value
  char speedframe[COMMAND_SIZE];      // string for speed value
  char posframe[COMMAND_SIZE];        // string for position value

  /* read acceleration, speed and position values into character strings  */
  
  acount = sprintf(accelframe, "A=%i\r", accel);
  scount = sprintf(speedframe, "S=%i\r", speed);
  pcount = sprintf(posframe, "P=%i\r", position);

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

  if ( (write_cm(cminfo, posframe, pcount, " position CML command")) < 0 ) {
    return -3;
  }

  #ifdef CM_DEBUG
  bprintf(info,"Writing excecute motion CML command to %s motor.", 
          cminfo->motorstr);
  #endif

  if ( (write_cm(cminfo, execframe, strlen(execframe), 
        " execute motion CML command")) < 0 ) {
    return -4;
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

void goto_cm(int az_accel, int el_accel, int az_speed, int el_speed, 
             int az, int el, struct CMInfoStruct *azinfoptr, struct 
             CMInfoStruct *elinfoptr)
{  
  slew_cm(az_accel, az_speed, az, azinfoptr);
  slew_cm(el_accel, el_speed, el, elinfoptr);

  checkpos_cm(azinfoptr);
  checkpos_cm(elinfoptr);
}

/* raster_cm performs a raster scan with the parameters given by the arguments 
   (all in Cool Muscle units) using a sequence of slew_cm calls */

void raster_cm(int az_width, int az_speed, int el_speed, int az_accel, 
               int el_accel, int step_size, int el_min, int el_max, struct 
               CMInfoStruct *azinfoptr, struct CMInfoStruct *elinfoptr) 
{ 
 
  int step_count;           // keep track of el steps                
  int pos_arg;              // argument to pass as az position value

  int N_steps;              // number of elevation steps

  N_steps = (el_max - el_min)/step_size;
 
  bprintf(info, "Performing raster scan...");
  
  /* move in azimuth to one end of the scan range */
  
  bprintf(info, "Moving to end of az scan range...");    
  slew_cm(az_accel, az_speed, -(az_width/2), azinfoptr);

  /* check to see if it got there */
  checkpos_cm(azinfoptr);

  /* move in elevation to min_el */

  bprintf(info, "Moving to min. elevation...");    
  slew_cm(el_accel, el_speed, el_min, elinfoptr);

  /* check to see if it got there */
  checkpos_cm(elinfoptr);
  
  /* start the raster scan */

  step_count = 0;
  
  while (step_count < N_steps) {
   
    step_count++;   

    pos_arg = ( ((step_count - 1) % 2) == 0 ) ? az_width/2 : -az_width/2;
   
    /* scan across in azimuth */
   
    bprintf(info, "Scanning in azimuth...");
    slew_cm(az_accel, az_speed, pos_arg, azinfoptr);
    checkpos_cm(azinfoptr);
     
    /* step in elevation */
    
    bprintf(info,"Elevation Step: %i", step_count);  
    bprintf(info, "Stepping in elevation...");
    if (step_size > 0) {
      slew_cm(el_accel, el_speed, el_min+(step_count*step_size), elinfoptr);
      checkpos_cm(elinfoptr);  
    }

    /* TODO - sjb: XY stage also did "el" scan with "az" steps. Do we want this?
     */
  }

}

/* init_cm sets a Cool Muscle's internal parameters */

void init_cm(struct CMInfoStruct* cminfo) 
{

  char* KH_query = "?90\r%87\r";
  
  char* resolution = "K37=100\r";  // sets resolution to 50,000 pulses
                                   // per rotation and the speed unit to
				   // 1 pulse/s

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

void allstop_cm(struct CMInfoStruct *azinfoptr, struct CMInfoStruct *elinfoptr)
{

  char *pause = "]\r";   // CML for "pause motion"
  char *disable = ")\r"; // CML for "de-energize motor windings"
  char *enable = "(\r";  // CML for "enable motor windings"

  int stopped = 0;

  while (!stopped){
    if (write_cm(elinfoptr,disable,strlen(disable)," disable CML command")==0 &&
        write_cm(azinfoptr,disable,strlen(disable)," disable CML command")==0 &&
        write_cm(elinfoptr,pause,strlen(pause)," disable CML command")==0 &&
        write_cm(azinfoptr,pause,strlen(pause)," disable CML command")==0) {
      stopped = 1;
    }
  }

  write_cm(elinfoptr, enable, strlen(enable), " enable CML command");
  write_cm(azinfoptr, enable, strlen(enable), " enable CML command");

}

/* read_cm waits for a response from a Cool Muscle to a query */

int read_cm(struct CMInfoStruct *cminfo, int read_flag) 
{

  char rxchar = 0;
  char status[7];           // store received chars to check for
                            // in-position message "Ux.1=8"

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
        //#ifdef CM_DEBUG
	bprintf(info, "%c", rxchar);
	//#endif
        if (rxchar == 'U') {  
	  store = 1;
	}
	if (store) {
	  status[count++] = rxchar;
	  if (count == 6) {
	    store = 0;
	    if ( strcmp("Ux.1=8", status) == 0) {
              /* in-position signal has been received.*/
	      bprintf(info,"%s motor is in position.", cminfo->motorstr);
	      data_avail = 0;  // break out of while loop
	    } else {
              bprintf(err,"\nAn error has occurred. Status message: %s\n", 
                      status);
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

  deriv = (deriv < 0) ? -deriv : deriv;

  return deriv;  // want to return a speed (always +ve). Direction is taken
                 // care of by position value
}

/* serial threads */

void* azComm(void* arg) {

  int ser_attempts = 0;  // number of attempts to open port
  int init_attempts = 0;     // number of attempts to initialize motor

  /* initialize values in the azinfo struct */

  azinfo.fd = 0;
  azinfo.open = 0;
  azinfo.init = 0;
  azinfo.closing = 0;
  strncpy(azinfo.motorstr, "az", 3);

  nameThread("AzComm");
  bprintf(startup, "Starting az motor serial thread.");
 
  /* try to open the port */

  while (azinfo.open == 0) {

    open_cm(AZ_DEVICE, &azinfo);
    
    if (ser_attempts == 10) {
      bputs(err, "Unable to open az motor port after 10 attempts.\n");
    }
    
    ser_attempts++;

    if (elinfo.open == 1) {
      bprintf(info, "Opened the az motor port on attempt %i", ser_attempts);
    } else sleep(1);	     
  }

  /* initialize motor internal parameters */

  init_attempts = 0;

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

  while (1) {

    /* TODO: NEED SOME KIND OF CHECKING OF ERROR STATES HERE */

    if (azinfo.closing == 1) {
      close_cm(&azinfo);
      usleep(10000);
    } else {
      /* I can't figure out what else this thread needs to do. */ 
      /* TODO - sjb: probably want, "too many errors, reconnecting logic here */
    }
  }
  return NULL;
}

void* elComm(void* arg) 
{

  int ser_attempts=0;      // number of attempts to open port
  int init_attempts=0;     // number of attempts to initialize motor

  /* initialize values in the elinfo struct */

  elinfo.fd = 0;
  elinfo.open = 0;
  elinfo.init = 0;
  elinfo.closing = 0;
  strncpy(elinfo.motorstr, "el", 3);

  nameThread("ElComm");
  bprintf(startup, "Starting el motor serial thread.");

  /* try to open the port */

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

    if (elinfo.closing == 1) {
      close_cm(&elinfo);
      usleep(10000);
    } else {
      /* I can't figure out what else this thread needs to do. */
      /* TODO - sjb: probably want, "too many errors, reconnecting logic here */
    }
  }
  return NULL;
}

/* AzElScan: actually do stuff */

void AzElScan()
{

  /* scan parameters -- (calibrated to Cool Muscle units) */

  int az_accel;
  int az_speed;
  int az_dest;
  int az_width;

  int el_accel;
  int el_speed;
  int el_dest;
  int el_step;
  int el_min;
  int el_max;
  
  double ext;        // extension of linear actuator
  static double ext0;// extension at startup (corresponds to a position of 0
                     // in CM pulses)
  double dextdtheta; // deriv. of extension w.r.t. el angle

  double dext;       // change in lin. act. ext. for given step in el. angle
  double el;         // current el. comes from encoder over Bbus, magically
  double ext_min;
  double ext_max;

  static struct BiPhaseStruct* elAddr;

  static struct NiosStruct* azWidthAddr;
  static struct NiosStruct* azVelAddr;
  static struct NiosStruct* elVelAddr;
  static struct NiosStruct* azAccelAddr;
  static struct NiosStruct* elAccelAddr;
  static struct NiosStruct* elStepAddr;
  static struct NiosStruct* elMinAddr;
  static struct NiosStruct* elMaxAddr;
  static struct NiosStruct* elGotoAddr;
  static struct NiosStruct* azGotoAddr;

  static int firsttime = 1;

  if (firsttime) {
    ext0 = xoftheta(el);

    elAddr = GetBiPhaseAddr("adc1_enc_el");

    azWidthAddr = GetNiosAddr("width_az");
    azVelAddr = GetNiosAddr("v_az");
    elVelAddr = GetNiosAddr("v_el");
    azAccelAddr = GetNiosAddr("a_az");
    elAccelAddr = GetNiosAddr("a_el");
    elStepAddr = GetNiosAddr("step_el");
    elMinAddr = GetNiosAddr("el_min");
    elMaxAddr = GetNiosAddr("el_max");
    elGotoAddr = GetNiosAddr("el");
    azGotoAddr = GetNiosAddr("az");
    
    firsttime = 0;
  }

  el = ReadData(elAddr);

  switch(CommandData.az_el.mode){

    case AzElNone:
      break;

    case AzElDisable:
      allstop_cm(&azinfo, &elinfo);
      CommandData.az_el.mode = AzElNone;
      break;

    case AzElGoto:
      az_dest = ((int)(CommandData.az_el.az)*AZ_GEAR_RATIO*CM_PULSES)/360;

      az_speed = ((int)(CommandData.az_el.az_speed)*AZ_GEAR_RATIO*CM_PULSES)
                 /(360*SPEED_UNIT);

      az_accel = ((int)(CommandData.az_el.az_accel)*AZ_GEAR_RATIO*CM_PULSES)
                 /(360*ACCEL_UNIT);

      /* compute lin. act. extension */
      ext = xoftheta(CommandData.az_el.el);
      dextdtheta = dxdtheta(el);                   // not ideal to use 
                                                   // current angle for  
                                                   // something that is a 
                                                   // continuous func. of theta

      el_dest =  ((ext-ext0)/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES;

      el_speed = ((dextdtheta*CommandData.az_el.el_speed/IN_TO_MM)
                 *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/SPEED_UNIT;

      el_accel = ((dextdtheta*CommandData.az_el.el_accel/IN_TO_MM)
                 *ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES)/ACCEL_UNIT;
    
      goto_cm(az_accel, el_accel, az_speed, el_speed, az_dest, el_dest, 
              &azinfo, &elinfo);

      CommandData.az_el.mode = AzElNone;
      break;

    case AzElRaster:
      az_width = ((int)(CommandData.az_el.az_width)*AZ_GEAR_RATIO*CM_PULSES)
                 /360;

      az_speed = ((int)(CommandData.az_el.az_speed)*AZ_GEAR_RATIO*CM_PULSES)
                 /(360*SPEED_UNIT);

      az_accel = ((int)(CommandData.az_el.az_accel)*AZ_GEAR_RATIO*CM_PULSES)
                 /(360*ACCEL_UNIT);

      ext_min = xoftheta(CommandData.az_el.el_min);
      ext_max = xoftheta(CommandData.az_el.el_max);
      dext = calc_dx(el, CommandData.az_el.el_step);
      dextdtheta = dxdtheta(el); // uses current elevation angle (okay for 
                                 // small step size?)

      el_step = (dext/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES;

      el_speed = ((dextdtheta*CommandData.az_el.el_speed/IN_TO_MM)*ROT_PER_INCH
                 *EL_GEAR_RATIO*CM_PULSES)/SPEED_UNIT;

      el_accel = ((dextdtheta*CommandData.az_el.el_accel/IN_TO_MM)*ROT_PER_INCH
                 *EL_GEAR_RATIO*CM_PULSES)/ACCEL_UNIT;

      el_min =  ((ext_min-ext0)/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES;
      el_max =  ((ext_max-ext0)/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO*CM_PULSES;

      /* TODO: need to make sure raster scan starts at el = 0 */

      raster_cm(az_width, az_speed, el_speed, az_accel, el_accel, el_step, 
                el_min, el_max, &azinfo, &elinfo); 
      
      CommandData.az_el.mode = AzElNone;
      break;

    default:
      bputs(err, "Invalid scan mode specified\n");
  }

  WriteData(azWidthAddr,(CommandData.az_el.az_width)*(65535.0/180.0),
            NIOS_QUEUE);
  
  WriteData(azVelAddr,(CommandData.az_el.az_speed)*(65535.0/10.0), NIOS_QUEUE);
 
  WriteData(elVelAddr, (CommandData.az_el.el_speed)*(65535.0/5.0), NIOS_QUEUE);

  WriteData(azAccelAddr,(CommandData.az_el.az_accel)*(65535.0/5.0), NIOS_QUEUE);

  WriteData(elAccelAddr,(CommandData.az_el.el_accel)*(65535.0/5.0), NIOS_QUEUE);

  WriteData(elStepAddr,(CommandData.az_el.el_step)*(65535.0/5.0), NIOS_QUEUE);

  WriteData(elMinAddr,(CommandData.az_el.el_min)*(65535.0/44.0), NIOS_QUEUE);

  WriteData(elMaxAddr,((CommandData.az_el.el_max)-45.0)*(65535.0/44.0), 
            NIOS_QUEUE); 

  WriteData(elGotoAddr,((CommandData.az_el.el)+10.0)*(65535.0/99.0),NIOS_QUEUE);
  
  WriteData(azGotoAddr,((CommandData.az_el.az)+90.0)*(65535.0/180.0),
            NIOS_QUEUE);
    
}

