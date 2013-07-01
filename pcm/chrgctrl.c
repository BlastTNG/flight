/*************************************************************************** 

pcm: the Spider master control program

This software is copyright (C) 2002-2013 University of Toronto

This file is part of pcm.

pcm is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
at your option) any later version.
 
pcm is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with pcm; if not, write to the Free Software Foundation, Inc.,
59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

**************************************************************************/


/*************************************************************************
 
 chrgctrl.c -- pcm code to read data from TriStar MPPT-60 charge
               controller over serial port using MODBUS comms. protocol.

*************************************************************************/

#include <stdio.h>           // standard input/output             
#include <sys/time.h>        // time structures for select()
#include <pthread.h>         // POSIX threads
#include <math.h>         
#include <string.h>
#include <stdlib.h>
#include <unistd.h>          // POSIX symbolic constants
#include <termios.h>         // POSIX terminal control definitions
#include <fcntl.h>           // file control definitions


#include "chrgctrl.h"        // charge controller MODBUS comms 
                             // function declarations 

#include "mcp.h" 

/*
#include "share/channels.h"
#include "tx.h"
 */

#define CHRGCTRL1_DEVICE "/dev/ttySI3" // change depending upon serial hub port
#define CHRGCTRL2_DEVICE "/dev/ttySI4" // change depending upon serial hub port
#define QUERY_SIZE 6
#define CHECKSUM_SIZE 2               

/*#define CHRGCTRL_VERBOSE            // uncomment to see useful debug info*/

static pthread_t chrgctrlcomm1_id;     // thread ID
static pthread_t chrgctrlcomm2_id;     // thread ID
static int char_interval_timeout;     // time to wait for next MODBUS byte in packet

struct CCInfo chrgctrlinfo[2];           // device status info -- see chrgctrl.h
struct CCData ChrgCtrlData[2];           // data from device -- see chrgctrl.h

static void* chrgctrlComm(void* cc);
static unsigned int modbus_crc(unsigned char *buf, int start, int cnt);

void nameThread(const char*);	      // in mcp.c
extern short int InCharge;            // in tx.c

//static int nlog = 0;                // counts number of frames printed to chrgctrl.log

//FILE *fp;                           // pointer to file chrgctrl.log, which logged 
                                      // frames returned by controller for debugging

/* create charge controller serial thread */

void startChrgCtrl()
{  
  pthread_create(&chrgctrlcomm1_id, NULL, chrgctrlComm, (void*)0);
  pthread_create(&chrgctrlcomm2_id, NULL, chrgctrlComm, (void*)1);
}



/* end charge controller serial thread */

void endChrgCtrl()          // declare in mcp.c along with startChrgCtrl
{

  int i = 0;
  chrgctrlinfo[0].closing = 1; // tells the serial thread to shut down
  chrgctrlinfo[1].closing = 1; // tells the serial thread to shut down

  while ((chrgctrlinfo[0].open == 1) && (chrgctrlinfo[1].open == 1) && (i++ < 100)) {
    usleep(10000);
  }
}


/* thread routine: continously poll charge controller for data */

void* chrgctrlComm(void* cc)
{
  const char *COMM[] = {CHRGCTRL1_DEVICE, CHRGCTRL2_DEVICE};
  const int slave = 0x01;   // default MODBUS device address   
                            // for charge controller
  int i_cc = (int)cc;       // which charge controller we should be talking to
                            
/*Relics from the test program -- have no use in mcp  
  const int nfaults = 11;   // number of faults conditions
  const int nalarms = 20;   // number of alarm conditions
  const int nstates = 10;   // number of charging states 
*/

  int n_conn=0, n_reconn=0; // loop constants
  int query_no;             // IDs which frame was from which query (for err. check)  
  int data_lengths[6];      // one element for each query

  double Vscale;            // voltage scaling factor
  double Iscale;            // current scaling factor

  char tname[10];
  
  /* struct for parsing alarm and fault bitfields, and other general
     status lookup tables. 
     
     This is a relic from the test program. Not needed for mcp
     since bitfields are in derived.c

  struct status {

    int flag;
    char *message;
  };
  */

  /* declare one chrgctrl_rawdata struct for each contiguous set of registers 
     to be read */

  struct chrgctrl_rawdata {
    
    int num;                // # of data bytes returned by controller
    int arr[MAX_READ_REGS]; // array to store serial data read
    
  } scale, elec, temp, fault, alarm, charge; 

  /*  

  This is a relic from the test program. Not needed for mcp
  since bitfields are in derived.c
  
  struct status fault_bits[] = {

    {1, "overcurrent"},
    {2, "FETs shorted"},
    {4, "software bug"},
    {8, "battery high voltage disconnect"},
    {16, "array high voltage disconnect"},
    {32, "settings switch changed"},
    {64, "custom settings edit"},
    {128, "remote temp. sensor shorted"},
    {256, "remote temp. sensor disconnected"},
    {512, "EEPROM retry limit"},
    {1024, "slave control timeout"}
  };

  struct status alarm_bits[] = {

    {1, "remote temp. sensor open"},
    {2, "remote temp. sensor shorted"},
    {4, "remote temp. sensor disconnected"},
    {8, "heatsink temp. sensor open"},
    {16, "heatsink temp. sensor shorted"},
    {32, "high temp. current limit"},
    {64, "current limit"},
    {128, "current offset"},
    {256, "battery sense out of range"},
    {512, "battery sense disconnected"},
    {1024, "uncalibrated"},
    {2048, "remote temp. sensor miswire"},
    {4096, "high voltage disconnect"},
    {8192, "undefined"},
    {16384, "system miswire"},
    {32768, "MOSFET open"},
    {65536, "P12 voltage off"},
    {131072, "high input voltage current limit"},
    {262144, "ADC input max"},
    {524288, "controller was reset"}
  };

  struct status charge_states[] = {

    {0, "START"},
    {1, "NIGHT_CHECK"},
    {2, "DISCONNECT"},
    {3, "NIGHT"},
    {4, "FAULT"}, 
    {5, "MPPT"}, 
    {6, "ABSORPTION"},
    {7, "FLOAT"},
    {8, "EQUALIZE"},
    {9, "SLAVE"}
  };
  */

  /* initialize values in charge controller info struct */

  chrgctrlinfo[i_cc].open = 0;
  chrgctrlinfo[i_cc].err = 0;
  chrgctrlinfo[i_cc].closing = 0;
  chrgctrlinfo[i_cc].reset = 0;
/*chrgctrlinfo[i_cc].closing = 1; // just for testing */

//  fp = fopen("/home/shariff/chrgctrl.log", "a");   // open the log file that records
                                                     // charge controller serial frames

  sprintf(tname, "ChrgC%1d", i_cc+1);
  nameThread(tname);

  /* check whether this is the ICC */

  while (!InCharge) {

    usleep(20000);
  }

  /* try to open serial port */

  while (chrgctrlinfo[i_cc].open == 0) {

    open_chrgctrl(COMM[i_cc], i_cc);

    if (chrgctrlinfo[i_cc].open == 0) {
      if (n_conn == 10) {
        bputs(err, "Failed to open after 10 attempts");
      }
      
      n_conn++;
      
      sleep(1);
    }
  }
  
  if (n_conn>=10) {
    bprintf(info, "Opened on attempt number %i", n_conn); 
  }
  
  while (1) {

    if (chrgctrlinfo[i_cc].closing == 1) {

      close_chrgctrl(i_cc);
      usleep(10000); 

    } else if (chrgctrlinfo[i_cc].reset == 1) {
      
      /* if there's an error, reset connection to charge controller */

      close_chrgctrl(i_cc);
     
      if (n_reconn == 0) {
        bprintf(info,"Error, attempting to re-connect.");
      }

      while (chrgctrlinfo[i_cc].open == 0) {

        open_chrgctrl(COMM[i_cc], i_cc);

#ifdef CHRGCTRL_VERBOSE
        if (n_reconn == 10) { 
	  bprintf(err,"Failed to re-open charge controller #%d after 10 attempts.", i_cc);
	}
#endif

        n_reconn++;

        if (chrgctrlinfo[i_cc].open == 1) { // reset succeeded

#ifdef CHRGCTRL_VERBOSE
            bprintf(info, "Re-opened charge controller #%d on attempt %i.", i_cc, n_reconn);
#endif
	  chrgctrlinfo[i_cc].reset = chrgctrlinfo[i_cc].err = 0;       

	} else {
            sleep(1);
        }
      }
    } else {                          // query the charge controller      

      /* retrieve voltage and current scaling factors (needed to turn the 
         ADC output into meaningful V & I values) from register 
         addresses 1-4 */

      scale.num = query_chrgctrl(slave, 1, 4, scale.arr, chrgctrlinfo[i_cc].fd);

      //      usleep(10000);  // ignore these -- just for debugging

      /* poll charge controller for battery and array voltages and currents,
         which range from register addresses [26 or]27-30 */
       
      elec.num = query_chrgctrl(slave, 26, 5, elec.arr, chrgctrlinfo[i_cc].fd);   

      //      usleep(10000);

      /* heatsink temperature in degrees C (addr 36) */

      temp.num = query_chrgctrl(slave, 36, 1, temp.arr, chrgctrlinfo[i_cc].fd);   

      //      usleep(10000);

      /* charge controller fault bitfield (addr 45) */

      fault.num = query_chrgctrl(slave, 45, 1, fault.arr, chrgctrlinfo[i_cc].fd);   

      //      usleep(10000);

      /* charge controller alarm bitfield (spans 2 regs with addrs 47,48) */

      alarm.num = query_chrgctrl(slave, 47, 2, alarm.arr, chrgctrlinfo[i_cc].fd);

      //      usleep(10000);

      /* controller LED state, charge state and target charging voltage (addrs 50, 51, 52) */

      charge.num = query_chrgctrl(slave, 50, 3, charge.arr, chrgctrlinfo[i_cc].fd);

      //      usleep(10000);

      /* error handling for communication failures or corrupted data */

      data_lengths[0] = scale.num;
      data_lengths[1] = elec.num;
      data_lengths[2] = temp.num;
      data_lengths[3] = fault.num;
      data_lengths[4] = alarm.num;
      data_lengths[5] = charge.num;

      
      for (query_no = 0; query_no < 6; query_no++) {

        if (data_lengths[query_no] <= 0) {

        /* Most of these won't happen, except PORT_FAILURE and COMMS_FAILURE
           EDIT: Except when serial comms. messes up! 
	 */

 #ifdef CHRGCTRL_VERBOSE
          switch (data_lengths[query_no]) {

	    case COMMS_FAILURE: 
	      bputs(err, "Charge controller produced no data.");
              break;

	    case ILLEGAL_FUNCTION:
              bputs(err, "Invalid MODBUS function code in query packet.");
              break;

	    case ILLEGAL_DATA_ADDRESS: 
              bputs(err, "Invalid MODBUS register address in query packet.");
              break;

	    case ILLEGAL_DATA_VALUE:
              bputs(err, "Invalid data value in MODBUS query packet.");
              break;

       	    case SLAVE_DEVICE_FAILURE:
	      bputs(err, "Unrecoverable MODBUS device error during request.");
              break;

  	    case ACKNOWLEDGE:
              bputs(err, "Charge controller is still processing request.");
              break;

	    case SLAVE_DEVICE_BUSY:
              bputs(err, "Charge controller is busy: try query again.");
              break;

	    case MEMORY_PARITY_ERROR: 
	      bputs(err, "Charge controller detected memory parity error.\n");
              break;

	    case PORT_FAILURE:
              bputs(err,"Error in reading from or writing to charge controller.");
              break;

 	    default:
              bputs(err, "An unknown charge controller error occurred.");  
	  }
  #endif
          chrgctrlinfo[i_cc].err = 1;
          query_no = 6; // break out of for loop upon first problem encountered
	}
      }

      if (chrgctrlinfo[i_cc].err == 1) {
        chrgctrlinfo[i_cc].reset = 1;
        /* put charge controller data values into obvious error state */
        ChrgCtrlData[i_cc].V_batt = 0.0;
        ChrgCtrlData[i_cc].V_arr = 0.0;
        ChrgCtrlData[i_cc].I_batt = 0.0;
        ChrgCtrlData[i_cc].I_arr = 0.0;
        ChrgCtrlData[i_cc].V_targ = 0.0;
        ChrgCtrlData[i_cc].T_hs = 0;
        ChrgCtrlData[i_cc].charge_state = 10;
        ChrgCtrlData[i_cc].led_state = 20;
        continue; // go back up to top of infinite loop
      } else if (n_reconn > 0) {

        bprintf(info, "Connected");
        n_reconn = 0; // managed this query cycle w/o errors; reset for the next one 
      }
    
      /* compute values of things that need scaling */

      Vscale = *scale.arr + (*(scale.arr+1))/65536.0;
      Iscale = *(scale.arr+2) + (*(scale.arr+3))/65536.0;

      //      ChrgCtrlData[i_cc].V_batt = *elec.arr * Vscale/32768.0;
      //      ChrgCtrlData[i_cc].V_arr =  *(elec.arr+1) * Vscale/32768.0;
      //      ChrgCtrlData[i_cc].I_batt = *(elec.arr+2) * Iscale/32768.0;
      //      ChrgCtrlData[i_cc].I_arr =  *(elec.arr+3) * Iscale/32768.0;

      /* *elec.arr is the battery voltage at the charging terminals
	 (less accurate) as opposed to the sense terminals. 
         EDIT: actually these terminals are shorted, so it doesn't
         matter which reading is used. */

      ChrgCtrlData[i_cc].V_batt = *(elec.arr+1) * Vscale/32768.0;
      ChrgCtrlData[i_cc].V_arr =  *(elec.arr+2) * Vscale/32768.0;
      ChrgCtrlData[i_cc].I_batt = *(elec.arr+3) * Iscale/32768.0;
      ChrgCtrlData[i_cc].I_arr =  *(elec.arr+4) * Iscale/32768.0;
    
      ChrgCtrlData[i_cc].V_targ = *(charge.arr+2) * Vscale/32768.0;

      ChrgCtrlData[i_cc].T_hs = *temp.arr;
      ChrgCtrlData[i_cc].fault_field = *fault.arr;

      ChrgCtrlData[i_cc].alarm_field_hi = *alarm.arr;
      ChrgCtrlData[i_cc].alarm_field_lo = *(alarm.arr+1);

      ChrgCtrlData[i_cc].led_state = *charge.arr;
      ChrgCtrlData[i_cc].charge_state = *(charge.arr+1);
    
      /* Relics from test program -- not for mcp!

      need to write data now. For testing purposes, just print stuff  

      printf("\n");
      printf("Voltage Scaling Factor: %.3f \n", Vscale);
      printf("Current Scaling Factor: %.3f \n \n", Iscale);    
      printf("Battery Sense Voltage: %.3f V \n", ChrgCtrlData[i_cc].V_batt);
      printf("Solar Array Input Voltage: %.3f V \n", ChrgCtrlData[i_cc].V_arr);
      printf("Output Current to Battery: %.3f A \n", ChrgCtrlData[i_cc].I_batt);
      printf("Input Current from Array: %.3f A \n", ChrgCtrlData[i_cc].I_arr);
      printf("Target Battery Charging Voltage: %.3f V \n \n", ChrgCtrlData[i_cc].V_targ);
      printf("Heatsink Temperature: %d C \n \n", ChrgCtrlData[i_cc].T_hs);

      parse fault bitfield 

      if (!ChrgCtrlData[i_cc].fault_field)
        printf("No fault conditions detected \n");

      for (i = 0; i < nfaults ; i++) {
     
        if (ChrgCtrlData[i_cc].fault_field & fault_bits[i].flag)
          printf("FAULT: %s \n", fault_bits[i].message);  
      }

      parse alarm bitfield 

      if (!ChrgCtrlData[i_cc].alarm_field)
        printf("No alarm conditions detected \n");

      for (j = 0; j < nalarms ; j++) {
     
        if (ChrgCtrlData[i_cc].alarm_field & alarm_bits[j].flag)
          printf("ALARM: %s \n", alarm_bits[j].message);  
      }

      determine charging state from lookup table    

      printf("Charging State: %s \n \n", charge_states[ChrgCtrlData[i_cc].charge_state].message);

      i = j = 0;*/
    }
    //    usleep(10000);
  }
  return NULL;
}


/* open serial port */

void open_chrgctrl(const char *dev_name, int i_cc)
{  
  struct termios settings;
		
  if ((chrgctrlinfo[i_cc].fd = open(dev_name, O_RDWR)) < 0) {
   
    /* port failed to open */
     
    chrgctrlinfo[i_cc].open = 0;
    return;

  } else {

    fcntl(chrgctrlinfo[i_cc].fd, F_SETFL, 0);
    chrgctrlinfo[i_cc].open = 1;

  }

  /* initialize settings by getting terminal attributes */
  if (tcgetattr(chrgctrlinfo[i_cc].fd, &settings)) {
    chrgctrlinfo[i_cc].open = 0;
    return;
  }


  /* charge controller only works at 9600 baud */

  cfsetispeed(&settings, B9600); // set input baud rate
  cfsetospeed(&settings, B9600); // set output baud rate

  char_interval_timeout = TO;    // defined in chrgctrl.h

  /* these flags are supposedly suitable for MODBUS */

  settings.c_line = 0; 

  /* input modes */

  settings.c_iflag |= IGNBRK;
  settings.c_iflag |= IGNPAR;
  settings.c_iflag &=~ PARMRK;
  settings.c_iflag &=~ INPCK;
  settings.c_iflag &=~ ISTRIP;
  settings.c_iflag &=~ INLCR;
  settings.c_iflag &=~ IGNCR;
  settings.c_iflag &=~ ICRNL;
  settings.c_iflag &=~ IUCLC;
  settings.c_iflag &=~ IXON;
  settings.c_iflag |= IXANY;
  settings.c_iflag &=~ IXOFF;
  settings.c_iflag &=~ IMAXBEL;

  /* output modes */

  settings.c_oflag |= OPOST;
  settings.c_oflag &=~ OLCUC;
  settings.c_oflag &=~ ONLCR;
  settings.c_oflag &=~ OCRNL;
  settings.c_oflag |= ONOCR;
  settings.c_oflag &=~ ONLRET;
  settings.c_oflag &=~ OFILL;
  settings.c_oflag &=~ OFDEL;

  /* control modes */

  settings.c_cflag &=~ CSIZE;
  settings.c_cflag |= CS8;
  settings.c_cflag |= CSTOPB;  // 2 stop bits: flag changed from MODBUS lib
  settings.c_cflag |= CREAD;
  settings.c_cflag &=~ PARENB; // no parity
  settings.c_cflag &=~ PARODD; // no parity
  settings.c_cflag &=~ HUPCL;
  settings.c_cflag |= CLOCAL;
  settings.c_cflag &=~ CRTSCTS;

  /* local modes */

  settings.c_lflag &=~ ISIG;
  settings.c_lflag &=~ ICANON;
  settings.c_lflag &=~ ECHO;
  settings.c_lflag |= IEXTEN;

  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;

  cfmakeraw(&settings);

  tcsetattr(chrgctrlinfo[i_cc].fd, TCSANOW, &settings);
}



/* close the serial port */

void close_chrgctrl(int i_cc)
{

  #ifdef CHRGCTRL_VERBOSE
    bprintf(info, "close_chrgctrl: closing connection to charge controller.");
  #endif

  if (chrgctrlinfo[i_cc].open == 0) {
  
    #ifdef CHRGCTRL_VERBOSE
      bprintf(info, "close_chrgctrl: charge controller is already closed!");
    #endif

  } else {
  
    #ifdef CHRGCTRL_VERBOSE
      bprintf(info, "close_chrgctrl: closing serial port.");
    #endif

      if (chrgctrlinfo[i_cc].fd >= 0) {
      close(chrgctrlinfo[i_cc].fd);
      }
    chrgctrlinfo[i_cc].open = 0;
    
    #ifdef CHRGCTRL_VERBOSE
      bprintf(info, "close_chrgctrl: connection to port is now closed.");
    #endif
  }    
}



/* send MODBUS query packet in order to read registers */

int query_chrgctrl(int dev_addr, unsigned int start_addr, unsigned int count, 
                   int *dest, int fd)
{

  const int function = 0x04;                        // MODBUS function code: 
                                                    // "read input registers"

  int num_bytes, write_num;

  unsigned char packet[QUERY_SIZE + CHECKSUM_SIZE +1]; // 6 + 2

  unsigned int temp_crc;

  size_t string_length = QUERY_SIZE;

  /*#ifdef CHRGCTRL_VERBOSE
    int i;
    #endif*/

  count = (count > MAX_READ_REGS) ? MAX_READ_REGS : count;

  --start_addr;
  
  /*build request packet*/

  packet[0] = dev_addr;
  packet[1] = function;
  packet[2] = start_addr >> 8;
  packet[3] = start_addr & 0x00FF;
  packet[4] = count >> 8;
  packet[5] = count & 0x00FF;  

  temp_crc = modbus_crc(packet, 0, string_length);

  packet[string_length++] = temp_crc >> 8;         // element 6
  packet[string_length++] = temp_crc & 0x00FF;     // element 7
  packet[string_length] = 0;                       // element 8: not written; likely useless.

  /*#ifdef CHRGCTRL_VERBOSE

      Print to stderr the hex value of each character that is about to be sent
      to the MODBUS slave. Replace this with mcp-friendly debug code
   

  for (i=0; i < string_length; i++) {

    fprintf(stderr, "[%.2X]", packet[i]);
  }
  fprintf(stderr, "\n");
  #endif */

  /* flush the input and output streams */

  tcflush(fd, TCIOFLUSH);

  #ifdef CHRGCTRL_VERBOSE
    bprintf(info, "query_chrgctrl: sending MODBUS query packet.");
  #endif

  write_num = write(fd, packet, string_length);

  tcflush(fd, TCIFLUSH); // maybe not necessary

  if (write_num > -1) {

    num_bytes = response_chrgctrl(dest, packet, fd);
    //num_bytes = PORT_FAILURE; // just for testing purposes.
  } else {
    num_bytes = PORT_FAILURE;
  }

  return(num_bytes);
}



/* response_chrgctrl -- receive MODBUS response packet from charge controller */

int response_chrgctrl(int *dest, unsigned char *query, int fd)
{
  /*unsigned short crc, crc_calc;*/
  
  unsigned char data[MAX_RESPONSE_LENGTH];
  int temp;
  int i;//j;
  //  char frame[50];
  //  int index=0, count=0;
  int rxchar = PORT_FAILURE;
  int data_avail = FALSE;
  int data_length;        // # of bytes of register data received 
  int bytes_received = 0; // total # of bytes received
  int read_stat;
  int timeout = 1;        // 1 second
  //  int nlog = 0;

  struct timeval tv;

  fd_set rfds;

  tv.tv_sec = timeout;
  tv.tv_usec = 0;

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  #ifdef CHRGCTRL_VERBOSE
    bprintf(info, "response_chrgctrl: waiting for response...");
  #endif

  /* wait for a response */

  data_avail = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
  
  if (!data_avail) { // select() returns zero if the timeout is reached

    bytes_received = data_length = 0;

    #ifdef CHRGCTRL_VERBOSE
      bputs(err, "Communication with charge controller timed out.");
    #endif 
  }

  tv.tv_sec = 0;
  tv.tv_usec = char_interval_timeout;

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  while (data_avail) {
  
    /* if no character at buffer, wait char_interval_timeout before
     * accepting end of response. */

    if(select(FD_SETSIZE, &rfds, NULL, NULL, &tv)) {

      read_stat = read(fd, &rxchar, 1);

      if (read_stat < 0) {

        bytes_received = data_length = PORT_FAILURE;
        data_avail = FALSE; // break out of while loop

      } else {
 
        rxchar = rxchar & 0xFF;  
        data[bytes_received++] = rxchar;

      }

      if (bytes_received >= MAX_RESPONSE_LENGTH) {
      
        bytes_received = data_length = PORT_FAILURE;
        data_avail = FALSE; 
      }

      /* The stuff below is superseded by code to convert each frame to a string for printing
         to the log file chrgctrl.log. */

      // Print the hex value of each character that is received.
      //  #ifdef CHRGCTRL_VERBOSE
      //        printf("<%.2X>", rxchar);
	//  #endif      

    } else { 
      data_avail = FALSE;
    }
  }

  /* store the returned packet as a string */

  //  for (j = 0; j < bytes_received; j++) {

  //    index += count;

  //    count = sprintf(&(frame[index]), "%.2x", data[j]);
  //  }

    //    bprintf(info, "[%s]", frame);


    /*#ifdef CHRGCTRL_VERBOSE
    fprintf(stderr, "\n");
    #endif*/

/*  
  if (bytes_received) {

    crc_calc = modbus_crc(data, 0, bytes_received - 2);

       last two received bytes are copied into high and low bytes 
     * of unsigned short int crc 

    crc = data[bytes_received - 2];
    crc = (unsigned) crc << 8;
    crc = crc | (unsigned) data[bytes_received - 1];

       compare crc returned in response packet with crc calculated by
     * modbus_crc 

    if (crc_calc != crc) {

      fprintf(stderr, "CRC received does not match CRC calculated:\n");
      fprintf(stderr, "CRC received: %#X", crc);
      fprintf(stderr, " -- CRC calculated: %#X\n",crc_calc);
		
      bytes_received = data_length = 0;
    }

  }*/
  
    /* check for exception response -- the second byte in the query packet
     * is the MODBUS function code. If the MODBUS query-response cycle occurs
     * without an error, the second byte in the response packet will also be
     * the function code. If an exception occurs, the second byte in the 
     * response packet will be this function code incremented by 0x80. Hence
     * we can check if an exception has occured by checking for an inequality
     * of the second bytes of the query and response packets: */
    
  if (bytes_received && (data[1] != query[1])) {

       /* print the returned packet for debugging purposes: */
    //if (nlog < 10000) {
    //      if (nlog == 0) {
	//	fprintf(fp, "\n\n New Session \n\n");
    //      }     
      // fprintf(fp, "ERROR: [%s]\n", frame);
      // fflush(fp);
      // nlog++;
      // }

       /* if exception occurs, third byte in packet is exception code, whose negative is 
       computed here */

    bytes_received = data_length = 0 - data[2];
  }
  
  if (bytes_received > 0) {

    /* print the returned packet for debugging purposes: */
    // if (nlog < 10000) {
    //    if (nlog == 0) {
	//	fprintf(fp, "\n\n New Session \n\n");
    //    }     
      // fprintf(fp, "       [%s]\n", frame);
      // fflush(fp);
      // nlog++;
      // }

    /* if no error occurs, the third byte is the count of bytes 
     * read from MODBUS registers */

    data_length =  data[2];
 
    /*#ifdef CHRGCTRL_VERBOSE // this consistency check is no longer needed
      fprintf(stderr, "number of data bytes (returned) = %d \n", data_length);
      fprintf(stderr, "number of data bytes (computed) = %d \n", 
              bytes_received - 5);
      fprintf(stderr, "\n");
      #endif*/

    /* i is the # of registers read, which is half the # of bytes */

    for (i = 0; i < data_length/2 ; i++) { 

      /* shift register high byte to temp */

      temp = data[3 + i*2] << 8;

      /* OR with low byte */

      temp |= data[4 + i*2];

      /* write each register data value into the destination array */
      
      dest[i] = temp;
    }
  }
  return data_length;
}



/******************************************************************** 
 modbus_crc  -- calculates crc high and low byte of query packet 
 
 arguments:
 
 buf         -- array containing packet to be sent to controller
 start       -- starting position in packet
 cnt         -- number of bytes in packet

 returns calculated CRC error checksum (2 bytes)
*********************************************************************/

unsigned int modbus_crc(unsigned char *buf, int start, int cnt) 
{

  int i, j;
  unsigned int temp, temp2, flag;

  temp=0xFFFF;

  for (i=start; i<cnt; i++) {

    temp = temp ^ buf[i];

    for (j=1; j<=8; j++) {
      
      flag = temp & 0x0001;
      temp = temp >> 1;

      if (flag) { 
        temp = temp ^ 0xA001;
      }
    }
  }

  /* Reverse byte order. */

  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;

  return temp;
}
