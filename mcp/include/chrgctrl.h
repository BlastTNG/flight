/*************************************************************************
 
 chrgctrl.h -- function declarations and #define statements for 
               charge controller code. 		

*************************************************************************/

#ifndef CHRGCTRL_H
#define CHRGCTRL_H

/* MODBUS restrictions on # of bytes in request/response packets */

#define MAX_QUERY_LENGTH    256
#define MAX_RESPONSE_LENGTH 256
#define MAX_DATA_LENGTH     MAX_RESPONSE_LENGTH - 6

/* MAX_DATA LENGTH = MAX_RESPONSE_LENGTH - server address (1 B)
                                         - CRC            (2 B)
                                         - function code  (1 B)
                                         - byte count     (1 B)
                                         - 1 to make even (1 B) 
                                         ----------------------
                                         -                (6 B) */

#define MAX_READ_REGS MAX_DATA_LENGTH/2  // each register is 2 bytes


#define FALSE 0 
#define TRUE  1

/* MODBUS exception codes */

#define COMMS_FAILURE         0
#define ILLEGAL_FUNCTION     -1
#define ILLEGAL_DATA_ADDRESS -2
#define ILLEGAL_DATA_VALUE   -3
#define SLAVE_DEVICE_FAILURE -4
#define ACKNOWLEDGE          -5
#define SLAVE_DEVICE_BUSY    -6
#define NEGATIVE_ACKNOWLEDGE -7
#define MEMORY_PARITY_ERROR  -8
#define PORT_FAILURE        -11

#define TO 100000	                 // timeout delay at end of packet in us
                                         // this value is just a guess
                                         // MODBUS spec says frame starts after 
                                         // silent interval of >= 3.5 char. times

/* charge controller info struct */

struct CCInfo {
  
  int fd;                      // file descriptor
  int open;                    // 0 for closed, 1 for open
  int err;                     // 1 if an exception was returned
  int closing;                 // 1 if in process of closing
  int reset;                   // 1 if a reset is required
};

extern struct CCInfo chrgctrlinfo[2];

/* charge controller data struct 
   written to by serial thread in chrgctrl.c */

struct CCData {

  double V_batt;               // battery voltage from sense terminals
  double V_arr;                // solar array input voltage
  double I_batt;               // current to battery
  double I_arr;                // current from solar array (+/- 20%)
  double V_targ;               // target charging voltage
  
  int T_hs;                    // heatsink temperature

  uint16_t  fault_field;  // fault bitfield
  unsigned int alarm_field_hi; // alarm high bitfield
  unsigned int alarm_field_lo; // alarm low bitfield
  unsigned int led_state;      // state of status LEDs on front of unit 
  unsigned int charge_state;   // charging state of controller   
};

extern struct CCData ChrgCtrlData[2];
                                         
/* function declarations */


/*************************************************************** 
 open_chrgctrl -- sets up a serial port for MODBUS communication
                  with the charge controller
 arguments:

 dev_name      -- pointer to character string containing path
                  of serial device file
 i_cc          -- index of the charge controller we are using
 ***************************************************************/
 void open_chrgctrl(const char *dev_name, int i_cc);


/*************************************************************** 
 close_chrgctrl -- closes serial port for MODBUS communication
                   with the charge controller
 ***************************************************************/
 void close_chrgctrl(int i_cc);



/*************************************************************** 
 query_chrgctrl -- sends MODBUS request packet to charge
                   controller to read data from MODBUS registers
 arguments:

 dev_addr       -- MODBUS device address
 start_addr     -- address of starting register 
 count          -- number of consecutive registers to read
 dest           -- int array where data will be stored
 fd             -- UNIX file descriptor of serial port

 returns an integer indicating the number of bytes that were
 received from data registers
 ***************************************************************/ 
 int query_chrgctrl(int dev_addr, unsigned int start_addr, 
                    unsigned int count, int *dest, int fd);


/*************************************************************** 
 response_chrgctrl -- receives MODBUS response packet from 
                      charge controller containing data read
                      from MODBUS registers 
 arguments:

 dest           -- int array where data will be stored
 query          -- pointer to the request packet
 fd             -- UNIX file descriptor of serial port

 returns an integer indicating the number of bytes that were
 received from data registers
 ***************************************************************/ 
 int response_chrgctrl(int *dest, unsigned char *query, int fd);

#endif  // CHRGCTRL_H
