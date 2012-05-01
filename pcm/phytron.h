/* phytron.h: code for interfacing to phytron motor controller
 *
 * This software is copyright (C) 2012 University of Toronto
 *
 * This file is part of the Spider flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef PHYTRON_H
#define PHYTRON_H

#include <termios.h>
#include <unistd.h>

/* Errors:
 * Phytron functions will return an integer error code.
 * The code should be interpreted as an error bitfield, as follows.
 *
 * Success is indicated by PH_ERR_OK, which should always be 0
 */
#define PH_ERR_OK       0x0000  //everything is okay
#define PH_ERR_OOD      0x0001  //unexpected out of data
#define PH_ERR_TIMEOUT  0x0002  //timeout
#define PH_ERR_TTY      0x0004  //serial error (open or write, so far)
#define PH_ERR_BAD_WHO  0x0008  //bad 'who' value. NOT always checked
#define	PH_ERR_BUSY     0x0010  //bus busy (as per Take/Release)
#define	PH_ERR_RESPONSE 0x0020  //malformed response from device/bad checksum
#define PH_ERR_POLL     0x0040  //didn't find all steppers in Poll
                                //or attempting to use an unpolled stepper
#define PH_ERR_NAK      0x0080  //NAK in response. Message/controller error
#define PH_ERR_PARAM    0x0100  //invalid parameter value (out of range)
#define PH_ERR_BAUD     0x0200  //unknown baud rate for stepper

/* Chatter:
 * The verbosity level of the Phytron library.
 * The levels are cumultive, printing everything from all lower levels
 */
#define	PH_CHAT_NONE  0   //print absolutely nothing
#define	PH_CHAT_ERR   1   //print only errors and warnings
#define	PH_CHAT_ACT   2   //also print bus actions
#define	PH_CHAT_SEIZE 3   //indicate when the bus is seized
#define PH_CHAT_BUS   4   //also print all bus chatter

/* Status:
 * The status of each stepper is by its status bitfield
 */
#define PH_STEP_USED	0x0001	  /* this stepper is/will be used */
#define PH_STEP_OK   	0x0002	  /* stepper has been polled, reset to repoll */
#define PH_STEP_INIT	0x0004	  /* stepper has been initialized */

/* Miscellaneous defines */
#define PH_BUS_NAME_LEN     0x100 //human-readable names
#define PH_BUS_BUF_LEN      0x100 //serial buffer
#define PH_BUS_NACT         6     //max number of actuators
#define PH_BUS_COMM_RETRIES 5     //default number of Comm retries
#define PH_STEPS_PER_REV    200.  //motor (micro)steps per revolution
                                  //probably "better" to be in bus->stepper


/* # Communication Errors before triggering a reconnect to the serial port */
#define PH_ERR_MAX  5

/* These error bits set in phtron->error increment the error count */
#define PH_ERR_MASK     0x0006    // PH_ERR_TIMEOUT and PH_ERR_TTY

struct phytron_stepper {
  unsigned short status;        //status field for each stepper
  char name[PH_BUS_NAME_LEN];   //name of the stepper
  char addr;                    //bus address of the contorller ('0', '1', etc)
  char axis;                    //axis of the stepper ('X' or 'Y')
  speed_t baud;                 //serial port baud rate to use for this stepper
  //parameters for the Phytron move commands
  int gear_teeth;               //number of gear teeth on rotator
  int usteps;                   //ustep reoslution
  double vel;                   //velocity (degree/s)
  //TODO should I set acc to non-default?
  //double acc;                   //acceleration (degree/s/s)
  double ihold;                 //hold current (A)
  double imove;                 //move (and boost) current (A)
};

struct phytron {
  struct phytron_stepper stepper[PH_BUS_NACT];
  int fd;                     //file descriptor for bus serial port
  const char *tty;            //name of the tty device
  char name[PH_BUS_NAME_LEN];	//named prefix to bus-related messages
  char buffer[PH_BUS_BUF_LEN];//buffer for responses
  int seized;                 //thread-unsafe concurrency for bus
  int chatter;                //verbosity of ezstep functions
  //TODO expose errors to user. Per-stepper basis? Autocycle?
  int error;                  //most recent error code
  int err_count;              //errors since last successful bus communication
};

/* initialize a struct phytron. Needed for all other Phytron funuctions
 * bus: struct phytron to initialize
 * tty: name of tty device for communications 
 * name: bus name. prepended to messages
 * inhibit_chatter: chatter level
 */
int Phytron_Init(struct phytron* bus, const char *tty, const char* name,
    int chatter);

/* Attempt to reset the serial connection to the stepper.
 * Closes and re-opens the port, attempt to reinitialize.
 * Returns 1 if it successfully reset or 0 if the reset failed.
 */
int Phytron_Reset(struct phytron* bus);

/* add a stepper at index 'who' to poll list (0 <= who < PH_BUS_NACT)
 * Located on bus at 'addr' (eg "1X"), with human-readable 'name'
 */
int Phytron_Add(struct phytron* bus, int who, const char* addr,
    const char* name);

/* simple blocking mechanism for device 'who' to take and release bus
 * if call to Phytron_IsUsable fails, Take and IsTaken will return PH_ERR_POLL
 * IsTaken returns PH_ERR_OK when who already has the bus
 */
int Phytron_Take(struct phytron* bus, int who);
int Phytron_Release(struct phytron* bus, int who);
int Phytron_IsTaken(struct phytron* bus, int who);

/* Phytron_Send sends command string 'what' to device 'who'
 * Phytron_NASend does the same, but omits axis ('X'/'Y') since some commands
 * won't work with the axis specified (they just want the controller addr)
 *
 * Ideally would want a smart send that decides for each command which to use
 */
int Phytron_Send(struct phytron* bus, int who, const char* what);
int Phytron_NASend(struct phytron* bus, int who, const char* what);

/* receive response from bus
 */
int Phytron_Recv(struct phytron* bus);

/* read and discard all responses currently available on the bus
 */
int Phytron_Recv_Flush(struct phytron* bus);

/* send command 'what' to 'who' and recieve response
 * will retry (every second) under certain error conditions (busy)
 * Phytron_Comm retries PH_BUS_COMM_RETRIES times
 * Phytron_CommRetry specifies number of retries
 * Phytron_CommVarg uses printf-style 'fmt' and arguments for what
 *
 * The NAComm variants are similar but use NASend (see above)
 */
int Phytron_Comm(struct phytron* bus, int who, const char* what);
int __attribute__((format(printf,3,4))) Phytron_CommVarg(struct phytron* bus,
    int who, const char* fmt, ...);
int Phytron_NAComm(struct phytron* bus, int who, const char* what);
int __attribute__((format(printf,3,4))) Phytron_NACommVarg(struct phytron* bus,
    int who, const char* fmt, ...);

/* send query command 'what' to 'who' to get integer response
 * if successful *val will be assigned the responose, otherwise it is unchanged
 * TODO: maybe want to read a string response, also/instead of int
 */
int Phytron_ReadInt(struct phytron* bus, int who, const char* what, int* val);

/* indicate that 'who' should be rescanned next time Phytron_Poll is called
 * when using Phytron_PollInit, will also lead to reinitialization
 */
int Phytron_ForceRepoll(struct phytron* bus, int who);

/* poll the bus for all steppers on poll list
 * will always skip steppers that are already okay
 */
int Phytron_Poll(struct phytron* bus);

/* Same as Phytron_Poll, except will call 'init' for each newfound stepper
 * init should return 0 on failure, take pointer to struct phytron and int who
 */
int Phytron_PollInit(struct phytron* bus, int (*phinit)(struct phytron*,int));

/* checks stepper status to see if stepper is usable
 * NB: returns boolean values and not an error code
 */
int Phytron_IsUsable(struct phytron* bus, int who);

/* sends a status query to a stepper to see if it will accept commands
 * PH_READY will be set if busy (NB: usually that bit means the opposite)
 * other error codes are returned as usual, with PH_READY also set on error
 * (meaning is reversed so that error states make IsBusy report true)
 * TODO: is IsBusy meaningful for phytrons?
 */
//int Phytron_IsBusy(struct phytron* bus, int who);


/* Simple motion:
 * Handles basic, generically useful movement commands.
 * More advanced uses are not (yet?) included in this library
 */

/* sets hold current for simple motion moves (in A)
 */
int Phytron_SetIHold(struct phytron* bus, int who, double current);

/* sets move current for simple motion moves (in A)
 */
int Phytron_SetIMove(struct phytron* bus, int who, double current);

/* sets velocity for simple motion moves (in degree/s)
 */
int Phytron_SetVel(struct phytron* bus, int who, double vel);

/* sets gear ratio for simple motion moves (in teeth per revolution)
 */
int Phytron_SetGear(struct phytron* bus, int who, int teeth);

/* use the gear ratio to convert degrees to (micro)steps (D2S) 
 * and (micro)steps to degrees (S2D)
 */
inline int Phytron_D2S(struct phytron* bus, int who, double degrees);
inline double Phytron_S2D(struct phytron* bus, int who, int steps);

/* sets acceleration for simple motion moves (in steps/s)
 */
//int Phytron_SetAccel(struct phytron* bus, int who, int acc);

/* sends motion parameters to the motor (ihold, imove, vel, etc)
 * calibrates parameter units as expected by the controller
 */
int Phytron_SendParams(struct phytron* bus, int who);

/* Terminate movement
 */
int Phytron_Stop(struct phytron* bus, int who);

/* Terminate movement, but don't even check if the stepper is polled
 */
int Phytron_Stop_All(struct phytron* bus);

/* Relative move by 'delta' (in degrees from current location)
 * delta can be positive or negative
 */
int Phytron_Move(struct phytron* bus, int who, double delta);

/* Relative move by 'delta' at speed vel (sets stepper.vel)
 */
int Phytron_MoveVel(struct phytron* bus, int who, double delta, double vel);

/* Continuous move at speed vel (sets stepper.vel)
 * vel can be positive or negative
 */
int Phytron_ContinuousVel(struct phytron* bus, int who, double vel);

#endif  //PHYTRON_H

