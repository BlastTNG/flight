/* ezstep.h: contains channel list specific prototypes and definitions
 *
 * This software is copyright (C) 2010 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_EZSTEP_H
#define INCLUDE_EZSTEP_H

#include <stdint.h>
/* Errors:
 * EZStep functions will return a 2-byte error code. The LSB contains data 
 * returned by the device itself, as per EZ_ERROR mask, EZ_READY, EZ_STATUS.
 * stepper errors for EZ_ERROR mask given by EZ_SERR_*
 * The MSB contains an error bitfield from this library
 *
 * I reserve the right to grow the error code to more than two bytes
 *
 * Success is indicated by EZ_BUS_OK, which should always be 0
 */
#define EZ_ERROR     0x0F
#define EZ_READY     0x20
#define EZ_STATUS    0x40
#define EZ_SERR_NONE    0 /* No error */
#define EZ_SERR_INIT    1 /* Initialisation error */
#define EZ_SERR_BADCMD  2 /* Bad command */
#define EZ_SERR_BADOP   3 /* Bad operand */
#define EZ_SERR_COMM    5 /* Communications error */
#define EZ_SERR_NOINIT  7 /* Not initialised */
#define EZ_SERR_OVER    9 /* overload error */
#define EZ_SERR_NOMOVE 11 /* Move Not allowed */
#define EZ_SERR_BUSY   15 /* Command overflow */

#define EZ_ERR_OK	    0x0000  // everything is okay
#define EZ_ERR_OOD	    0x0100  // unexpected out of data
#define EZ_ERR_TIMEOUT	0x0200  // timeout
#define EZ_ERR_TTY	    0x0400  // serial error (open or write, so far)
#define EZ_ERR_BAD_WHO	0x0800  // bad 'who' value. NOT always checked
#define	EZ_ERR_BUSY	    0x1000  // bus busy (as per Take/Release)
#define	EZ_ERR_RESPONSE	0x2000  // malformed response from device/bad checksum
#define EZ_ERR_POLL	    0x4000  // not all polled steppers found

/* Who:
 * EZStep functions that need to address a stepper (or multiple steppers) use an
 * address 'who', which is an ASCII character between '1' and '_'.
 * To address single motors, it is also possible to use an integer from 1 to 16
 */
// single steppers (NB: in ASCII, these are consecutive)
#define	EZ_WHO_S1     '1'
#define	EZ_WHO_S2     '2'
#define	EZ_WHO_S3     '3'
#define	EZ_WHO_S4     '4'
#define	EZ_WHO_S5     '5'
#define	EZ_WHO_S6     '6'
#define	EZ_WHO_S7     '7'
#define	EZ_WHO_S8     '8'
#define	EZ_WHO_S9     '9'
#define	EZ_WHO_S10    ':'
#define	EZ_WHO_S11    ';'
#define	EZ_WHO_S12    '<'
#define	EZ_WHO_S13    '='
#define	EZ_WHO_S14    '>'
#define	EZ_WHO_S15    '?'
#define	EZ_WHO_S16    '@'
// groups of 2
#define	EZ_WHO_G1_2   'A'
#define	EZ_WHO_G3_4   'C'
#define	EZ_WHO_G5_6   'E'
#define	EZ_WHO_G7_8   'G'
#define	EZ_WHO_G9_10  'I'
#define	EZ_WHO_G11_12 'K'
#define	EZ_WHO_G13_14 'M'
#define	EZ_WHO_G15_16 'O'
// groups of 4
#define	EZ_WHO_G1_4   'Q'
#define	EZ_WHO_G5_8   'U'
#define	EZ_WHO_G9_12  'Y'
#define	EZ_WHO_G13_16 ']'
// all steppers
#define	EZ_WHO_ALL    '_'

/* Chatter:
 * The verbosity level of the EZStep library.
 * The levels are cumulative, printing everything from all lower levels
 */
#define	EZ_CHAT_NONE	0   // print absolutely nothing
#define	EZ_CHAT_ERR	    1   // print only errors and warnings
#define	EZ_CHAT_ACT	    2   // also print bus actions
#define	EZ_CHAT_SEIZE	3   // indicate when the bus is seized
#define EZ_CHAT_BUS	    4   // also print all bus chatter

/* Status:
 * The status of each stepper is by its status bitfield
 */
#define EZ_STEP_USED	0x0001	  /* this stepper is/will be used */
#define EZ_STEP_OK   	0x0002	  /* stepper has been polled, reset to repoll */
#define EZ_STEP_INIT	0x0004	  /* stepper has been initialized */

/* Miscellaneous defines */
#define EZ_BUS_NAME_LEN	    0x100
#define EZ_BUS_BUF_LEN	    0x100
#define EZ_BUS_NACT	    16
#define EZ_BUS_COMM_RETRIES 5
#define EZ_BUS_TIMEOUT_MSEC 1000

/* Number of Communication Errors before triggering a reconnect to the serial port */
#define EZ_ERR_MAX  5

/* These error bits set in ezstep->error increment the error count */
#define EZ_ERR_MASK     0x0600    // EZ_ERR_TIMEOUT and EZ_ERR_TTY

struct ezstep {
  uint16_t  status;                 // status field for each stepper
  char name[EZ_BUS_NAME_LEN];       // name of the stepper

  // parameters for the EZBus move commands
  int vel;			                // velocity (steps/s)
  int acc;			                // acceleration (steps/s/s)
  int ihold;			            // hold current (0-50, % of max)
  int imove;			            // move current (0-100, % of max)
  char preamble[EZ_BUS_BUF_LEN];    // command preamble. Set resolution, etc.
};

struct ezbus {
  struct ezstep stepper[EZ_BUS_NACT];
  int fd;			            // file descriptor for bus serial port
  char name[EZ_BUS_NAME_LEN];	// named prefix to bus-related messages
  char buffer[EZ_BUS_BUF_LEN];  // buffer for responses
  int seized;			        // thread-unsafe concurrency for bus
  int chatter;			        // verbosity of ezstep functions
  int error;			        // most recent error code
  int err_count;		        // number of errors since we last successfully communicated
                                // with ezbus
};

/* initialize a struct ezbus. Needed for all other EZbus funuctions
 * bus: struct ezbus to initialize
 * tty: name of tty device for communications 
 * name: bus name. prepended to messages
 * inhibit_chatter: chatter level
 */
int EZBus_Init(struct ezbus* bus, const char *tty, const char* name, int chatter);

/* Attempt to reset the serial connection to the stepper.
 * Closes and re-opens the port, attempt to reinitialize.
 * Returns 1 if it successfully reset or 0 if the reset failed.
 */
int EZBus_Reset(struct ezbus* bus, const char* tty);

/* add a stepper at address 'who', called 'name' to poll list
 */
int EZBus_Add(struct ezbus* bus, char who, const char* name);

/* simple blocking mechanism for device 'who' to take and release bus
 * if a call to EZBus_IsUsable fails, Take and IsTaken will return EZ_ERR_POLL;
 * IsTaken returns EZ_ERR_OK when who has the bus
 */
int EZBus_Take(struct ezbus* bus, char who);
int EZBus_Release(struct ezbus* bus, char who);
int EZBus_IsTaken(struct ezbus* bus, char who);

/* send command string 'what' to device 'who'
 * For multi-stepper who values, will just use who, rather than looping over
 * individual steppers (useful for sending "R" or "T" all at once)
 */
int EZBus_Send(struct ezbus* bus, char who, const char* what);

/* read bytes from a file descriptor
 * 
 */
int EZBus_Read(int m_port, char *m_buf, size_t m_bytes);
/* 
 * receive response from bus
 */
int EZBus_Recv(struct ezbus* bus);

/* send command 'what' to 'who' and recieve response
 * will retry (every second) under certain error conditions (busy)
 * EZBus_Comm retries EZ_BUS_COMM_RETRIES times
 * EZBus_CommRetry specifies number of retries
 */
int EZBus_Comm(struct ezbus* bus, char who, const char* what);
int EZBus_CommRetry(struct ezbus* bus, char who, const char* what, int retries);

/* send query command 'what' to 'who' to get integer response
 * if successful *val will be assigned the responose, otherwise it is unchanged
 */
int EZBus_ReadInt(struct ezbus* bus, char who, const char* what, int* val);

/* indicate that 'who' should be rescanned next time EZBus_Poll is called
 * when using EZBus_PollInit, will also lead to reinitialization
 * allows multi-stepper who by looping over individual steppers
 */
int EZBus_ForceRepoll(struct ezbus* bus, char who);

/* poll the bus for all steppers on poll list
 * will always skip steppers that are already okay
 */
int EZBus_Poll(struct ezbus* bus);

/* Same as EZBus_Poll, except will call function init for each newfound stepper
 * init should return 0 on failure, take pointer to struct ezbus and who char
 */
int EZBus_PollInit(struct ezbus* bus, int (*ezinit)(struct ezbus*, char));

/* checks stepper status to see if stepper is usable
 * NB: returns boolean values and not an error code
 */
int EZBus_IsUsable(struct ezbus* bus, char who);

/* sends a status query to a stepper to see if it will accept commands
 * EZ_READY will be set if busy (NB: usually that bit means the opposite)
 * other error codes are returned as usual, with EZ_READY also set on error
 * (meaning is reversed so that error states make IsBusy report true)
 */
int EZBus_IsBusy(struct ezbus* bus, char who);


/* Simple motion:
 * Handles basic, generically useful movement commands.
 * More advanced uses are not (yet?) included in this library
 */

/* sets hold current for simple motion moves (in % of max. 0-50)
 */
int EZBus_SetIHold(struct ezbus* bus, char who, int current);

/* sets move current for simple motion moves (in % of max. 0-100)
 */
int EZBus_SetIMove(struct ezbus* bus, char who, int current);

/* sets velocity for simple motion moves (in steps/s)
 */
int EZBus_SetVel(struct ezbus* bus, char who, int vel);

/* sets preamble string for simple motion moves
 * This is primarily for setting microstep resolution, or encoder counts
 */
int EZBus_SetPreamble(struct ezbus* bus, char who, const char* preamble);

/* sets acceleration for simple motion moves (in steps/s/s)
 */
int EZBus_SetAccel(struct ezbus* bus, char who, int acc);

/* Terminate movement
 */
int EZBus_Stop(struct ezbus* bus, char who);

/* Generic function for creating commands strings
 * Creates a command string (in buffer) from printf-style fmt
 * prepends correct preamble parameters for the stepper 
 * Will return empty string for stepper groups
 */
char* __attribute__((format(printf, 5, 6))) EZBus_StrComm(struct ezbus* bus,
    char who, size_t len, char* buffer, const char* fmt, ...);

/* Generic function for sending movement commands. 
 * This will loop properly over stepper groups.
 */
int EZBus_MoveComm(struct ezbus* bus, char who, const char* what);

/* Sets the current warm encoder position.
 * Thould only be used for individual steppers (not groups).
 */
int EZBus_SetEnc(struct ezbus* bus, char who, int enc);

/* Absolute move to 'pos' (measured in steps from "zero")
 */
int EZBus_Goto(struct ezbus* bus, char who, int pos);

/* Absolute move to 'pos' at speed vel (sets stepper.vel)
 */
int EZBus_GotoVel(struct ezbus* bus, char who, int pos, int vel);

/* Relative move by 'delta' (in steps from current location)
 * delta can be positive or negative
 * for infinite moves, send INT_MIN or INT_MAX
 */
int EZBus_RelMove(struct ezbus* bus, char who, int delta);

/* Relative move by 'delta' at speed vel (sets stepper.vel)
 */
int EZBus_RelMoveVel(struct ezbus* bus, char who, int delta, int vel);

/* Continuous move at speed vel (sets stepper.vel)
 * vel can be positive or negative
 */
int EZBus_MoveVel(struct ezbus* bus, char who, int vel);

#endif

