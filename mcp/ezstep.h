/* channels.h: contains channel list specific prototypes and definitions
 *
 * This software is copyright (C) 2002-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef EZSTEP_H
#define EZSTEP_H

/* Error masks and error codes used by EZStepper devices */
#define EZ_ERROR  0x0F
#define EZ_READY  0x20
#define EZ_STATUS 0x40
#define EZ_ERR_OK      0 /* No error */
#define EZ_ERR_INIT    1 /* Initialisation error */
#define EZ_ERR_BADCMD  2 /* Bad command */
#define EZ_ERR_BADOP   3 /* Bad operand */
#define EZ_ERR_COMM    5 /* Communications error */
#define EZ_ERR_NOINIT  7 /* Not initialised */
#define EZ_ERR_OVER    9 /* overload error */
#define EZ_ERR_NOMOVE 11 /* Move Not allowed */
#define EZ_ERR_BUSY   15 /* Command overflow */

/* Error codes used by EZbus library (must be in upper byte) */
#define EZ_BUS_OK       0x0000
#define EZ_BUS_OOD      0x0100
#define EZ_BUS_TIMEOUT  0x0200
#define EZ_BUS_CHECKSUM 0x0400

/* Bitmasks for stepper status */
#define EZ_STEP_USED	0x0001	  /* this stepper is/will be used */
#define EZ_STEP_OKAY 	0x0002	  /* stepper has been polled, no bad errors */

#define EZ_BUS_NAME_LEN	0x100
#define EZ_BUS_BUF_LEN	0x100
#define EZ_BUS_NACT	16

struct ezstep {
  unsigned short status;        //status field for each stepper
  char name[EZ_BUS_NAME_LEN];   //name of the stepper
};

struct ezbus {
  struct ezstep step[EZ_BUS_NACT];
  int fd;			//file descriptor for bus serial port
  char name[EZ_BUS_NAME_LEN];	//named prefix to bus-related messages
  char buffer[EZ_BUS_BUF_LEN];  //buffer for responses
  int seized;			//non-threaded concurrency for bus
  int chatter;			//set to inihibit info messages
				//0 = no messages
				//1 = bus actions
				//2 = bus actions, and communication details
};

/* initialize a struct ezbus. Needed for remaining EZbus funuctions
 * bus: struct ezbus to initialize
 * tty: name of tty device for communications 
 * name: bus name. prepended to messages
 * inhibit_chatter: chatter level
 */
int EZBus_Init(struct ezbus* bus,const char *tty,const char* name,int chatter);

/* add a stepper at address 'who', called 'name' to poll list
 */
void EZBus_Add(struct ezbus* bus, int who, const char* name);

/* simple blocking mechanism for device 'who' to take and release bus
 * who: 0x30+number (1-16) (which is '1' to '9', to start)
 */
int EZBus_Take(struct ezbus* bus, int who);
void EZBus_Release(struct ezbus* bus, int who);

/* send command string 'what' to device 'who'
 */
void EZBus_Send(struct ezbus* bus, int who, const char* what);

/* receive response from bus
 */
int EZBus_Recv(struct ezbus* bus);

/* send command 'what' to 'who' and recieve response
 */
int EZBus_Comm(struct ezbus* bus, int who, const char* what, int naive);

/* send query command 'what' to 'who' and return integer response
 * old: is returned in case of timeout/out-of-data
 */
int EZBus_ReadInt(struct ezbus* bus, int who, const char* what, int old);

/* poll the bus for all steppers on poll list
 * rescan: if true, will skip steppers that are already okay
 */
int EZBus_Poll(struct ezbus* bus, const int rescan);

#endif

