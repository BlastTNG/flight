/***********************************************************************/
/*                                                                     */
/*   Definitions for using the BBCPCI device                              */
/*                                                                     */
/***********************************************************************/
#include <linux/ioctl.h>

#define BBCPCI_IOC_MAGIC 0xbb

/* Define the ioctl commands - use the return value (int) to return the info
 rather than using the arguments.  This is more convenient I think. */

/* reset the controller: clear memory and reset counter to zero */
#define DECOM_IOC_RESET     _IO(BBCPCI_IOC_MAGIC, 0)
#define DECOM_IOC_VERSION   _IO(BBCPCI_IOC_MAGIC, 1)
#define DECOM_IOC_COUNTER   _IO(BBCPCI_IOC_MAGIC, 2)
#define DECOM_IOC_FRAMELEN  _IOR(BBCPCI_IOC_MAGIC, 3, unsigned int)

#define SIZE_UINT           sizeof(unsigned int)

#define ADD_VERSION         0x00 * SIZE_UINT
#define ADD_FRAME_LEN       0x01 * SIZE_UINT
#define ADD_COUNTER         0x02 * SIZE_UINT
#define ADD_COMREG          0x03 * SIZE_UINT
#define ADD_READ_BUF        0x100
#define ADD_READ_BUF_END    0x2ff0
#define ADD_READ_BUF_WP     0x05 * SIZE_UINT
#define ADD_READ_BUF_RP     0x06 * SIZE_UINT

#define COMREG_RESET        0x00000001

#define DECOM_SYNC          0xeb90
