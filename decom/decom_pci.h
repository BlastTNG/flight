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
#define DECOM_IOC_RESET         _IO(BBCPCI_IOC_MAGIC, 0)
#define DECOM_IOC_VERSION       _IO(BBCPCI_IOC_MAGIC, 1)
#define DECOM_IOC_COUNTER       _IO(BBCPCI_IOC_MAGIC, 2)
#define DECOM_IOC_FRAMELEN      _IOR(BBCPCI_IOC_MAGIC, 3, unsigned int)
#define DECOM_IOC_LOCKED        _IO(BBCPCI_IOC_MAGIC, 4)
#define DECOM_IOC_NUM_UNLOCKED  _IO(BBCPCI_IOC_MAGIC, 5)
#define DECOM_IOC_FORCE_UNLOCK  _IO(BBCPCI_IOC_MAGIC, 6)

#define DECOM_SIZE_UINT           sizeof(unsigned int)

#define DECOM_ADD_READ_BUF        0x100
#define DECOM_ADD_READ_BUF_END    0x2ff0
#define DECOM_ADD_VERSION         (0x00 * DECOM_SIZE_UINT)
#define DECOM_ADD_FRAME_LEN       (0x01 * DECOM_SIZE_UINT)
#define DECOM_ADD_COUNTER         (0x02 * DECOM_SIZE_UINT)
#define DECOM_ADD_COMREG          (0x03 * DECOM_SIZE_UINT)
#define DECOM_ADD_LOCKED          (0x05 * DECOM_SIZE_UINT)
#define DECOM_ADD_NUM_UNLOCKED    (0x06 * DECOM_SIZE_UINT)
#define DECOM_ADD_READ_BUF_WP     (0x07 * DECOM_SIZE_UINT)
#define DECOM_ADD_READ_BUF_RP     (0x08 * DECOM_SIZE_UINT)

#define DECOM_COMREG_RESET        0x00000001
#define DECOM_COMREG_FORCE_UNLOCK 0x00000002

#define DECOM_SYNC                0xeb90
