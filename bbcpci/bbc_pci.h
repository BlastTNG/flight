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
#define BBCPCI_IOC_RESET    _IO(BBCPCI_IOC_MAGIC, 0)
#define BBCPCI_IOC_SYNC     _IO(BBCPCI_IOC_MAGIC, 1)
#define BBCPCI_IOC_VERSION  _IO(BBCPCI_IOC_MAGIC, 2)
#define BBCPCI_IOC_COUNTER  _IO(BBCPCI_IOC_MAGIC, 3)
#define BBCPCI_IOC_SECRET   _IOR(BBCPCI_IOC_MAGIC, 4, unsigned char)

#define SIZE_UINT           sizeof(unsigned int)

#define ADD_WRITE_BUF_P     0x01 * SIZE_UINT
#define ADD_IR_WRITE_BUF    0x40
#define IR_WRITE_BUF_SIZE   0x100
#define ADD_COMREG          0x03 * SIZE_UINT
#define ADD_VERSION         0x00 * SIZE_UINT
#define ADD_COUNTER         0x02 * SIZE_UINT
#define ADD_READ_BUF        0x840
#define ADD_READ_BUF_END    0x2ff0
#define ADD_READ_BUF_WP     0x04 * SIZE_UINT
#define ADD_READ_BUF_RP     0x05 * SIZE_UINT

#define MAX_FRAME_SIZE      0x10000

#define SD_WFRAME1          0x00
#define SD_WFRAME2          (SD_WFRAME1 + MAX_FRAME_SIZE)
#define SD_BI0_BUF          0x800000
#define WFRAME1_ADD(x)      (SD_WFRAME1 + x)
#define WFRAME2_ADD(x)      (SD_WFRAME2 + x)
#define BI0_ADD(x)          (SD_BI0_BUF + x)

/* Command register bitfield. */
#define COMREG_RESET    0x00000001  /* Fully reset all pointers etc. */
#define COMREG_SYNC     0x00000002  /* Clear read buffers and start frame */

/* The BBus bitfield looks like:
 * 1      1       1 
 * F      8       0       8       0
 * fwsnnnnnnrccccccdddddddddddddddd
 * 
 * where:
 *      d = data           (16 bits starting at bit  0)
 *      c = channel (0-63)  (6 bits starting at bit 16)
 *      r = read flag       (1 bit           at bit 22)
 *      n = node (0-63)     (6 bits starting at bit 23)
 *      s = ADC sync bit    (1 bit           at bit 29)
 *      w = write flag      (1 bit           at bit 30)
 *      f = frame sync      (1 bit           at bit 31)
 */

#define BBC_NODE(x)     ((unsigned int) x <<23)
#define BBC_CH(x)       ((unsigned int) x <<16)
#define BBC_ADC_SYNC    (0x20000000)
#define BBC_READ        (0x00400000)
#define BBC_WRITE       (0x40000000)
#define BBC_FSYNC       (0x80000000)
#define BBC_ENDWORD     (0x00000000)
#define BBC_BI0_SYNC    (0x0000eb90)
#define BBC_BI0_ENDWORD (0xffff0000)

#define BBC_ADDRESS(x)  (((unsigned int)x & 0xffff0000) >> 16)
#define BBC_DATA(x)     ((unsigned int)x & 0x0000ffff)

#define GET_CH(x)       ((x >> 16) & 0x3f)
#define GET_NODE(x)     ((x >> 23) & 0x3f)
#define GET_STORE(x)    ((x & BBC_WRITE) != 0)
#define GET_READ(x)     ((x & BBC_READ) != 0)
#define GET_SYNC(x)     ((x & BBC_ADC_SYNC) != 0)
