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
#define BBCPCI_IOC_JIFFIES  _IO(BBCPCI_IOC_MAGIC, 5)
#define BBCPCI_IOC_WRITEBUF _IO(BBCPCI_IOC_MAGIC, 6)
#define BBCPCI_IOC_CBCOUNTER _IO(BBCPCI_IOC_MAGIC, 7)
#define BBCPCI_IOC_COMREG   _IO(BBCPCI_IOC_MAGIC, 8)
#define BBCPCI_IOC_READBUF_WP   _IO(BBCPCI_IOC_MAGIC, 9)
#define BBCPCI_IOC_READBUF_RP   _IO(BBCPCI_IOC_MAGIC, 10)
#define BBCPCI_IOC_WRITEBUF_N   _IO(BBCPCI_IOC_MAGIC, 11)

#define BBCPCI_SIZE_UINT           sizeof(unsigned int)
#define BBCPCI_ADD_WRITE_BUF_P     (0x01 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_IR_WRITE_BUF    0x40
#define BBCPCI_ADD_IR_WRITE_PRE    0x36
#define BBCPCI_IR_WRITE_BUF_SIZE   0x100
#define BBCPCI_ADD_COMREG          (0x03 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_VERSION         (0x00 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_COUNTER         (0x02 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_READ_BUF        0x840
#define BBCPCI_ADD_READ_BUF_END    0x2ff0
#define BBCPCI_ADD_READ_BUF_WP     (0x04 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_READ_BUF_RP     (0x05 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_BI0_WP          0x06 * BBCPCI_SIZE_UINT
#define BBCPCI_ADD_BI0_RP          0x07 * BBCPCI_SIZE_UINT
#define BBCPCI_IR_BI0_BUF          0x3000
#define BBCPCI_IR_BI0_BUF_END      0x3ff8

#define BBCPCI_MAX_FRAME_SIZE      0x10000

#define BBCPCI_SD_WFRAME1          0x00
#define BBCPCI_SD_WFRAME2          (BBCPCI_SD_WFRAME1 + BBCPCI_MAX_FRAME_SIZE)
#define BI0_SD_BUF                  0x800000
#define BBCPCI_WFRAME1_ADD(x)      (BBCPCI_SD_WFRAME1 + x)
#define BBCPCI_WFRAME2_ADD(x)      (BBCPCI_SD_WFRAME2 + x)

/* Command register bitfield. */
#define BBCPCI_COMREG_RESET  0x00000001 /* Fully reset all pointers etc. */
#define BBCPCI_COMREG_SYNC   0x00000002 /* Clear read buffers and start frame */

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

#define BBC_NEXT_CHANNEL(x)   (x + 0x10000)
#define BI0_MAGIC(x)          ((x >> 16) & 0x1fff)
#define BI0_TABLE_SIZE        0x2000

#define GET_CH(x)       ((x >> 16) & 0x3f)
#define GET_NODE(x)     ((x >> 23) & 0x3f)
#define GET_STORE(x)    ((x & BBC_WRITE) != 0)
#define GET_READ(x)     ((x & BBC_READ) != 0)
#define GET_SYNC(x)     ((x & BBC_ADC_SYNC) != 0)

/* timings */
#define BBC_MASTER_CLK         32000000       /* set by the oscilator */
#define BBC_MCLKS_PER_BBC_CLK  8
#define BBC_CLK                (BBC_MASTER_CLK / BBC_MCLKS_PER_BBC_CLK)

#define BBC_ADC_MULTIPLIER     384            /* set by the ADC hardware */
#define BBC_ADC_RATE           (BBC_CLK / BBC_ADC_MULTIPLIER)
#define BBC_ADCS_PER_SAMPLE    104            /* this sets the frame size */
#define BBC_FRAME_RATE         /* = (BBC_ADC_RATE / BBC_ADS_PER_SAMPLE) */ \
                               (BBC_CLK / (BBC_ADC_MULTIPLIER \
                                           * BBC_ADCS_PER_SAMPLE))

#define BBC_WORD_SIZE          (32 * 2)       /* x2 because of tx/rx pair */
#define BBC_WORD_RATE          (BBC_CLK / BBC_WORD_SIZE)
#define BBC_MAX_FRAME_SIZE     /* = (BBC_WORD_RATE / BBC_FRAME_RATE) */ \
                               ((BBC_ADC_MULTIPLIER * BBC_ADCS_PER_SAMPLE) \
                                / BBC_WORD_SIZE)
#define BBC_EFFICIENCY         520 / 624
#define BBC_FRAME_SIZE         /* = (BBC_MAX_FRAME_SIZE * BBC_EFFICIENCY) */ \
                               ((BBC_ADC_MULTIPLIER * BBC_ADCS_PER_SAMPLE \
                                 * BBC_EFFICIENCY) / BBC_WORD_SIZE)

#define BI0_MCLKS_PER_BI0_CLK  32
#define BI0_CLK                (BBC_MASTER_CLK / BI0_MCLKS_PER_BI0_CLK)
#define BI0_WORD_SIZE          16
#define BI0_WORD_RATE          (BI0_CLK / BI0_WORD_SIZE)
#define BI0_FRAME_SIZE         /* = (BBC_WORD_RATE / BI0_FRAME_RATE) */ \
                               ((BBC_MCLKS_PER_BBC_CLK * BBC_ADC_MULTIPLIER \
                                 * BBC_ADCS_PER_SAMPLE) \
                                 / (BI0_MCLKS_PER_BI0_CLK * BI0_WORD_SIZE))
