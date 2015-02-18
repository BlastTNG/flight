/* bbc_pci.h: contains definitions for the PCI BLAST Bus Controller module
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/ioctl.h>

#define BBCPCI_IOC_MAGIC 0xbb

/* Define the ioctl commands - use the return value (int) to return the info
 rather than using the arguments.  This is more convenient I think. */

/* reset the controller: clear memory and reset counter to zero */
#define BBCPCI_IOC_RESET        _IO(BBCPCI_IOC_MAGIC, 0)
#define BBCPCI_IOC_SYNC         _IO(BBCPCI_IOC_MAGIC, 1)
#define BBCPCI_IOC_VERSION      _IO(BBCPCI_IOC_MAGIC, 2)
#define BBCPCI_IOC_COUNTER      _IO(BBCPCI_IOC_MAGIC, 3)
#define BBCPCI_IOC_WRITEBUF     _IO(BBCPCI_IOC_MAGIC, 4)
#define BBCPCI_IOC_COMREG       _IO(BBCPCI_IOC_MAGIC, 5)
#define BBCPCI_IOC_READBUF_WP   _IO(BBCPCI_IOC_MAGIC, 6)
#define BBCPCI_IOC_READBUF_RP   _IO(BBCPCI_IOC_MAGIC, 7)
#define BBCPCI_IOC_BBC_FIONREAD _IO(BBCPCI_IOC_MAGIC, 8)
#define BBCPCI_IOC_BI0_FIONREAD _IO(BBCPCI_IOC_MAGIC, 9)
#define BBCPCI_IOC_ON_IRQ       _IO(BBCPCI_IOC_MAGIC, 10)
#define BBCPCI_IOC_OFF_IRQ	_IO(BBCPCI_IOC_MAGIC, 11)
#define BBCPCI_IOC_IRQT_READ	_IO(BBCPCI_IOC_MAGIC, 12)
#define BBCPCI_IOC_IRQ_RATE	_IO(BBCPCI_IOC_MAGIC, 13) /* deprecated */
#define BBCPCI_IOC_RESET_SERIAL _IO(BBCPCI_IOC_MAGIC, 14)
#define BBCPCI_IOC_GET_SERIAL	_IO(BBCPCI_IOC_MAGIC, 15)
#define BBCPCI_IOC_SERIAL_RDY	_IO(BBCPCI_IOC_MAGIC, 16)
#define BBCPCI_IOC_EXT_SER_ON   _IO(BBCPCI_IOC_MAGIC, 17)
#define BBCPCI_IOC_EXT_SER_OFF  _IO(BBCPCI_IOC_MAGIC, 18)
#define BBCPCI_IOC_FRAME_RATE	_IO(BBCPCI_IOC_MAGIC, 19) /* deprecated */
#define BBCPCI_IOC_FRAME_COUNT	_IO(BBCPCI_IOC_MAGIC, 20)
/* on newer firmware, these are preferred to the deprecated versions above */
#define BBCPCI_IOC_IRQ_RATE_INT   _IO(BBCPCI_IOC_MAGIC, 21)
#define BBCPCI_IOC_IRQ_RATE_EXT   _IO(BBCPCI_IOC_MAGIC, 22)
#define BBCPCI_IOC_FRAME_RATE_INT _IO(BBCPCI_IOC_MAGIC, 23)
#define BBCPCI_IOC_FRAME_RATE_EXT _IO(BBCPCI_IOC_MAGIC, 24)

#define BBCPCI_SIZE_UINT           sizeof(unsigned int)
#define BBCPCI_ADD_WRITE_BUF_P     (0x01 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_IR_WRITE_BUF    0x40
#define BBCPCI_ADD_IR_WRITE_PRE    0x36
#define BBCPCI_IR_WRITE_BUF_SIZE   0x100
#define BBCPCI_ADD_COMREG          (0x03 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_IRQREG          (0x0a * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_IRQ_RATE	   (0x0b * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_FRAME_RATE      (0x0e * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_VERSION         (0x00 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_COUNTER         (0x02 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_SERIAL	   (0x0c * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_SERIAL_RDY	   (0x0d * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_FRAME_COUNT	   (0x0f * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_READ_BUF        0x840
#define BBCPCI_ADD_READ_BUF_END    0x2ff0
#define BBCPCI_ADD_READ_BUF_WP     (0x04 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_READ_BUF_RP     (0x05 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_BI0_WP          (0x06 * BBCPCI_SIZE_UINT)
#define BBCPCI_ADD_BI0_RP          (0x07 * BBCPCI_SIZE_UINT)
#define BBCPCI_IR_BI0_BUF          0x3000
#define BBCPCI_IR_BI0_BUF_END      0x3ff8

#define BBCPCI_MAX_FRAME_SIZE      0x10000

#define BBCPCI_SD_WFRAME           0x00
#define BBCPCI_WFRAME_ADD(x)       (BBCPCI_SD_WFRAME + x)

//compatibility hack for code that's looking for two buses
#define BBCPCI_WFRAME1_ADD(x)	    BBCPCI_WFRAME_ADD(x)
#define BBCPCI_SD_WFRAME2          (BBCPCI_SD_WFRAME + BBCPCI_MAX_FRAME_SIZE)
//NB on new firmware, this is start of NIOS read buffer. Writing on startup is
//okay (beginning entries skipped anyway). Writing again can corrupt data!!!
#define BBCPCI_WFRAME2_ADD(x)      (BBCPCI_SD_WFRAME2 + x)

/* Command register bitfield. */
#define BBCPCI_COMREG_RESET       0x00000001 // Fully reset all pointers etc.
#define BBCPCI_COMREG_SYNC        0x00000002 // Clear rx buffers; start frame.
#define BBCPCI_COMREG_ON_IRQ      0x00000004 // Turn on IRQ generation.
#define BBCPCI_COMREG_OFF_IRQ     0x00000008 // Turn off IRQ generation.
#define BBCPCI_COMREG_EXT_SER_ON  0x00000010 // Set serial generation external.
#define BBCPCI_COMREG_EXT_SER_OFF 0x00000020 // Set serial generation internal.

/* Message register bitfield. */
#define BBCPCI_MSGREG_IRQ    0x00000001 /* An IRQ was generated. */

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

#define BBC_NODE(x)     ((unsigned int) (x) << 23)
#define BBC_CH(x)       ((unsigned int) (x) << 16)
#define BBC_ADC_SYNC    (0x20000000)
#define BBC_READ        (0x00400000)
#define BBC_WRITE       (0x40000000)
#define BBC_FSYNC       (0x80000000)
#define BBC_ENDWORD     (0x00000000)
#define BBC_BI0_SYNC    (0x0000eb90)
#define BBC_BI0_ENDWORD (0xffff0000)

#define BBC_ADDRESS(x)  (((unsigned int)(x) & 0xffff0000) >> 16)
#define BBC_DATA(x)     ( (unsigned int)(x) & 0x0000ffff)

#define BBC_NEXT_CHANNEL(x)   ((x) + 0x10000)
#define BI0_MAGIC(x)          (((x) >> 16) & 0x1fff)
#define BI0_TABLE_SIZE        0x2000

#define GET_CH(x)       (((x) >> 16) & 0x3f)
#define GET_NODE(x)     (((x) >> 23) & 0x3f)
#define GET_STORE(x)    (((x) & BBC_WRITE) != 0)
#define GET_READ(x)     (((x) & BBC_READ) != 0)
#define GET_SYNC(x)     (((x) & BBC_ADC_SYNC) != 0)

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
