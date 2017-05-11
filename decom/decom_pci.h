/* decom_pci.h: contains definitions for the PCI BLAST Decom module
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
#define DECOM_IOC_RESET         _IO(BBCPCI_IOC_MAGIC, 0)
#define DECOM_IOC_VERSION       _IO(BBCPCI_IOC_MAGIC, 1)
#define DECOM_IOC_COUNTER       _IO(BBCPCI_IOC_MAGIC, 2)
#define DECOM_IOC_FRAMELEN      _IOR(BBCPCI_IOC_MAGIC, 3, unsigned int)
#define DECOM_IOC_LOCKED        _IO(BBCPCI_IOC_MAGIC, 4)
#define DECOM_IOC_NUM_UNLOCKED  _IO(BBCPCI_IOC_MAGIC, 5)
#define DECOM_IOC_FORCE_UNLOCK  _IO(BBCPCI_IOC_MAGIC, 6)
#define DECOM_IOC_FIONREAD      _IO(BBCPCI_IOC_MAGIC, 7)

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

#ifdef __KERNEL__
#	if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
#		if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#			define BBC_DEVICE_CREATE(_cls, _parent, _devt, _drvdata, _fmt, ...) \
				device_create((_cls), (_parent), (_devt), (_drvdata), (_fmt), ##__VA_ARGS__)
#		else
#			define BBC_DEVICE_CREATE(_cls, _parent, _devt, _drvdata, _fmt,  ...) \
				device_create((_cls), (_parent), (_devt), (_fmt), ##__VA_ARGS__)
#		endif
#		define BBC_DEVICE_DESTROY			device_destroy
#		define BBC_DEVICE					struct device
#	else
#		define BBC_DEVICE_CREATE(_cls, _parent, _devt, _drvdata, _fmt, ...) \
			class_device_create((_cls), (_parent), (_devt), (_drvdata), (_fmt), ##__VA_ARGS__)
#		define BBC_DEVICE					struct class_device
#		define BBC_DEVICE_DESTROY			class_device_destroy
#	endif
#endif
