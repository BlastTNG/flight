/* decom_pci: decom kernel driver for the PCI BLAST Decom Card
 * 
 * decom_pci is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * decom_pci is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with decom_pci; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>

#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "decom_pci.h"

#define DRV_NAME    "decom_pci"
#define DRV_VERSION "1.0"
#define DRV_RELDATE ""

#define DECOM_MAJOR 251

#ifndef VERSION_CODE
#  define VERSION_CODE(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

#if LINUX_VERSION_CODE < VERSION_CODE(2,6,0)
# error "This kernel is too old and is not supported" 
#endif
//#if LINUX_VERSION_CODE >= VERSION_CODE(2,7,0)
//# error "This kernel version is not supported"
//#endif


int decom_major = 0;

static struct pci_device_id decom_pci_tbl[] = {
 {0x5045, 0x4244, (PCI_ANY_ID), (PCI_ANY_ID), 0, 0, 0},
  {0,}
};
MODULE_DEVICE_TABLE(pci, decom_pci_tbl);

#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,15)
static struct class *decom_class;
#endif

/*******************************************************************/
/* Definitions                                                     */
/*******************************************************************/


#define FIFO_DISABLED 0
#define FIFO_ENABLED  1

#define RX_BUFFER_SIZE 0x10000

extern volatile unsigned long jiffies;

static struct {
  unsigned long mem_base_raw;
  void          *mem_base;
  unsigned long flags;
  unsigned long len;
  struct timer_list timer;

  int use_count;
  int timer_on;
} decom_drv;

#define DECOM_WFIFO_SIZE (RX_BUFFER_SIZE)
static struct {
  volatile int i_in;
  volatile int i_out;
  unsigned short data[DECOM_WFIFO_SIZE];
  volatile int status;
  atomic_t n;
} decom_wfifo;



static void timer_callback(unsigned long dummy)
{
  static loff_t wp, rp;
  static unsigned short in_data;

/*   static int all = 0; */
/*   static unsigned long old = 0; */

/*   all++; */
/*   if(all == 1000) {  */
/*     printk(KERN_EMERG "Ciaooooo------------------------------ %d %ld %x\n", */
/* 	   all, (jiffies- old), bbc_wfifo.n); */
/*         old = jiffies; */
/*     all = 0; */
/*   } */

  wp = readl(decom_drv.mem_base + DECOM_ADD_READ_BUF_WP); // Where NIOS is about to write.
  
  while(1) {
    if(atomic_read(&decom_wfifo.n) == DECOM_WFIFO_SIZE) {
      printk(KERN_WARNING "%s: overrun on receiving buffer\n", DRV_NAME);
      break;
    }

    rp = readl(decom_drv.mem_base + DECOM_ADD_READ_BUF_RP); // Where pci read last.
    rp += DECOM_SIZE_UINT;
    if( rp >= DECOM_ADD_READ_BUF_END ) rp = DECOM_ADD_READ_BUF;
    if (rp == wp) break;

    in_data = readl(decom_drv.mem_base + rp);
    writel(rp, decom_drv.mem_base + DECOM_ADD_READ_BUF_RP);

    decom_wfifo.data[decom_wfifo.i_in] = 
      (unsigned short)(in_data & 0x0000ffffL);

    if(decom_wfifo.i_in == (DECOM_WFIFO_SIZE - 1)) {
      decom_wfifo.i_in = 0;
    } else {
      decom_wfifo.i_in++;
    }

    atomic_inc(&decom_wfifo.n); 
  }


  decom_drv.timer.expires = jiffies + 1;
  add_timer(&decom_drv.timer);
}


static ssize_t decom_read(struct file *filp, char __user *buf, 
			size_t count, loff_t *dummy) 
{
  size_t to_read;
  size_t available;
  unsigned short out_data;
  unsigned short *bufs;
  int i;

  to_read = count / sizeof(unsigned short);
  available = atomic_read(&decom_wfifo.n);

  if(to_read > available) to_read = available;

  if( to_read == 0 ) return 0;
 
  bufs = (unsigned short *)buf;
  for(i = 0; i < to_read; i++) {
    out_data = decom_wfifo.data[decom_wfifo.i_out];
    if(put_user(out_data, bufs)) return -EFAULT;
    if( decom_wfifo.i_out == (DECOM_WFIFO_SIZE - 1) ) {
      decom_wfifo.i_out = 0;
    } else {
      decom_wfifo.i_out++;
    }
    bufs++;
    atomic_dec(&decom_wfifo.n);
  }

  return i*sizeof(unsigned short);
}

static ssize_t decom_write(struct file *filp, const char __user *buf, 
			 size_t count, loff_t *dummy) 
{
  return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long decom_ioctl(struct file *filp,
#else
static int decom_ioctl(struct inode *inode, struct file *filp,
#endif
                     unsigned int cmd, unsigned long arg)
{
  int ret = 0;
  int size = _IOC_SIZE(cmd);

  if (_IOC_DIR(cmd) & _IOC_READ) {
    if (!access_ok(VERIFY_WRITE, (void *)arg, size))
      return -EFAULT;
  }
  else if (_IOC_DIR(cmd) & _IOC_WRITE) {
    if (!access_ok(VERIFY_READ, (void *)arg, size))
      return -EFAULT;
  }
  
  switch(cmd) {
    case DECOM_IOC_RESET:
      writel(DECOM_COMREG_RESET, decom_drv.mem_base + DECOM_ADD_COMREG);
      break;
    case DECOM_IOC_VERSION:
      ret = readl(decom_drv.mem_base + DECOM_ADD_VERSION);
      break;
    case DECOM_IOC_COUNTER:
      ret = readl(decom_drv.mem_base + DECOM_ADD_COUNTER);
      break;
    case DECOM_IOC_FRAMELEN:
      writel(arg, decom_drv.mem_base + DECOM_ADD_FRAME_LEN);
      ret = arg;
      break;
    case DECOM_IOC_LOCKED:
      ret = readl(decom_drv.mem_base + DECOM_ADD_LOCKED);
      break;
    case DECOM_IOC_NUM_UNLOCKED:
      ret = readl(decom_drv.mem_base + DECOM_ADD_NUM_UNLOCKED);
      break;
    case DECOM_IOC_FORCE_UNLOCK:
      writel(DECOM_COMREG_FORCE_UNLOCK, decom_drv.mem_base + DECOM_ADD_COMREG);
      break;
    case DECOM_IOC_FIONREAD:
      ret = atomic_read(&decom_wfifo.n);
      break;
    default:
      break;
  }

  return ret;
}

static int decom_open(struct inode *inode, struct file *filp) 
{

  if(decom_wfifo.status & FIFO_ENABLED) return -ENODEV;
  
  decom_wfifo.i_in = decom_wfifo.i_out = 0;
  atomic_set(&decom_wfifo.n, 0);
  decom_wfifo.status = FIFO_ENABLED;
  
  decom_drv.use_count++;
  
   
  if(decom_drv.use_count && decom_drv.timer_on == 0) {
    // enable timer
    decom_drv.timer_on = 1;
    init_timer(&decom_drv.timer);
    decom_drv.timer.function = timer_callback;
    decom_drv.timer.expires = jiffies + 1;
    add_timer(&decom_drv.timer);
  }
  
  return 0;
}

static int decom_release(struct inode *inode, struct file *filp)
{
  decom_wfifo.status = FIFO_DISABLED;
  decom_drv.use_count--;
  
  
  if(decom_drv.use_count == 0 && decom_drv.timer_on) {
    decom_drv.timer_on = 0;
    del_timer_sync(&decom_drv.timer);
  }
  
  return 0;
}


static struct file_operations decom_fops = {
  owner:          THIS_MODULE,
  read:           decom_read,
  write:          decom_write,
#ifdef HAVE_UNLOCKED_IOCTL
  unlocked_ioctl: decom_ioctl,
#else
  ioctl:          decom_ioctl,
#endif
  open:           decom_open,
  release:        decom_release,
  llseek:		  no_llseek
};


/* *********************************************************************** */
/* *** this defines the pci layer                                       *** */
/* *********************************************************************** */

static int decom_pci_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret;
# if LINUX_VERSION_CODE >= VERSION_CODE(2,6,15)
	BBC_DEVICE *class_err;
# endif

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	decom_drv.mem_base_raw = pci_resource_start(pdev, 0);
	decom_drv.flags = pci_resource_flags(pdev, 0);
	decom_drv.len = pci_resource_len(pdev, 0);

	if (!pci_resource_len(pdev, 0) || !(pci_resource_flags(pdev, 0) & IORESOURCE_MEM))
	{
		printk(KERN_ERR "%s: no I/O resource at PCI BAR #0\n", DRV_NAME);
		ret = -ENODEV;
		goto out_enable;
	}

	/* Register the I/O memory regions for the device given by the DRV_NAME parameter (the device name). */
	if (pci_request_regions(pdev, DRV_NAME))
	{
		printk(KERN_ERR "%s: io resource busy\n", DRV_NAME);
		ret = -EAGAIN;
		goto out_enable;
	}

	decom_drv.mem_base = ioremap_nocache(decom_drv.mem_base_raw, decom_drv.len);
	if (decom_drv.mem_base == NULL)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Could not map memory from card\n", DRV_NAME);
		goto out_request;
	}

	// Interface with SYSFS/udev
# if LINUX_VERSION_CODE >= VERSION_CODE(2,6,15)
	class_err = BBC_DEVICE_CREATE(decom_class, NULL, MKDEV(decom_major, 0), NULL, "decom_pci");
	if (IS_ERR(class_err))
	{
		ret = PTR_ERR(class_err);
		goto out_class;
	}
# endif

	decom_drv.timer_on = 0;
	decom_drv.use_count = 0;
	decom_wfifo.status = FIFO_DISABLED;

	printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);

	ret = 0;
	goto out;

# if LINUX_VERSION_CODE >= VERSION_CODE(2,6,15)
out_class:
	BBC_DEVICE_DESTROY(decom_class, MKDEV(decom_major, 0));
# endif
	iounmap(decom_drv.mem_base);

out_request:
	pci_release_regions(pdev);
out_enable:
	pci_disable_device(pdev);
out:
	return ret;
}

static void decom_pci_remove_one(struct pci_dev *pdev)
{
# if LINUX_VERSION_CODE >= VERSION_CODE(2,6,15)
	BBC_DEVICE_DESTROY(decom_class, MKDEV(decom_major, 0));
# endif

	pci_iounmap(pdev, decom_drv.mem_base);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static struct pci_driver decom_driver = {
  .name     = DRV_NAME,
  .probe    = decom_pci_init_one,
  .remove   = decom_pci_remove_one,
  .id_table = decom_pci_tbl,
};

static int __init decom_pci_init(void)
{
	int error;

	/* Register this as a character device. */
	if((decom_major = register_chrdev(0, DRV_NAME, &decom_fops)) < 0)
	{
		printk(KERN_WARNING "%s: unable to get major device\n", DRV_NAME);
		error = decom_major;
		goto out;
	}

	decom_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(decom_class))
	{
		printk(KERN_ERR "%s: failed to create sysfs class\n", DRV_NAME);
		error = PTR_ERR(decom_class);
		goto chr_remove;
	}

	/* Register the PCI driver functions */
	error =	pci_register_driver(&decom_driver);
	if (error)
	{
		printk(KERN_ERR "%s: unable to register driver\n", DRV_NAME);
		goto class_destroy;
	}


	printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);
	return 0;

class_destroy:
	class_destroy(decom_class);
chr_remove:
	unregister_chrdev(decom_major, DRV_NAME);
out:
	return error;
}

static void __exit decom_pci_cleanup(void)
{
	pci_unregister_driver(&decom_driver);
	unregister_chrdev(decom_major, DRV_NAME);
	class_destroy(decom_class);
	printk(KERN_NOTICE "%s: driver removed\n", DRV_NAME);
}


/* Declare module init and exit functions */
module_init(decom_pci_init);
module_exit(decom_pci_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Blast 2004, EBEX 2011");
MODULE_DESCRIPTION("decom_pci: a driver for the pci dcom card");
MODULE_ALIAS("/dev/decom_pci");

module_param(decom_major, int, 0444);

MODULE_PARM_DESC(decom_major, " decom major number");
