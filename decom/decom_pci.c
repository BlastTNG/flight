#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/delay.h>

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
#if LINUX_VERSION_CODE >= VERSION_CODE(2,7,0)
# error "This kernel version is not supported"
#endif


static int decom_major = DECOM_MAJOR;

static struct pci_device_id decom_pci_tbl[] = {
 {0x5045, 0x4244, (PCI_ANY_ID), (PCI_ANY_ID), 0, 0, 0},
  {0,}
};
MODULE_DEVICE_TABLE(pci, decom_pci_tbl);

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

static int decom_ioctl(struct inode *inode, struct file *filp,
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
  ioctl:          decom_ioctl,
  open:           decom_open,
  release:        decom_release
};


/* *********************************************************************** */
/* *** this defines the pci layer                                       *** */
/* *********************************************************************** */

static int __devinit decom_pci_init_one(struct pci_dev *pdev, 
				      const struct pci_device_id *ent)
{
  int ret;
 
  ret = pci_enable_device (pdev);
  if(ret) return ret;

  decom_drv.mem_base_raw = pci_resource_start(pdev, 0);
  decom_drv.flags        = pci_resource_flags(pdev, 0);
  decom_drv.len          = pci_resource_len(pdev, 0);
  
  if(!decom_drv.mem_base_raw || ((decom_drv.flags & IORESOURCE_MEM)==0)) {
    printk(KERN_ERR "%s: no I/O resource at PCI BAR #0\n", DRV_NAME);
    return -ENODEV;
  }

  if (check_mem_region(decom_drv.mem_base_raw, decom_drv.len)) {
    printk(KERN_WARNING "%s: memory already in use\n", DRV_NAME);
    return -EBUSY;
  }
  
  request_mem_region(decom_drv.mem_base_raw, decom_drv.len, DRV_NAME);

  decom_drv.mem_base = ioremap_nocache(decom_drv.mem_base_raw, decom_drv.len);

  // Register this as a character device
  if(register_chrdev(decom_major, DRV_NAME, &decom_fops) != 0) {
    printk(KERN_ERR "%s: unable to get major device\n", DRV_NAME);
    return -EIO;
  }
  
  
  decom_drv.timer_on = 0;
  decom_drv.use_count = 0;
  decom_wfifo.status = FIFO_DISABLED;

  printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);
  
  return 0;
}

static void __devexit decom_pci_remove_one(struct pci_dev *pdev)
{
  iounmap(decom_drv.mem_base);
  release_mem_region(decom_drv.mem_base_raw, decom_drv.len);
  
  unregister_chrdev(decom_major, DRV_NAME);

  pci_disable_device(pdev);
  printk(KERN_NOTICE "%s: driver removed\n", DRV_NAME); 
}

static struct pci_driver decom_driver = {
  .name     = DRV_NAME,
  .probe    = decom_pci_init_one,
  .remove   = __devexit_p(decom_pci_remove_one),
  .id_table = decom_pci_tbl,
};

static int __init decom_pci_init(void)
{
  return pci_module_init(&decom_driver);
}

static void __exit decom_pci_cleanup(void)
{
  pci_unregister_driver(&decom_driver);
}


/* Declare module init and exit functions */
module_init(decom_pci_init);
module_exit(decom_pci_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Blast 2004");
MODULE_DESCRIPTION("decom_pci: a driver for the pci dcom card");
MODULE_ALIAS_CHARDEV_MAJOR(DECOM_MAJOR);
MODULE_ALIAS("/dev/decompci");

MODULE_PARM(decom_major, "i");

MODULE_PARM_DESC(decom_major, " decom major number");
