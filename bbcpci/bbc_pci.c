#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "bbc_pci.h"

#define DRV_NAME    "bbc_pci"
#define DRV_VERSION "1.0"
#define DRV_RELDATE ""

#define BBC_MAJOR 250

#ifndef VERSION_CODE
#  define VERSION_CODE(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

#if LINUX_VERSION_CODE < VERSION_CODE(2,6,0)
# error "This kernel is too old and is not supported" 
#endif
#if LINUX_VERSION_CODE >= VERSION_CODE(2,7,0)
# error "This kernel version is not supported"
#endif


static int bbc_major = BBC_MAJOR;
static int bbc_minor = 0;
static int bi0_minor = 1;


static struct pci_device_id bbc_pci_tbl[] = {
  {0x5045, 0x4243, (PCI_ANY_ID), (PCI_ANY_ID), 0, 0, 0},
  {0,}
};
MODULE_DEVICE_TABLE(pci, bbc_pci_tbl);

/*******************************************************************/
/* Definitions                                                     */
/*******************************************************************/


#define FIFO_DISABLED 0
#define FIFO_ENABLED  1
#define FIFO_EMPTY    2
#define FIFO_FULL     4

#define TX_BUFFER_SIZE 0x4000
#define RX_BUFFER_SIZE 0x4000
#define BI0_BUFFER_SIZE (624*25)

extern volatile unsigned long jiffies;
DECLARE_WAIT_QUEUE_HEAD(bbc_read_wq);


static struct {
  unsigned long mem_base_raw;
  void          *mem_base;
  unsigned long flags;
  unsigned long len;
  struct timer_list timer;

  int use_count;
  int timer_on;
} bbc_drv;

#define BBC_WFIFO_SIZE (2*TX_BUFFER_SIZE)
static struct {
  volatile int i_in;
  volatile int i_out;
  unsigned data[BBC_WFIFO_SIZE];
  volatile int status;
  atomic_t n;
} bbc_wfifo;

#define BBC_RFIFO_SIZE (RX_BUFFER_SIZE)
static struct {
  volatile int i_in;
  volatile int i_out;
  unsigned data[BBC_RFIFO_SIZE];
  volatile int status;
  atomic_t n;
} bbc_rfifo;

#define BI0_WFIFO_SIZE (BI0_BUFFER_SIZE)
static struct {
  volatile int i_in;
  volatile int i_out;
  unsigned short data[BI0_WFIFO_SIZE];
  volatile int status;
  atomic_t n;
} bi0_wfifo;





int bbc_read_procmem(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
  int len;
  int limit;


  len = 0;
  limit = count - 80;

  if( (limit - len) >= 80)
    len += sprintf(buf+len, "           %10s %10s %10s\n", 
		   "i_in", "i_out", "n");
  
  if( (limit - len) >= 80)
    len += sprintf(buf+len, "bbc_wfifo: %10d %10d %10d\n", 
		   bbc_wfifo.i_in, bbc_wfifo.i_out, 
		   atomic_read(&bbc_wfifo.n));

  if( (limit - len) >= 80)
    len += sprintf(buf+len, "bbc_rfifo: %10d %10d %10d\n", 
		   bbc_rfifo.i_in, bbc_rfifo.i_out, 
		   atomic_read(&bbc_rfifo.n));
  
  if( (limit - len) >= 80)
    len += sprintf(buf+len, "bi0_wfifo: %10d %10d %10d\n", 
		   bi0_wfifo.i_in, bi0_wfifo.i_out, 
		   atomic_read(&bi0_wfifo.n));
  
  *eof = 1;
  return len;
}




static void timer_callback(unsigned long dummy)
{
  static loff_t wp, rp;
  static int nwritten;
  static unsigned short out_data[2];
  static int idx = 2;

/*
  static int firstime = 1;
  static int all = 0; 
  static unsigned long old = 0;

  if(firstime) {
    firstime = 0;
    old = jiffies;
  }

  all++; 
  if(all == 1) {  
    printk(KERN_EMERG "Ciaooooo------------------------------ %d %ld %x %x\n", 
        all, (jiffies- old), bbc_wfifo.n, bi0_wfifo.n); 
    old = jiffies; 
    all = 0; 
  } 
*/
  
  /* Check if NIOS is busy resetting itself */
  if(readl(bbc_drv.mem_base + BBCPCI_ADD_COMREG)) 
    goto tc_end;

  /* Read data from NIOS */
  if(bbc_rfifo.status & FIFO_ENABLED) {
    while(atomic_read(&bbc_rfifo.n) < BBC_RFIFO_SIZE)  {
      wp = readl(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_WP);
      rp = readl(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_RP);      
      rp += BBCPCI_SIZE_UINT;
      if(rp >= BBCPCI_ADD_READ_BUF_END) rp = BBCPCI_ADD_READ_BUF;
      if(rp == wp) break;

      bbc_rfifo.data[bbc_rfifo.i_in] = readl(bbc_drv.mem_base + rp);
      
      if(bbc_rfifo.i_in == (BBC_RFIFO_SIZE - 1)) {
	bbc_rfifo.i_in = 0;
      } else {
	bbc_rfifo.i_in++;
      }
      
      writel(rp, bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_RP);
      atomic_inc(&bbc_rfifo.n);
    }
    if(atomic_read(&bbc_rfifo.n))
      wake_up_interruptible(&bbc_read_wq);
  }

  /* Write Blast data to NIOS */
  if(bbc_wfifo.status & FIFO_ENABLED) {
    wp = readl(bbc_drv.mem_base + BBCPCI_ADD_WRITE_BUF_P);
    if(wp < BBCPCI_ADD_IR_WRITE_BUF) {

      nwritten = 0;
      while( (nwritten < BBCPCI_IR_WRITE_BUF_SIZE) && atomic_read(&bbc_wfifo.n) ) {
        wp += 2*BBCPCI_SIZE_UINT;
        writel(bbc_wfifo.data[bbc_wfifo.i_out+1], bbc_drv.mem_base + wp + BBCPCI_SIZE_UINT);
        writel(bbc_wfifo.data[bbc_wfifo.i_out], bbc_drv.mem_base + wp);
        if(bbc_wfifo.i_out == (BBC_WFIFO_SIZE - 2)) {
          bbc_wfifo.i_out = 0;
        } else {
          bbc_wfifo.i_out += 2;
        }
        nwritten++;
	atomic_sub(2, &bbc_wfifo.n);
      }

      if(nwritten) {
        writel(wp, bbc_drv.mem_base + BBCPCI_ADD_WRITE_BUF_P);
      }
    }
  } 

  /* Write bi-phase data to NIOS */
  if(bi0_wfifo.status & FIFO_ENABLED) {
    rp = readl(bbc_drv.mem_base + BBCPCI_ADD_BI0_RP);
    while( atomic_read(&bi0_wfifo.n) ) {
      wp = readl(bbc_drv.mem_base + BBCPCI_ADD_BI0_WP);
      if(wp == rp) break;
      out_data[--idx] = bi0_wfifo.data[bi0_wfifo.i_out];
      if(bi0_wfifo.i_out == (BI0_WFIFO_SIZE - 1)) {
        bi0_wfifo.i_out = 0;
      } else {
        bi0_wfifo.i_out++;
      }
      if(idx == 0) {
        idx = 2;
        writel(*(unsigned *)out_data, bbc_drv.mem_base + wp);

        if (wp >= BBCPCI_IR_BI0_BUF_END) {
          wp = BBCPCI_IR_BI0_BUF;
        } else {
          wp += BBCPCI_SIZE_UINT;
        }

        writel(wp, bbc_drv.mem_base + BBCPCI_ADD_BI0_WP);
      }
      atomic_dec(&bi0_wfifo.n);
    }
  }

 tc_end:
  /* Reset timer callback and exit */
  bbc_drv.timer.expires = jiffies + 1;
  add_timer(&bbc_drv.timer);
}


static ssize_t bbc_read(struct file *filp, char __user *buf, 
    size_t count, loff_t *dummy) 
{
  int minor;
  size_t i;
  size_t to_read;
  void *out_buf = (void *)buf;

  if( !access_ok(VERIFY_WRITE, (void *)buf, count) ) {
    printk(KERN_WARNING "%s: (read)  error accessing user space memory\n", DRV_NAME);
    return -EFAULT;
  }

  minor = *(int *)filp->private_data;

  if (minor == bbc_minor) {

    if(count < BBCPCI_SIZE_UINT) return 0;
    to_read = count / BBCPCI_SIZE_UINT;

    /* What to do if no data are available ? */
    if(atomic_read(&bbc_rfifo.n) == 0) {
      if(filp->f_flags & O_NONBLOCK) {
	/* Non blocking read, return immediately */
	return -EAGAIN;
      } else {
	/* Blocking read: sleep untill data are available */
	interruptible_sleep_on(&bbc_read_wq);
      }
    }

    for(i = 0; i < to_read; i++) {
      if(atomic_read(&bbc_rfifo.n) == 0) break;
      
      __copy_to_user(out_buf, &bbc_rfifo.data[bbc_rfifo.i_out], BBCPCI_SIZE_UINT);
      out_buf += BBCPCI_SIZE_UINT;

      if(bbc_rfifo.i_out == (BBC_RFIFO_SIZE - 1)) {
	bbc_rfifo.i_out = 0;
      } else {
	bbc_rfifo.i_out++;
      }
      
      atomic_dec(&bbc_rfifo.n);
    }
    
    return i*BBCPCI_SIZE_UINT;

  } else if (minor == bi0_minor) {
    return 0;
  }

  return 0;
}

static ssize_t bbc_write(struct file *filp, const char __user *buf, 
    size_t count, loff_t *dummy) 
{
  int minor;
  size_t i;
  size_t to_write;
  void *in_buf = (void *)buf;


  if( !access_ok(VERIFY_READ, (void *)buf, count) ) {
    printk(KERN_WARNING "%s: (write) error accessing user space memory\n", DRV_NAME);
    return -EFAULT;
  }

  minor = *(int *)filp->private_data;

  if (minor == bbc_minor) {

    if(count < 2*BBCPCI_SIZE_UINT) return 0;
    to_write = count / (2*BBCPCI_SIZE_UINT);

    for(i = 0; i < to_write; i++) {
      
      if(atomic_read(&bbc_wfifo.n) == BBC_WFIFO_SIZE) {
        printk(KERN_WARNING "%s: bbc buffer overrun. size = %x\n", 
            DRV_NAME, atomic_read(&bbc_wfifo.n));
        break;
      }

      __copy_from_user((void *)&bbc_wfifo.data[bbc_wfifo.i_in], 
          in_buf, 2*BBCPCI_SIZE_UINT);

      if( bbc_wfifo.i_in == (BBC_WFIFO_SIZE - 2) ) {
        bbc_wfifo.i_in = 0;
      } else {
        bbc_wfifo.i_in += 2;
      }
    
      in_buf += 2*BBCPCI_SIZE_UINT;
      atomic_add(2, &bbc_wfifo.n);
    
    }

    return (2*i*BBCPCI_SIZE_UINT);

  } else if (minor == bi0_minor) {
    if(count < sizeof(unsigned short) ) return 0;
    to_write = count / sizeof(unsigned short);
    
    for(i = 0; i < to_write; i++) {

      if(atomic_read(&bi0_wfifo.n) == BI0_WFIFO_SIZE) {
        printk(KERN_WARNING "%s: bi0 buffer overrun. size = %x\n", 
            DRV_NAME, atomic_read(&bi0_wfifo.n));
        break;
      }

      __copy_from_user( (void *)&bi0_wfifo.data[bi0_wfifo.i_in],
          in_buf, sizeof(unsigned short) );

      if( bi0_wfifo.i_in == (BI0_WFIFO_SIZE - 1) ) {
        bi0_wfifo.i_in = 0;
      } else {
        bi0_wfifo.i_in++;
      }

      in_buf += sizeof(unsigned short);
      atomic_inc(&bi0_wfifo.n);

    }

    return i*sizeof(unsigned short);

  } 

  return 0;
}

static int bbc_ioctl(struct inode *inode, struct file *filp,
    unsigned int cmd, unsigned long arg)
{
  int ret = 0;

  switch(cmd) {
  case BBCPCI_IOC_RESET:    /* Reset the BBC board - clear fifo, registers */
    writel(BBCPCI_COMREG_RESET, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_SYNC:     /* Clear read buffers and restart frame. */
    writel(BBCPCI_COMREG_SYNC, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_VERSION:  /* Get the current version. */
    ret = readl(bbc_drv.mem_base + BBCPCI_ADD_VERSION);
    break;
  case BBCPCI_IOC_COUNTER:  /* Get the counter. */
    ret = readl(bbc_drv.mem_base + BBCPCI_ADD_COUNTER);
    break;
  case BBCPCI_IOC_WRITEBUF:  /* How many words in the NIOS write buf? */
    ret  = readl(bbc_drv.mem_base + BBCPCI_ADD_WRITE_BUF_P);
    ret -= BBCPCI_ADD_IR_WRITE_BUF;
    break;
  case BBCPCI_IOC_COMREG:  /* Return the command register. */
    ret = readl(bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_READBUF_WP: /* Where nios is about to write. */
    ret = readl(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_WP);
    break;
  case BBCPCI_IOC_READBUF_RP: /* Where the PC is about to read. */
    ret = readl(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_RP);
    break;
  case BBCPCI_IOC_BBC_FIONREAD:
    ret = atomic_read(&bbc_wfifo.n);
    break;
  case BBCPCI_IOC_BI0_FIONREAD:
    ret = atomic_read(&bi0_wfifo.n);
    break;
  default:
    break;
  }

  return ret;
}

static int bbc_open(struct inode *inode, struct file *filp) 
{
  int minor = MINOR(inode->i_rdev);

  filp->private_data = (minor == bbc_minor) ? &bbc_minor : &bi0_minor;

  if(minor == bbc_minor) {
    if( (bbc_wfifo.status & FIFO_ENABLED) || (bbc_rfifo.status & FIFO_ENABLED) ) return -ENODEV;
    filp->private_data = &bbc_minor;

    bbc_wfifo.i_in = bbc_wfifo.i_out = 0;
    atomic_set(&bbc_wfifo.n, 0);
    bbc_wfifo.status |= FIFO_ENABLED;

    bbc_rfifo.i_in = bbc_rfifo.i_out = 0;
    atomic_set(&bbc_rfifo.n, 0);
    bbc_rfifo.status |= FIFO_ENABLED;

    bbc_drv.use_count++;
  }

  if(minor == bi0_minor) {
    if(bi0_wfifo.status & FIFO_ENABLED) return -ENODEV;
    filp->private_data = &bi0_minor;

    bi0_wfifo.i_in = bi0_wfifo.i_out = 0;
    atomic_set(&bi0_wfifo.n, 0);
    bi0_wfifo.status |= FIFO_ENABLED;

    bbc_drv.use_count++;
  }

  if(bbc_drv.use_count && bbc_drv.timer_on == 0) {
    // sync bbc
    writel(BBCPCI_COMREG_SYNC, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    // enable timer
    bbc_drv.timer_on = 1;
    init_timer(&bbc_drv.timer);
    bbc_drv.timer.function = timer_callback;
    bbc_drv.timer.expires = jiffies + 1;
    add_timer(&bbc_drv.timer);
  }

  return 0;
}

static int bbc_release(struct inode *inode, struct file *filp)
{
  int minor = MINOR(inode->i_rdev);

  if (minor == bbc_minor) {
    bbc_wfifo.status = FIFO_DISABLED;
    bbc_rfifo.status = FIFO_DISABLED;
    bbc_drv.use_count--;
  }
  if (minor == bi0_minor) {
    bi0_wfifo.status = FIFO_DISABLED;
    bbc_drv.use_count--;
  }

  if(bbc_drv.use_count == 0 && bbc_drv.timer_on) {
    bbc_drv.timer_on = 0;
    del_timer_sync(&bbc_drv.timer);

    // Reset bbc 
    writel(BBCPCI_COMREG_RESET, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
  }

  return 0;
}


static struct file_operations bbc_fops = {
owner:          THIS_MODULE,
read:           bbc_read,
write:          bbc_write,
ioctl:          bbc_ioctl,
open:           bbc_open,
release:        bbc_release
};


/* *********************************************************************** */
/* *** this define the pci layer                                       *** */
/* *********************************************************************** */

static int __devinit bbc_pci_init_one(struct pci_dev *pdev, 
    const struct pci_device_id *ent)
{
  int ret;

  ret = pci_enable_device (pdev);
  if(ret) return ret;

  bbc_drv.mem_base_raw = pci_resource_start(pdev, 0);
  bbc_drv.flags        = pci_resource_flags(pdev, 0);
  bbc_drv.len          = pci_resource_len(pdev, 0);

  if(!bbc_drv.mem_base_raw || ((bbc_drv.flags & IORESOURCE_MEM)==0)) {
    printk(KERN_ERR "%s: no I/O resource at PCI BAR #0\n", DRV_NAME);
    return -ENODEV;
  }

  if (check_mem_region(bbc_drv.mem_base_raw, bbc_drv.len)) {
    printk(KERN_WARNING "%s: bbc_pci: memory already in use\n", DRV_NAME);
    return -EBUSY;
  }

  request_mem_region(bbc_drv.mem_base_raw, bbc_drv.len, DRV_NAME);

  bbc_drv.mem_base = ioremap_nocache(bbc_drv.mem_base_raw, bbc_drv.len);

  // Register this as a character device
  if(register_chrdev(bbc_major, DRV_NAME, &bbc_fops) != 0) {
    printk(KERN_ERR "%s: unable to get major device\n", DRV_NAME);
    return -EIO;
  }

  /* Create /proc entry */
  create_proc_read_entry(DRV_NAME, 
			 0    /* default mode */,
			 NULL /* parent dir */, 
			 bbc_read_procmem, 
			 NULL /* client data */);

  // Reset bbc
  writel(BBCPCI_COMREG_RESET, bbc_drv.mem_base + BBCPCI_ADD_COMREG);

  bbc_drv.timer_on = 0;
  bbc_drv.use_count = 0;
  bbc_wfifo.status = FIFO_DISABLED;
  bbc_rfifo.status = FIFO_DISABLED;
  bi0_wfifo.status = FIFO_DISABLED;

  printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);

  return 0;
}

static void __devexit bbc_pci_remove_one(struct pci_dev *pdev)
{
  iounmap(bbc_drv.mem_base);
  release_mem_region(bbc_drv.mem_base_raw, bbc_drv.len);

  unregister_chrdev(bbc_major, DRV_NAME);

  pci_disable_device(pdev);
  printk(KERN_NOTICE "%s: driver removed\n", DRV_NAME); 
}

static struct pci_driver bbc_driver = {
  .name     = DRV_NAME,
  .probe    = bbc_pci_init_one,
  .remove   = __devexit_p(bbc_pci_remove_one),
  .id_table = bbc_pci_tbl,
};

static int __init bbc_pci_init(void)
{
  return pci_module_init(&bbc_driver);
}

static void __exit bbc_pci_cleanup(void)
{
  pci_unregister_driver(&bbc_driver);
}


/* Declare module init and exit functions */
module_init(bbc_pci_init);
module_exit(bbc_pci_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Blast 2004");
MODULE_DESCRIPTION("bbc_pci: a driver for the pci Blast Bus Controller");
MODULE_ALIAS_CHARDEV_MAJOR(BBC_MAJOR);
MODULE_ALIAS("/dev/bbcpci");
MODULE_ALIAS("/dev/bi0_pci");

MODULE_PARM(bbc_major, "i");
MODULE_PARM(bbc_minor, "i");
MODULE_PARM(bi0_minor, "i");

MODULE_PARM_DESC(bbc_major, " bbc major number");
MODULE_PARM_DESC(bbc_minor, " bbc minor number");
MODULE_PARM_DESC(bi0_minor, " bi0 minor number");
