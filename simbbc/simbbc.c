/* simbbc: kernel driver simulating the BLAST Bus and controller
 *
 * simbbc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * simbbc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with bbc_pci; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA 
 * Or visit http://www.gnu.org/licenses/
 *
 * Written by Enzo Pascale, Sept. 2 2009
 * Updated University of Toronto, 2011
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/sched.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/msr.h>
#include <asm/uaccess.h>

#include "bbc_pci.h"
#include "simbbc.h"
#include "simdata.h"

#define DRV_NAME    "simbbc_pci"
#define DRV_VERSION "1.1"
#define DRV_RELDATE ""

#ifndef VERSION_CODE
#  define VERSION_CODE(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

#if LINUX_VERSION_CODE < VERSION_CODE(2,6,0)
# error "This kernel is too old and is not supported" 
#endif
#if LINUX_VERSION_CODE > VERSION_CODE(3,7,1)
# error "This kernel version is not supported"
#endif
//incompatible API change, may have changed before 2.6.27
#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,27)
#define USE_NEWER_DEVICE_CREATE
#endif

static int bbc_minor = 0;

#define FIFO_DISABLED 0
#define FIFO_ENABLED  1
#define FIFO_EMPTY    2
#define FIFO_FULL     4

#define BUFFER_SIZE  (2*BBCPCI_MAX_FRAME_SIZE)

DECLARE_WAIT_QUEUE_HEAD(bbc_read_wq);

static struct {
  struct timer_list timer;

  int use_count;
  int timer_on;
  struct cdev bbc_cdev;
} bbc_drv;

#define BBC_RFIFO_SIZE (BUFFER_SIZE)
static struct {
  volatile int i_in;
  volatile int i_out;
  unsigned data[BBC_RFIFO_SIZE];
  volatile int status;
  atomic_t n;
} bbc_rfifo;


static struct class *bbcpci_class;
struct device *bbcpci_d;

static long int bbc_compat_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg);
static int bbc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
                     unsigned long arg);
static int bbcpci_start_sysfs(void);
static void bbcpci_remove_sysfs(void);

//------------------------------------------------------------------------------
// Procmem.
//------------------------------------------------------------------------------

int bbc_read_procmem(char *buf, char **start, off_t offset, int count, int *eof,
                     void *data) {
  int len;
  int limit;

  len = 0;
  limit = count - 80;

  if( (limit - len) >= 80)
    len += sprintf(buf+len, "           %10s %10s %10s %10s\n", 
		   "i_in", "i_out", "n", "status");

  if( (limit - len) >= 80)
    len += sprintf(buf+len, "bbc_rfifo: %10d %10d %10d 0x%08x\n", 
		   bbc_rfifo.i_in, bbc_rfifo.i_out, 
		   atomic_read(&bbc_rfifo.n), bbc_rfifo.status);
  
  *eof = 1;
  return len;
}


//------------------------------------------------------------------------------
// Is this a bottom half?  (Note to self, -AH.)
//------------------------------------------------------------------------------

static void PushFifo(unsigned data) 
{
  /* redundant
  if(atomic_read(&bbc_rfifo.n) >= BBC_RFIFO_SIZE) {
    printk(KERN_WARNING "buffer overrun\n");
    return;
  } */
  
  bbc_rfifo.data[bbc_rfifo.i_in] = data;
  
  if(bbc_rfifo.i_in == (BBC_RFIFO_SIZE - 1)) bbc_rfifo.i_in = 0;
  else bbc_rfifo.i_in++;
  
  atomic_inc(&bbc_rfifo.n);
}

static void timer_callback(unsigned long dummy)
{
  int done = 0;
  unsigned data[2];

  // Check if the rx buffer has been initialized
  if (bbc_rfifo.status & FIFO_ENABLED) {
    HandleFrameLogic();

    while(atomic_read(&bbc_rfifo.n) < BBC_RFIFO_SIZE && !done) {
      done = GetFrameNextWord(data);
      PushFifo(data[0]);	      //loopback of bus word
      if (data[1]) PushFifo(data[1]); //possible read response
    }
    
    if (atomic_read(&bbc_rfifo.n)) wake_up_interruptible(&bbc_read_wq);
  } 

  // Reset timer callback and exit.
  bbc_drv.timer.expires = jiffies + TIME_STEP;
  add_timer(&bbc_drv.timer);
}

//--------------------------------------------------------------------------
// Poll for readability
//--------------------------------------------------------------------------
unsigned int bbc_poll(struct file *filp, poll_table *wait)
{
  unsigned int mask = 0;

  poll_wait (filp, &bbc_read_wq, wait);
  if (atomic_read(&bbc_rfifo.n) > 0)
    mask |= POLLIN | POLLRDNORM;

  return mask;
}

//------------------------------------------------------------------------------
// User read.
//------------------------------------------------------------------------------

static ssize_t bbc_read(struct file *filp, char __user *buf, size_t count,
                        loff_t *dummy) {
  size_t i;
  size_t to_read;
  void *out_buf = (void *)buf;
  unsigned long dum;

  if (!access_ok(VERIFY_WRITE, (void *)buf, count)) {
    printk(KERN_WARNING "%s: (read) error accessing user space memory\n",
           DRV_NAME);
    return -EFAULT;
  } 


  if(count < BBCPCI_SIZE_UINT) return 0;
  to_read = count / BBCPCI_SIZE_UINT;

  // What to do if no data are available?
  while (atomic_read(&bbc_rfifo.n) == 0) {
    // Non blocking read, return immediately.
    if (filp->f_flags & O_NONBLOCK)
      return -EAGAIN;
    
    // Blocking read: sleep until data are available.
    if (wait_event_interruptible(bbc_read_wq, atomic_read(&bbc_rfifo.n) != 0))
      return -ERESTARTSYS; // Signal: tell the fs layer to handle it.
  }
    
  for (i = 0; i < to_read; i++) {
    if(atomic_read(&bbc_rfifo.n) == 0) 
      break;

    dum =__copy_to_user(out_buf, &bbc_rfifo.data[bbc_rfifo.i_out],
			BBCPCI_SIZE_UINT);
    out_buf += BBCPCI_SIZE_UINT;

    if (bbc_rfifo.i_out == (BBC_RFIFO_SIZE - 1))
      bbc_rfifo.i_out = 0;
    else
      bbc_rfifo.i_out++;
      
    atomic_dec(&bbc_rfifo.n);
  }
    
  return i * BBCPCI_SIZE_UINT;

}

//------------------------------------------------------------------------------
// User write.
//------------------------------------------------------------------------------

static ssize_t bbc_write(struct file *filp, const char __user *buf, 
    size_t count, loff_t *dummy) {
  size_t i;
  size_t to_write;
  unsigned data[2];

  void *in_buf = (void *)buf;


  if (!access_ok(VERIFY_READ, (void *)buf, count)) {
    printk(KERN_WARNING "%s: (write) error accessing user space memory\n", 
           DRV_NAME);
    return -EFAULT;
  }

  if (count < 2 * BBCPCI_SIZE_UINT) return 0;
  to_write = count / (2 * BBCPCI_SIZE_UINT);

  for (i = 0; i < to_write; i++) {
    
    __copy_from_user((void *)data, in_buf, 2 * BBCPCI_SIZE_UINT);
    in_buf += 2*BBCPCI_SIZE_UINT;
    
    if(data[0] >= 2 * BBCPCI_MAX_FRAME_SIZE) {
      printk(KERN_WARNING "BBC buffer overflow: mcp error: 0x%X\n", data[0]);
    } else {
      WriteToFrame(data[0], data[1]);
    }
  }

  return (2 * i * BBCPCI_SIZE_UINT);
}

//------------------------------------------------------------------------------
// Handler for ioctl.
//------------------------------------------------------------------------------

static long int bbc_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  return bbc_ioctl (filp->f_dentry->d_inode, filp, cmd, arg);
}

static int bbc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
                     unsigned long arg) {
  int ret = 0;
  switch(cmd) {
  case BBCPCI_IOC_VERSION:  // Get the current version.
    ret = 0xdead0bbc;
    break;
  case BBCPCI_IOC_GET_SERIAL:   // Get most recent serial number.
    ret = GetFrameCount();
    break;

  }
  return ret;
}


//------------------------------------------------------------------------------
// Open the bbc device.
//------------------------------------------------------------------------------

static int bbc_open(struct inode *inode, struct file *filp) {
  if (bbc_rfifo.status & FIFO_ENABLED) return -ENODEV;
  
  bbc_rfifo.i_in = bbc_rfifo.i_out = 0;
  atomic_set(&bbc_rfifo.n, 0);
  bbc_rfifo.status |= FIFO_ENABLED;
  
  bbc_drv.use_count++;

  if (bbc_drv.use_count && bbc_drv.timer_on == 0) {
    // Enable timer.
    bbc_drv.timer_on = 1;
    init_timer(&bbc_drv.timer);
    bbc_drv.timer.function = timer_callback;
    bbc_drv.timer.expires = jiffies + 1;
    add_timer(&bbc_drv.timer);
  }

  return 0;
}


//------------------------------------------------------------------------------
// Release BBC.
//------------------------------------------------------------------------------

static int bbc_release(struct inode *inode, struct file *filp)
{
  bbc_rfifo.status = FIFO_DISABLED;
  bbc_drv.use_count--;

  if(bbc_drv.use_count == 0 && bbc_drv.timer_on) {
    bbc_drv.timer_on = 0;
    del_timer_sync(&bbc_drv.timer);
  }

  return 0;
}


//------------------------------------------------------------------------------
// File operations definitions.
//------------------------------------------------------------------------------

static struct file_operations bbc_fops = {
  .owner =            THIS_MODULE,
  .read  =            bbc_read,
  .write =            bbc_write,
#ifdef HAVE_UNLOCKED_IOCTL  //prefer unlocked_ioctl, when available
  .unlocked_ioctl =   bbc_compat_ioctl,
#else
  .ioctl =            bbc_ioctl,
#endif
#ifdef HAVE_COMPAT_IOCTL    //also implement compat_ioctl, when available
  .compat_ioctl =     bbc_compat_ioctl,
#endif
  .open =             bbc_open,
  .release =          bbc_release,
  .poll =             bbc_poll
};

//------------------------------------------------------------------------------
// Control the sysfs.
//------------------------------------------------------------------------------

struct device_attribute *da_list_bbcpci[] = {
  NULL
};

static int bbcpci_start_sysfs() {
  struct device_attribute *dap, **dapp;
  dev_t devnum;
    
  bbcpci_class = class_create(THIS_MODULE, "simbbc_pci");
  if (IS_ERR(bbcpci_class)) return 0;

  devnum = bbc_drv.bbc_cdev.dev;
#ifndef USE_NEWER_DEVICE_CREATE
  bbcpci_d = device_create(bbcpci_class, NULL, devnum, "bbcpci");
#else
  bbcpci_d = device_create(bbcpci_class, NULL, devnum, NULL, "bbcpci");
#endif
  if (IS_ERR(bbcpci_d)) return 0;

  //loop through device attributes and create files
  //if these are ever used, may need a different set for bi0
  for (dapp = da_list_bbcpci; *dapp; dapp++) {
    dap = *dapp;
    if (device_create_file(bbcpci_d, dap))
      printk(KERN_ERR "bbc_sync: class_device_create_file failed\n");
  }
  
  return 0;
}

static void bbcpci_remove_sysfs() {
  struct device_attribute *dap, **dapp;
  dev_t devnum;

  for (dapp = da_list_bbcpci; *dapp; dapp++) {
    dap = *dapp;
    device_remove_file(bbcpci_d, dap);
  }

  devnum = bbc_drv.bbc_cdev.dev;
  if (devnum && !IS_ERR(bbcpci_d))
    device_destroy(bbcpci_class, devnum);
  class_destroy(bbcpci_class);
}

//------------------------------------------------------------------------------
// Define the "PCI" layer.
//------------------------------------------------------------------------------

static int __devinit bbc_pci_init_one(struct pci_dev *pdev, 
                                      const struct pci_device_id *ent) {
  int ret;
  dev_t devnum;
#if PARALLEL_BASE
  struct resource  *res;
#endif

  //create and register character device
  //request 1 minor device number at a dynamically allocated major
  ret = alloc_chrdev_region(&devnum, bbc_minor, 1, DRV_NAME);
  if (ret < 0) {
    printk(KERN_WARNING DRV_NAME " can't allocate major\n");
    return ret;
  }
  printk(KERN_DEBUG DRV_NAME " major: %d minor: %d dev: %d\n", 
      MAJOR(devnum), bbc_minor, devnum);
  cdev_init(&bbc_drv.bbc_cdev, &bbc_fops);
  bbc_drv.bbc_cdev.owner = THIS_MODULE;
  ret = cdev_add(&bbc_drv.bbc_cdev, devnum, 1);
  if (ret < 0) {
    printk(KERN_WARNING DRV_NAME " -failed to registed the bbc_pci device\n");
    unregister_chrdev_region(devnum, 1);
    return ret;
  }

#if PARALLEL_BASE
  // Register parallel port
  res = request_region(PARALLEL_BASE, 1, DRV_NAME);
  if(res == NULL) {
    printk(KERN_WARNING DRV_NAME " failed to register the parallel port\n");
    cdev_del(&bbc_drv.bbc_cdev);
    unregister_chrdev_region(devnum, 1);
    return -ENODEV;
  }
#endif

  // Create /proc entry.
  create_proc_read_entry(DRV_NAME, 
                         0,         // Default mode.
                         NULL,      // Parent dir. 
                         bbc_read_procmem,
                         NULL       // Client data.
                        );

  bbc_drv.timer_on = 0;
  bbc_drv.use_count = 0;
  bbc_rfifo.status = FIFO_DISABLED;
  InitBBCData();
  InitAuxData();

  bbcpci_start_sysfs();

  printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);

  return 0;
}


//------------------------------------------------------------------------------
// Remove the device.
//------------------------------------------------------------------------------

static void __devexit bbc_pci_remove_one(struct pci_dev *pdev) {
  bbcpci_remove_sysfs();
  
#if PARALLEL_BASE
  // Release parallel port
  release_region(PARALLEL_BASE, 1);
#endif
  
  cdev_del(&bbc_drv.bbc_cdev);
  unregister_chrdev_region(bbc_drv.bbc_cdev.dev, 1);

  printk(KERN_NOTICE "%s: driver removed\n", DRV_NAME); 
}


//------------------------------------------------------------------------------
// More definitions.
//------------------------------------------------------------------------------

static int __init bbc_pci_init(void) {
  //maintain naming despite not using a PCI device
  return bbc_pci_init_one(NULL, NULL);
}

static void __exit bbc_pci_cleanup(void) {
  //maintain naming despite not using a PCI device
  bbc_pci_remove_one(NULL);
}

// Declare module init and exit functions.
module_init(bbc_pci_init);
module_exit(bbc_pci_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Blast 2009");
MODULE_DESCRIPTION("bbc_pci: a driver to simulate the pci Blast Bus Controller");
MODULE_ALIAS("/dev/bbcpci");

#ifdef module_param
module_param(bbc_minor, int, S_IRUGO);
#else
#warning "using old crufty MODULE_PARM"
MODULE_PARM(bbc_minor, "i");
#endif

MODULE_PARM_DESC(bbc_minor, " bbc minor number");
