/* bbc_pci: kernel driver for the PCI BLAST Bus Controller
 *
 * This software is copyright (C) 2004 University of Toronto
 * Updated 2006 by Adam Hincks, Princeton University, for the ACT experiment.
 * Re-updated 2007 by University of Toronto for Spider
 * Hacked (and copyright) 2008-2010 by Matthew Truch, for the BLAST experiment.
 * 
 * This file is part of bbc_pci.
 * 
 * bbc_pci is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * bbc_pci is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with bbc_pci; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA 
 * Or visit http://www.gnu.org/licenses/
 *
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

#define DRV_NAME    "bbc_pci"
#define DRV_VERSION "1.1"
#define DRV_RELDATE ""

#ifndef VERSION_CODE
#  define VERSION_CODE(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

#if LINUX_VERSION_CODE < VERSION_CODE(2,6,0)
# error "This kernel is too old and is not supported" 
#endif
#if LINUX_VERSION_CODE >= VERSION_CODE(3,3,0)
# error "This kernel version is not supported"
#endif
//incompatible API change, may have changed before 2.6.27
#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,27)
#define USE_NEWER_DEVICE_CREATE
#endif

static int bbc_minor = 0;
static int bi0_minor = 1;

static struct pci_device_id bbc_pci_tbl[] = {
  {0x5045, 0x4243, (PCI_ANY_ID), (PCI_ANY_ID), 0, 0, 0}, 
  {0,}
};
MODULE_DEVICE_TABLE(pci, bbc_pci_tbl);

#define FIFO_DISABLED 0
#define FIFO_ENABLED  1
#define FIFO_EMPTY    2
#define FIFO_FULL     4

#define TX_BUFFER_SIZE 0x4000
#define RX_BUFFER_SIZE 0x4000
#define BI0_BUFFER_SIZE (624*25)

DECLARE_WAIT_QUEUE_HEAD(bbc_read_wq);

static struct {
  unsigned long mem_base_raw;
  void          *mem_base;
  unsigned long flags;
  unsigned char irq;
  unsigned long len;
  struct timer_list timer;

  int use_count;
  int timer_on;

  //NB. since there's only one global bbc_drv, need one cdev for each device
  struct cdev bbc_cdev;
  struct cdev bi0_cdev;
} bbc_drv;

#define BBC_WFIFO_SIZE (2*TX_BUFFER_SIZE)
static struct {
  volatile int i_in;
  volatile int i_out;
  unsigned data[BBC_WFIFO_SIZE];
  volatile int status;
  atomic_t n;
} bbc_wfifo;

static struct { 
  int i_in;
  int i_out;
  struct {
    unsigned long long cputime;  
    unsigned int bad;  
  } time_data[BBC_WFIFO_SIZE];
  unsigned char status;
} irq_time_fifo;

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

static struct class *bbcpci_class;
struct device *bbcpci_d;
struct device *bbcbi0_d;

static long int bbc_compat_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg);
static int bbc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
                     unsigned long arg);
static int bbcpci_start_sysfs(void);
static void bbcpci_remove_sysfs(void);

//------------------------------------------------------------------------------
// Functions for external calling (for bbc_sync module).
//------------------------------------------------------------------------------

int bbcpci_get_irq(void) {
  return bbc_drv.irq;
}

int bbcpci_interrupt_test_and_clear(void) {
  if (ioread32(bbc_drv.mem_base + BBCPCI_ADD_IRQREG)) {
    // Acknowledge receipt of IRQ.
    iowrite32(0, bbc_drv.mem_base + BBCPCI_ADD_IRQREG);
    return 1;
  }
  else
    return 0;
}

int bbcpci_enable_interrupts(void) {
  return bbc_ioctl(NULL, NULL, BBCPCI_IOC_ON_IRQ, 0uL);
}

int bbcpci_disable_interrupts(void) {
  irq_time_fifo.status = 0;
  return bbc_ioctl(NULL, NULL, BBCPCI_IOC_OFF_IRQ, 0uL);
}

static int interrupt_period, external_serial_on;

int bbcpci_set_interrupt_period(int period) {
  (void)bbc_ioctl(NULL, NULL, BBCPCI_IOC_IRQ_RATE, (unsigned long) period);
  return period - 1;
}

int bbcpci_get_interrupt_period(void) {
  return interrupt_period;
}

unsigned int bbcpci_get_serialnumber(int *busy) {
  // N.B.: ioctl returns 1 = new S# is ready, 0=busy reading new S#.
  // Thus 1 means NOT busy, 0 means busy!
  *busy = bbc_ioctl(NULL, NULL, BBCPCI_IOC_SERIAL_RDY, 0uL) ? 0 : 1;
  
  return bbc_ioctl(NULL, NULL, BBCPCI_IOC_GET_SERIAL, 0uL);
}

int bbcpci_set_ext_serial(int external_on) {
  unsigned int cmd;
  if (external_on)
    cmd = BBCPCI_IOC_EXT_SER_ON;
  else
    cmd = BBCPCI_IOC_EXT_SER_OFF;
  return bbc_ioctl(NULL, NULL, cmd, 0UL);
}

int bbcpci_get_ext_serial(void) {
  return external_serial_on;
}

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
    len += sprintf(buf+len, "bbc_wfifo: %10d %10d %10d 0x%08x\n", 
		   bbc_wfifo.i_in, bbc_wfifo.i_out, 
		   atomic_read(&bbc_wfifo.n), bbc_wfifo.status);

  if( (limit - len) >= 80)
    len += sprintf(buf+len, "bbc_rfifo: %10d %10d %10d 0x%08x\n", 
		   bbc_rfifo.i_in, bbc_rfifo.i_out, 
		   atomic_read(&bbc_rfifo.n), bbc_rfifo.status);
  
  if( (limit - len) >= 80)
    len += sprintf(buf+len, "bi0_wfifo: %10d %10d %10d 0x%08x\n", 
		   bi0_wfifo.i_in, bi0_wfifo.i_out, 
		   atomic_read(&bi0_wfifo.n), bi0_wfifo.status);
  
  *eof = 1;
  return len;
}


//------------------------------------------------------------------------------
// Is this a bottom half?  (Note to self, -AH.)
//------------------------------------------------------------------------------

static void timer_callback(unsigned long dummy)
{
  static loff_t wp, rp;
  unsigned int wp_next; //used by bi0
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
  
  // Check if NIOS is busy resetting itself.
  if(ioread32(bbc_drv.mem_base + BBCPCI_ADD_COMREG)) 
    goto tc_end;

  // Read data from NIOS.
  if (bbc_rfifo.status & FIFO_ENABLED) {
    while (atomic_read(&bbc_rfifo.n) < BBC_RFIFO_SIZE)  {
      wp = ioread32(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_WP);
      rp = ioread32(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_RP);      
      rp += BBCPCI_SIZE_UINT;
      if(rp >= BBCPCI_ADD_READ_BUF_END) rp = BBCPCI_ADD_READ_BUF;
      if(rp == wp) break;

      bbc_rfifo.data[bbc_rfifo.i_in] = ioread32(bbc_drv.mem_base + rp);

      /*
      // The old method for inserting serial numbers before we added the snyc
      // [sic] box.
      if (GET_NODE(bbc_rfifo.data[bbc_rfifo.i_in]) == 63) {
        if (GET_CH(bbc_rfifo.data[bbc_rfifo.i_in]) == 0)
          bbc_rfifo.data[bbc_rfifo.i_in] = 
            (bbc_rfifo.data[bbc_rfifo.i_in] & 0xffff0000) | 
           (temp_counter & 0x0000ffff);
        else
          bbc_rfifo.data[bbc_rfifo.i_in] = 
            (bbc_rfifo.data[bbc_rfifo.i_in] & 0xffff0000) | 
            ((temp_counter >> 16) & 0x0000ffff);
      }*/
      
      if(bbc_rfifo.i_in == (BBC_RFIFO_SIZE - 1))
        bbc_rfifo.i_in = 0;
      else
        bbc_rfifo.i_in++;
      
      iowrite32(rp, bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_RP);
      atomic_inc(&bbc_rfifo.n);
    }
    if (atomic_read(&bbc_rfifo.n))
      wake_up_interruptible(&bbc_read_wq);
  }

  // Write Blast data to NIOS.
  if (bbc_wfifo.status & FIFO_ENABLED) {
    wp = ioread32(bbc_drv.mem_base + BBCPCI_ADD_WRITE_BUF_P);
    if (wp < BBCPCI_ADD_IR_WRITE_BUF) {

      nwritten = 0;
      while ((nwritten < BBCPCI_IR_WRITE_BUF_SIZE) 
             && atomic_read(&bbc_wfifo.n) ) {
        wp += 2 * BBCPCI_SIZE_UINT;
        iowrite32(bbc_wfifo.data[bbc_wfifo.i_out + 1],
               bbc_drv.mem_base + wp + BBCPCI_SIZE_UINT);
        iowrite32(bbc_wfifo.data[bbc_wfifo.i_out], bbc_drv.mem_base + wp);
        if (bbc_wfifo.i_out == (BBC_WFIFO_SIZE - 2))
          bbc_wfifo.i_out = 0;
        else
          bbc_wfifo.i_out += 2;
        nwritten++;
	    atomic_sub(2, &bbc_wfifo.n);
      }

      if (nwritten)
        iowrite32(wp, bbc_drv.mem_base + BBCPCI_ADD_WRITE_BUF_P);
    }
  } 

  // Write bi-phase data to NIOS.
  if(bi0_wfifo.status & FIFO_ENABLED) {
    rp = ioread32(bbc_drv.mem_base + BBCPCI_ADD_BI0_RP);
    wp = ioread32(bbc_drv.mem_base + BBCPCI_ADD_BI0_WP);
    while( atomic_read(&bi0_wfifo.n) ) {
      if (wp >= BBCPCI_IR_BI0_BUF_END) {
        wp_next = BBCPCI_IR_BI0_BUF;
      } else {
        wp_next = wp + BBCPCI_SIZE_UINT;
      }
      if(wp_next == rp) break;

      out_data[--idx] = bi0_wfifo.data[bi0_wfifo.i_out];
      if(bi0_wfifo.i_out == (BI0_WFIFO_SIZE - 1)) {
        bi0_wfifo.i_out = 0;
      } else {
        bi0_wfifo.i_out++;
      }
      if(idx == 0) {
        idx = 2;
        iowrite32(*(unsigned *)out_data, bbc_drv.mem_base + wp);
        wp = wp_next;
      }
      atomic_dec(&bi0_wfifo.n);
    }
    iowrite32(wp, bbc_drv.mem_base + BBCPCI_ADD_BI0_WP);
  }

 tc_end:
  // Reset timer callback and exit.
  bbc_drv.timer.expires = jiffies + 1;
  add_timer(&bbc_drv.timer);
}

//--------------------------------------------------------------------------
// Poll for readability
//--------------------------------------------------------------------------
unsigned int bbc_poll(struct file *filp, poll_table *wait)
{
  int minor;
  unsigned int mask = 0;

  minor = *(int *)filp->private_data;
  if (minor == bbc_minor)
  {
    poll_wait (filp, &bbc_read_wq, wait);
    if (atomic_read(&bbc_rfifo.n) > 0) mask |= POLLIN | POLLRDNORM;
  }

  return mask;
}

//------------------------------------------------------------------------------
// User read.
//------------------------------------------------------------------------------

static ssize_t bbc_read(struct file *filp, char __user *buf, size_t count,
                        loff_t *dummy) {
  int minor;
  size_t i;
  size_t to_read;
  void *out_buf = (void *)buf;
  unsigned long dum;
  
  if (!access_ok(VERIFY_WRITE, (void *)buf, count)) {
    printk(KERN_WARNING "%s: (read) error accessing user space memory\n", 
           DRV_NAME);
    return -EFAULT;
  }

  minor = *(int *)filp->private_data;

  if (minor == bbc_minor) {

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
        // Old, deprecated: interruptible_sleep_on(&bbc_read_wq);
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
  else if (minor == bi0_minor)
    return 0;

  return 0;
}


//------------------------------------------------------------------------------
// User write.
//------------------------------------------------------------------------------

static ssize_t bbc_write(struct file *filp, const char __user *buf, 
    size_t count, loff_t *dummy) {
  int minor;
  size_t i;
  size_t to_write;
  void *in_buf = (void *)buf;


  if (!access_ok(VERIFY_READ, (void *)buf, count)) {
    printk(KERN_WARNING "%s: (write) error accessing user space memory\n", 
           DRV_NAME);
    return -EFAULT;
  }

  minor = *(int *)filp->private_data;

  if (minor == bbc_minor) {
    if (count < 2 * BBCPCI_SIZE_UINT) 
      return 0;
    to_write = count / (2 * BBCPCI_SIZE_UINT);

    for (i = 0; i < to_write; i++) {
      if (atomic_read(&bbc_wfifo.n) == BBC_WFIFO_SIZE) {
        printk(KERN_WARNING "%s: bbc buffer overrun. size = %x\n", 
               DRV_NAME, atomic_read(&bbc_wfifo.n));
        break;
      }

      __copy_from_user((void *)&bbc_wfifo.data[bbc_wfifo.i_in], 
                       in_buf, 2 * BBCPCI_SIZE_UINT);

      if (bbc_wfifo.i_in == (BBC_WFIFO_SIZE - 2))
        bbc_wfifo.i_in = 0;
      else
        bbc_wfifo.i_in += 2;
    
      in_buf += 2*BBCPCI_SIZE_UINT;
      atomic_add(2, &bbc_wfifo.n);
    }

    return (2 * i * BBCPCI_SIZE_UINT);
  } 
  else if (minor == bi0_minor) {
    if (count < sizeof(unsigned short)) 
      return 0;
    to_write = count / sizeof(unsigned short);
    
    for (i = 0; i < to_write; i++) {
      if (atomic_read(&bi0_wfifo.n) == BI0_WFIFO_SIZE) {
        printk(KERN_WARNING "%s: bi0 buffer overrun. size = %x\n", 
               DRV_NAME, atomic_read(&bi0_wfifo.n));
        break;
      }

      __copy_from_user((void *)&bi0_wfifo.data[bi0_wfifo.i_in], in_buf, 
                       sizeof(unsigned short));

      if (bi0_wfifo.i_in == (BI0_WFIFO_SIZE - 1))
        bi0_wfifo.i_in = 0;
      else
        bi0_wfifo.i_in++;

      in_buf += sizeof(unsigned short);
      atomic_inc(&bi0_wfifo.n);
    }

    return i * sizeof(unsigned short);
  } 

  return 0;
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
  unsigned long dum;
  
  switch(cmd) {
  case BBCPCI_IOC_RESET:    // Reset the BBC board - clear fifo, registers.
    iowrite32(BBCPCI_COMREG_RESET, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_SYNC:     // Clear read buffers and restart frame.
    iowrite32(BBCPCI_COMREG_SYNC, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_VERSION:  // Get the current version.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_VERSION);
    break;
  case BBCPCI_IOC_COUNTER:  // Get the counter.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_COUNTER);
    break;
  case BBCPCI_IOC_WRITEBUF:  // How many words in the NIOS write buf?
    ret  = ioread32(bbc_drv.mem_base + BBCPCI_ADD_WRITE_BUF_P);
    ret -= BBCPCI_ADD_IR_WRITE_BUF;
    break;
  case BBCPCI_IOC_COMREG:  // Return the command register.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_READBUF_WP: // Where nios is about to write.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_WP);
    break;
  case BBCPCI_IOC_READBUF_RP: // Where the PC is about to read.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_READ_BUF_RP);
    break;
  case BBCPCI_IOC_BBC_FIONREAD:
    ret = atomic_read(&bbc_wfifo.n);
    break;
  case BBCPCI_IOC_BI0_FIONREAD:
    ret = atomic_read(&bi0_wfifo.n);
    break;
  case BBCPCI_IOC_ON_IRQ: // Enable IRQ generation.
    iowrite32(BBCPCI_COMREG_ON_IRQ, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_OFF_IRQ: // Disable IRQ generation.
    iowrite32(BBCPCI_COMREG_OFF_IRQ, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    break;
  case BBCPCI_IOC_IRQT_READ:
    if (irq_time_fifo.i_in != irq_time_fifo.i_out)
      ret = 1;
    else
      ret = 0;
    dum = __copy_to_user((void *)arg, 
                         &(irq_time_fifo.time_data[irq_time_fifo.i_out]),
                         sizeof(irq_time_fifo.time_data[irq_time_fifo.i_out]));
    if (ret) {
      if (++irq_time_fifo.i_out >= BBC_WFIFO_SIZE)
        irq_time_fifo.i_out = 0;
    }
    break;  
  case BBCPCI_IOC_IRQ_RATE: // IRQ rate
    //deprecated. On new firmware, this word combines internal/external rates
    iowrite32(arg - 1, bbc_drv.mem_base + BBCPCI_ADD_IRQ_RATE);
    interrupt_period = arg - 1;
    break;
  case BBCPCI_IOC_IRQ_RATE_INT: // IRQ rate, internal only
    //internal units are 4MHz periods
    dum = ioread32(bbc_drv.mem_base + BBCPCI_ADD_IRQ_RATE);
    dum = (dum & 0x000000ff) | (((arg-1) & 0x00ffffff) << 8);
    iowrite32(dum, bbc_drv.mem_base + BBCPCI_ADD_IRQ_RATE);
    interrupt_period = arg - 1;
    break;
  case BBCPCI_IOC_IRQ_RATE_EXT: // IRQ rate, external only
    //external units are in snyc box serial periods
    dum = ioread32(bbc_drv.mem_base + BBCPCI_ADD_IRQ_RATE);
    dum = (dum & 0xffffff00) | ((arg-1) & 0x000000ff);
    iowrite32(dum, bbc_drv.mem_base + BBCPCI_ADD_IRQ_RATE);
    interrupt_period = arg - 1;
    break;
  case BBCPCI_IOC_FRAME_RATE: // BBC frame rate
    //deprecated. On new firmware, this word combines internal/external rates
    iowrite32(arg - 1, bbc_drv.mem_base + BBCPCI_ADD_FRAME_RATE);
    break;
  case BBCPCI_IOC_FRAME_RATE_INT: // BBC frame rate, internal only
    //internal units are 4MHz periods
    dum = ioread32(bbc_drv.mem_base + BBCPCI_ADD_FRAME_RATE);
    dum = (dum & 0x000000ff) | (((arg-1) & 0x00ffffff) << 8);
    iowrite32(dum, bbc_drv.mem_base + BBCPCI_ADD_FRAME_RATE);
    break;
  case BBCPCI_IOC_FRAME_RATE_EXT: // BBC frame rate, external only
    //external units are in snyc box serial periods
    dum = ioread32(bbc_drv.mem_base + BBCPCI_ADD_FRAME_RATE);
    dum = (dum & 0xffffff00) | ((arg-1) & 0x000000ff);
    iowrite32(dum, bbc_drv.mem_base + BBCPCI_ADD_FRAME_RATE);
    break;
  case BBCPCI_IOC_RESET_SERIAL: // Reset serial.
    // Deprecated.
    break;
  case BBCPCI_IOC_GET_SERIAL:   // Get most recent serial number.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_SERIAL);
    break;
  case BBCPCI_IOC_SERIAL_RDY:   // See if most recent serial number is fresh.
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_SERIAL_RDY);
    break;
  case BBCPCI_IOC_EXT_SER_ON: // Set serial generation to external source.
    iowrite32(BBCPCI_COMREG_EXT_SER_ON, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    external_serial_on = 1;
    break;
  case BBCPCI_IOC_EXT_SER_OFF: // Set serial generation to internal source.
    iowrite32(BBCPCI_COMREG_EXT_SER_OFF, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    external_serial_on = 0;
    break;
  case BBCPCI_IOC_FRAME_COUNT: // Count of clock cycles per frame
    ret = ioread32(bbc_drv.mem_base + BBCPCI_ADD_FRAME_COUNT);
    break;
  default:
    break;
  }

  return ret;
}


//------------------------------------------------------------------------------
// Open the bbc device.
//------------------------------------------------------------------------------

static int bbc_open(struct inode *inode, struct file *filp) {
  int minor = MINOR(inode->i_rdev);

  filp->private_data = (minor == bbc_minor) ? &bbc_minor : &bi0_minor;

  if (minor == bbc_minor) {
    if ((bbc_wfifo.status & FIFO_ENABLED) || 
        (bbc_rfifo.status & FIFO_ENABLED)) {
      return -ENODEV;
    }
    filp->private_data = &bbc_minor;

    bbc_wfifo.i_in = bbc_wfifo.i_out = 0;
    atomic_set(&bbc_wfifo.n, 0);
    bbc_wfifo.status |= FIFO_ENABLED;

    bbc_rfifo.i_in = bbc_rfifo.i_out = 0;
    atomic_set(&bbc_rfifo.n, 0);
    bbc_rfifo.status |= FIFO_ENABLED;

    bbc_drv.use_count++;
  }

  if (minor == bi0_minor) {
    if (bi0_wfifo.status & FIFO_ENABLED)
      return -ENODEV;
    filp->private_data = &bi0_minor;

    bi0_wfifo.i_in = bi0_wfifo.i_out = 0;
    atomic_set(&bi0_wfifo.n, 0);
    bi0_wfifo.status |= FIFO_ENABLED;

    bbc_drv.use_count++;
  }

  if (bbc_drv.use_count && bbc_drv.timer_on == 0) {
    // Sync bbc.
    iowrite32(BBCPCI_COMREG_SYNC, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
    // Enable timer.
    bbc_drv.timer_on = 1;
    init_timer(&bbc_drv.timer);
    bbc_drv.timer.function = timer_callback;
    bbc_drv.timer.expires = jiffies + 1;
    add_timer(&bbc_drv.timer);
  }

  // Remove.
  irq_time_fifo.status = 0;
  
  return 0;
}


//------------------------------------------------------------------------------
// Release BBC.
//------------------------------------------------------------------------------

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

    // Reset BBC. 
    iowrite32(BBCPCI_COMREG_RESET, bbc_drv.mem_base + BBCPCI_ADD_COMREG);
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

/*
//------------------------------------------------------------------------------
// For debugging interrupts; interrupts are generally handled by act_timing.
//------------------------------------------------------------------------------

static irqreturn_t adaminterrupt(int irq, void *dev_id, struct pt_regs *regs) {
   irqreturn_t ret;
  
  if (!irq_time_fifo.status) {
    irq_time_fifo.status = 1;
    irq_time_fifo.i_in = 0;
    irq_time_fifo.i_out = 0;
  }
    
  rdtscll(irq_time_fifo.time_data[irq_time_fifo.i_in].cputime);
  
  if (!bbcpci_interrupt_test_and_clear()) {
    irq_time_fifo.time_data[irq_time_fifo.i_in].bad = 1;
    ret = IRQ_NONE;
  }
  else {
    irq_time_fifo.time_data[irq_time_fifo.i_in].bad = 0;
    ret = IRQ_HANDLED;
  }

  if (++irq_time_fifo.i_in >= BBC_WFIFO_SIZE)
    irq_time_fifo.i_in = 0;
    
  return ret;
}
*/


//------------------------------------------------------------------------------
// Control the sysfs.
//------------------------------------------------------------------------------

struct device_attribute *da_list_bbcpci[] = {
  NULL
};

static int bbcpci_start_sysfs() {
  struct device_attribute *dap, **dapp;
  dev_t devnum;
    
  bbcpci_class = class_create(THIS_MODULE, "bbc_pci");
  if (IS_ERR(bbcpci_class)) return 0;

  devnum = bbc_drv.bbc_cdev.dev;
#ifndef USE_NEWER_DEVICE_CREATE
  bbcpci_d = device_create(bbcpci_class, NULL, devnum, "bbcpci");
#else
  bbcpci_d = device_create(bbcpci_class, NULL, devnum, NULL, "bbcpci");
#endif
  if (IS_ERR(bbcpci_d)) return 0;
  devnum = bbc_drv.bi0_cdev.dev;
#ifndef USE_NEWER_DEVICE_CREATE
  bbcbi0_d = device_create(bbcpci_class, NULL, devnum, "bbc_bi0");
#else
  bbcbi0_d = device_create(bbcpci_class, NULL, devnum, NULL, "bbc_bi0");
#endif
  if (IS_ERR(bbcbi0_d)) return 0;

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
  devnum = bbc_drv.bi0_cdev.dev;
  if (devnum && !IS_ERR(bbcbi0_d))
    device_destroy(bbcpci_class, devnum);
  class_destroy(bbcpci_class);
}

//------------------------------------------------------------------------------
// Define the PCI layer.
//------------------------------------------------------------------------------

static int __devinit bbc_pci_init_one(struct pci_dev *pdev, 
                                      const struct pci_device_id *ent) {
  int ret;
  dev_t devnum;

  ret = pci_enable_device (pdev);
  if(ret) return ret;

  bbc_drv.mem_base_raw = pci_resource_start(pdev, 0);
  bbc_drv.flags        = pci_resource_flags(pdev, 0);
  bbc_drv.len          = pci_resource_len(pdev, 0);
  bbc_drv.irq          = pdev->irq;
  
  if (!bbc_drv.mem_base_raw || ((bbc_drv.flags & IORESOURCE_MEM) == 0)) {
    printk(KERN_ERR "%s: no I/O resource at PCI BAR #0\n", DRV_NAME);
    return -ENODEV;
  }

  /* used only for debugging
  printk(KERN_WARNING "%s: bbc_pci: base %x len %x \n", DRV_NAME, 
	  (unsigned int)bbc_drv.mem_base_raw, (unsigned int)bbc_drv.len);
  */

  if (!request_mem_region(bbc_drv.mem_base_raw, bbc_drv.len, DRV_NAME)) {
    printk(KERN_WARNING "%s: bbc_pci: memory already in use\n", DRV_NAME);
    return -EBUSY;
  }

  bbc_drv.mem_base = ioremap_nocache(bbc_drv.mem_base_raw, bbc_drv.len);

  //create and register character device
  //request 2 minor device numbers at a dynamically allocated major
  ret = alloc_chrdev_region(&devnum, bbc_minor, 2, DRV_NAME);
  if (ret < 0) {
    printk(KERN_WARNING DRV_NAME " can't allocate major\n");
    return ret;
  }
  printk(KERN_DEBUG DRV_NAME " major: %d minor: %d dev: %d\n", 
      MAJOR(devnum), bbc_minor, devnum);
  cdev_init(&bbc_drv.bbc_cdev, &bbc_fops);
  bbc_drv.bbc_cdev.owner = THIS_MODULE;
  ret = cdev_add(&bbc_drv.bbc_cdev, devnum, 1);
  if (ret < 0)
    printk(KERN_WARNING DRV_NAME " failed to registed the bbc_pci device");

  cdev_init(&bbc_drv.bi0_cdev, &bbc_fops);
  bbc_drv.bi0_cdev.owner = THIS_MODULE;
  devnum = MKDEV(MAJOR(devnum), bi0_minor);
  ret = cdev_add(&bbc_drv.bi0_cdev, devnum, 1);
  if (ret < 0)
    printk(KERN_WARNING DRV_NAME " failed to registed the bbc_bi0 device");

  // Create /proc entry.
  create_proc_read_entry(DRV_NAME, 
                         0,         // Default mode.
                         NULL,      // Parent dir. 
                         bbc_read_procmem,
                         NULL       // Client data.
                        );

  // Reset BBC.
  iowrite32(BBCPCI_COMREG_RESET, bbc_drv.mem_base + BBCPCI_ADD_COMREG);

  bbc_drv.timer_on = 0;
  bbc_drv.use_count = 0;
  bbc_wfifo.status = FIFO_DISABLED;
  bbc_rfifo.status = FIFO_DISABLED;
  bi0_wfifo.status = FIFO_DISABLED;

  bbcpci_start_sysfs();

  printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);

  return 0;
}


//------------------------------------------------------------------------------
// Remove the device.
//------------------------------------------------------------------------------

static void __devexit bbc_pci_remove_one(struct pci_dev *pdev) {
  // Make sure interrupts have been turned off.
  bbcpci_disable_interrupts();

  bbcpci_remove_sysfs();
  
  iounmap(bbc_drv.mem_base);
  release_mem_region(bbc_drv.mem_base_raw, bbc_drv.len);

  cdev_del(&bbc_drv.bbc_cdev);
  cdev_del(&bbc_drv.bi0_cdev);
  unregister_chrdev_region(bbc_drv.bbc_cdev.dev, 2);

  pci_disable_device(pdev);
  printk(KERN_NOTICE "%s: driver removed\n", DRV_NAME); 
}


//------------------------------------------------------------------------------
// More definitions.
//------------------------------------------------------------------------------

static struct pci_driver bbc_driver = {
  .name     = DRV_NAME,
  .probe    = bbc_pci_init_one,
  .remove   = __devexit_p(bbc_pci_remove_one),
  .id_table = bbc_pci_tbl,
};

static int __init bbc_pci_init(void) {
  return pci_register_driver(&bbc_driver);
}

static void __exit bbc_pci_cleanup(void) {
  pci_unregister_driver(&bbc_driver);
}

// Declare module init and exit functions.
module_init(bbc_pci_init);
module_exit(bbc_pci_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Blast 2004");
MODULE_DESCRIPTION("bbc_pci: a driver for the pci Blast Bus Controller");
MODULE_ALIAS_CHARDEV_MAJOR(BBC_MAJOR);
MODULE_ALIAS("/dev/bbcpci");
MODULE_ALIAS("/dev/bi0_pci");

#ifdef module_param
module_param(bbc_minor, int, S_IRUGO);
module_param(bi0_minor, int, S_IRUGO);
#else
#warning "using old crufty MODULE_PARM"
MODULE_PARM(bbc_minor, "i");
MODULE_PARM(bi0_minor, "i");
#endif

MODULE_PARM_DESC(bbc_minor, " bbc minor number");
MODULE_PARM_DESC(bi0_minor, " bi0 minor number");


// Exported functions.

EXPORT_SYMBOL(bbcpci_get_irq);
EXPORT_SYMBOL(bbcpci_interrupt_test_and_clear);
EXPORT_SYMBOL(bbcpci_enable_interrupts);
EXPORT_SYMBOL(bbcpci_disable_interrupts);
EXPORT_SYMBOL(bbcpci_set_interrupt_period);
EXPORT_SYMBOL(bbcpci_get_interrupt_period);
EXPORT_SYMBOL(bbcpci_get_serialnumber);
EXPORT_SYMBOL(bbcpci_set_ext_serial);
EXPORT_SYMBOL(bbcpci_get_ext_serial);
