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
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Written by Enzo Pascale, Sept. 2 2009
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

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/msr.h>
#include <asm/uaccess.h>

#include "bbc_pci.h"

#include "simbbc.h"


#define BBCPCI_SIZE_UINT           sizeof(unsigned int)

#define FIFO_DISABLED 0
#define FIFO_ENABLED  1
#define FIFO_EMPTY    2
#define FIFO_FULL     4

#define DRV_NAME    "simbbc"
#define DRV_VERSION "1.0"
#define DRV_RELDATE ""

#define BBC_MAJOR 0
#define BUFFER_SIZE  (2*BBCPCI_MAX_FRAME_SIZE)


#define PARALLEL_BASE 0x378

#define ACS0 23  /* ACS0 node */


static struct {
  struct timer_list timer;

  int use_count;
  int timer_on;
  int timer_event;
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


static struct {
  unsigned framecounter;
  unsigned bbc_data[2*BBCPCI_MAX_FRAME_SIZE];
  unsigned *bb1_data;
  unsigned *bb2_data;
  unsigned *bb1_ptr;
  unsigned *bb2_ptr;
  
} bbc_data;


struct ChannelStruct {
  char node;
  char addr;
  unsigned int data;
};
 
static struct {
  struct ChannelStruct cam0_pulse;
  struct ChannelStruct cam1_pulse;
  struct ChannelStruct cam0_trigger;
  struct ChannelStruct cam1_trigger;
  struct ChannelStruct gyro1;
  struct ChannelStruct gyro2;
  struct ChannelStruct gyro3;
  struct ChannelStruct lat;
  struct ChannelStruct lon;
  /********************************************/
  unsigned int cam0_index, cam0_counter;
  unsigned int cam1_index, cam1_counter;
} aux_data; // these is data we want to simulate on the bus



static int bbc_major = BBC_MAJOR;
static int bbc_minor = 0;

DECLARE_WAIT_QUEUE_HEAD(bbc_read_wq);

static void PushFifo(unsigned *data) 
{

  if(atomic_read(&bbc_rfifo.n) >= BBC_RFIFO_SIZE) {
    printk(KERN_WARNING "buffer overrun\n");
    return;
  }
  
  bbc_rfifo.data[bbc_rfifo.i_in] = *data;
  
  if(bbc_rfifo.i_in == (BBC_RFIFO_SIZE - 1))
    bbc_rfifo.i_in = 0;
  else
    bbc_rfifo.i_in++;
  
  atomic_inc(&bbc_rfifo.n);
}

static void InitAuxData(void) {
  aux_data.cam0_pulse.node = CAM0_PULSE_NODE;
  aux_data.cam0_pulse.addr = CAM0_PULSE_CH;
  aux_data.cam0_pulse.data = 0;
  //
  aux_data.cam1_pulse.node = CAM1_PULSE_NODE;
  aux_data.cam1_pulse.addr = CAM1_PULSE_CH;
  aux_data.cam1_pulse.data = 0;
  //
  aux_data.cam0_trigger.node = CAM0_TRIGGER_NODE;
  aux_data.cam0_trigger.addr = CAM0_TRIGGER_CH;
  aux_data.cam0_trigger.data = 0;
  //
  aux_data.cam1_trigger.node = CAM1_TRIGGER_NODE;
  aux_data.cam1_trigger.addr = CAM1_TRIGGER_CH;
  aux_data.cam1_trigger.data = 0;
  //
  aux_data.gyro1.node = GYRO1_NODE;
  aux_data.gyro1.addr = GYRO1_CH;
  aux_data.gyro1.data = GYRO1_DATA;
  //
  aux_data.gyro2.node = GYRO2_NODE;
  aux_data.gyro2.addr = GYRO2_CH;
  aux_data.gyro2.data = GYRO2_DATA;
  //
  aux_data.gyro3.node = GYRO3_NODE;
  aux_data.gyro3.addr = GYRO3_CH;
  aux_data.gyro3.data = GYRO3_DATA;
  //
  aux_data.lat.node = LAT_NODE;
  aux_data.lat.addr = LAT_CH;
  aux_data.lat.data = LAT_DATA;
  //
  aux_data.lon.node = LON_NODE;
  aux_data.lon.addr = LON_CH;
  aux_data.lon.data = LON_DATA;

  /* **************************** */
  aux_data.cam0_index   = aux_data.cam1_index   = 0;
  aux_data.cam0_counter = aux_data.cam1_counter = 0;
  
  return;
}

static void HandleAuxData(unsigned *data) 
{
  static unsigned auxdata;
  // Handle aux data
  auxdata = 0;
  if(GET_NODE(*data) == aux_data.cam0_trigger.node && 
     GET_CH(*data)   == aux_data.cam0_trigger.addr) {
    if((BBC_DATA(*data) & 0xc000) != aux_data.cam0_index) {
      aux_data.cam0_index   = BBC_DATA(*data) & 0xc000;
      aux_data.cam0_counter = BBC_DATA(*data) & 0x3fff;
      if(aux_data.cam0_counter & 1) aux_data.cam0_counter++;
    }
    PushFifo(data);
  } else if(GET_NODE(*data) == aux_data.cam1_trigger.node && 
	    GET_CH(*data)   == aux_data.cam1_trigger.addr) {
    if((BBC_DATA(*data) & 0xc000) != aux_data.cam1_index) {
      aux_data.cam1_index   = BBC_DATA(*data) & 0xc000;
      aux_data.cam1_counter = BBC_DATA(*data) & 0x3fff;
      if(aux_data.cam1_counter & 1) aux_data.cam1_counter++;
    }
    PushFifo(data);
  } else if(GET_NODE(*data) == aux_data.cam0_pulse.node && 
	    GET_CH(*data)   == aux_data.cam0_pulse.addr) {
    auxdata  = BBC_DATA(aux_data.cam0_pulse.data);
    auxdata |= BBC_NODE(aux_data.cam0_pulse.node);
    auxdata |= BBC_CH(aux_data.cam0_pulse.addr);
    auxdata |= BBC_READ | BBC_WRITE;
    PushFifo(data);
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.cam1_pulse.node && 
		GET_CH(*data)   == aux_data.cam1_pulse.addr) {
    auxdata  = BBC_DATA(aux_data.cam1_pulse.data);
    auxdata |= BBC_NODE(aux_data.cam1_pulse.node);
    auxdata |= BBC_CH(aux_data.cam1_pulse.addr);
    auxdata |= BBC_READ | BBC_WRITE;
    PushFifo(data);
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.gyro1.node && 
		GET_CH(*data)   == aux_data.gyro1.addr) {
    auxdata  = BBC_DATA(aux_data.gyro1.data);
    auxdata |= BBC_NODE(aux_data.gyro1.node);
    auxdata |= BBC_CH(aux_data.gyro1.addr);
    auxdata |= BBC_READ | BBC_WRITE;
    PushFifo(data);
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.gyro2.node && 
		GET_CH(*data)   == aux_data.gyro2.addr) {
    auxdata  = BBC_DATA(aux_data.gyro2.data);
    auxdata |= BBC_NODE(aux_data.gyro2.node);
    auxdata |= BBC_CH(aux_data.gyro2.addr);
    auxdata |= BBC_READ | BBC_WRITE;
    PushFifo(data);
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.gyro3.node && 
		GET_CH(*data)   == aux_data.gyro3.addr) {
    auxdata  = BBC_DATA(aux_data.gyro3.data);
    auxdata |= BBC_NODE(aux_data.gyro3.node);
    auxdata |= BBC_CH(aux_data.gyro3.addr);
    auxdata |= BBC_READ | BBC_WRITE;
    PushFifo(data);
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.lat.node && 
		GET_CH(*data)   == aux_data.lat.addr) {
    auxdata  = BBC_DATA(aux_data.lat.data);
    auxdata |= BBC_NODE(aux_data.lat.node);
    auxdata |= BBC_CH(aux_data.lat.addr);
    auxdata |= BBC_WRITE;
    // this is a loop channel
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.lat.node && 
		GET_CH(*data)   == aux_data.lat.addr+1) {
    auxdata  = BBC_DATA((aux_data.lat.data >> 16));
    auxdata |= BBC_NODE(aux_data.lat.node);
    auxdata |= BBC_CH((aux_data.lat.addr+1));
    auxdata |= BBC_WRITE;
    // this is a loop channel
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.lon.node && 
		GET_CH(*data)   == aux_data.lon.addr) {
    auxdata  = BBC_DATA(aux_data.lon.data);
    auxdata |= BBC_NODE(aux_data.lon.node);
    auxdata |= BBC_CH(aux_data.lon.addr);
    auxdata |= BBC_WRITE;
    // this is a loop channel
    PushFifo(&auxdata);
  } else if(GET_NODE(*data) == aux_data.lon.node && 
	    GET_CH(*data)   == aux_data.lon.addr+1) {
    auxdata  = BBC_DATA((aux_data.lon.data >> 16));
    auxdata |= BBC_NODE(aux_data.lon.node);
    auxdata |= BBC_CH((aux_data.lon.addr+1));
    auxdata |= BBC_WRITE;
    // this is a loop channel
    PushFifo(&auxdata);
  } else {
    PushFifo(data);
  }
}

static void timer_callback(unsigned long dummy)
{
  static unsigned data;
  static unsigned wordcount = 0;
  static unsigned char bout;
  static int done;

  bbc_data.framecounter++;
  
  // Check if the tx buffer has been initialized
  if (bbc_rfifo.status & FIFO_ENABLED) {

    /* Handle Camera Trigger */
    if(aux_data.cam0_counter>0) {
      aux_data.cam0_counter--;
      aux_data.cam0_pulse.data = 1;
    } else {
      aux_data.cam0_pulse.data = 0;
    }
    if(aux_data.cam1_counter>0) {
      aux_data.cam1_counter--;
      aux_data.cam1_pulse.data = 1;
    } else {
      aux_data.cam1_pulse.data = 0;
    }

    bout  = aux_data.cam0_pulse.data;
    bout |= aux_data.cam1_pulse.data << 1; 
    outb(bout, PARALLEL_BASE);
    wmb();


    // BB1    
    done = (*bbc_data.bb1_ptr == BBC_ENDWORD) ? 1 : 0;
    while(atomic_read(&bbc_rfifo.n) < BBC_RFIFO_SIZE && !done) {
      data =  *bbc_data.bb1_data;
      if(wordcount == 1) {
	data = (data & 0xffff0000) | (bbc_data.framecounter & 0x0000ffff); 
      } else if (wordcount == 2) {
	data = (data & 0xffff0000) | ((bbc_data.framecounter >> 16) & 0x0000ffff);
      }
      
      HandleAuxData(&data);

      bbc_data.bb1_data++;
      wordcount++;
      
      if(*bbc_data.bb1_data == 0) 
	bbc_data.bb1_data = bbc_data.bb1_ptr;      

      if(*bbc_data.bb1_data & BBC_FSYNC) {
	wordcount = 0;
	done = 1;
      }
    }

    // BB2    
    done = (*bbc_data.bb2_ptr == BBC_ENDWORD) ? 1 : 0;
    while(atomic_read(&bbc_rfifo.n) < BBC_RFIFO_SIZE && !done) {
      data =  *bbc_data.bb2_data;
      
      PushFifo(&data);
      
      bbc_data.bb2_data++;
      
      if(*bbc_data.bb2_data == BBC_ENDWORD) 
	bbc_data.bb2_data = bbc_data.bb2_ptr;

      if(*bbc_data.bb2_data & BBC_FSYNC) 
	done = 1;
      
    }
  
    if (atomic_read(&bbc_rfifo.n))
      wake_up_interruptible(&bbc_read_wq);

  } 
  
  bbc_drv.timer.expires = jiffies + 1;
  add_timer(&bbc_drv.timer);
}


int bbc_open(struct inode *inode, struct file *filp)
{
  int k;

  if (bbc_rfifo.status & FIFO_ENABLED) return -ENODEV;
  
  bbc_rfifo.i_in = bbc_rfifo.i_out = 0;
  atomic_set(&bbc_rfifo.n, 0);
  bbc_rfifo.status |= FIFO_ENABLED;
  

  bbc_data.bb1_data = bbc_data.bbc_data;
  bbc_data.bb2_data = bbc_data.bbc_data+BBCPCI_MAX_FRAME_SIZE;
  bbc_data.bb1_ptr = bbc_data.bbc_data;
  bbc_data.bb2_ptr = bbc_data.bbc_data+BBCPCI_MAX_FRAME_SIZE;
  bbc_data.framecounter = 0;
  
  for(k = 0; k < BBCPCI_MAX_FRAME_SIZE; k++) {
    bbc_data.bb1_data[k] = 0;
    bbc_data.bb2_data[k] = 0;
  }
    
  bbc_drv.use_count++;

  if (bbc_drv.use_count && bbc_drv.timer_on == 0) {
    // Enable timer.
    bbc_drv.timer_on = 1;
    bbc_drv.timer_event = 0;
    init_timer(&bbc_drv.timer);
    bbc_drv.timer.function = timer_callback;
    bbc_drv.timer.expires = jiffies + 1;
    add_timer(&bbc_drv.timer);
  }

  return 0;
}

int bbc_release(struct inode *inode, struct file *filp)
{
  bbc_rfifo.status = FIFO_DISABLED;
  
  bbc_drv.use_count--;

  if(bbc_drv.use_count == 0 && bbc_drv.timer_on) {
    bbc_drv.timer_on = 0;
    del_timer_sync(&bbc_drv.timer);
  }

  return 0;
}


ssize_t bbc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  size_t i;
  size_t to_read;
  void *out_buf = (void *)buf;
  unsigned long dum;

  if (!access_ok(VERIFY_WRITE, (void *)buf, count)) {
    printk(KERN_WARNING "%s: (read) error accessing user space memory\n", DRV_NAME);
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

ssize_t bbc_write(struct file *filp, const char __user *buf, 
		  size_t count, loff_t *f_pos)
{
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
    
    if(data[0] >= 2*BBCPCI_MAX_FRAME_SIZE) {
      printk(KERN_WARNING "BBC buffer overflow: mcp error\n");
    } else {
      bbc_data.bbc_data[data[0]] = data[1];
    }
  }

  return (2 * i * BBCPCI_SIZE_UINT);
}

int bbc_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
  int retval = 0;

  return retval;
}

struct file_operations bbc_fops = {
  .owner = THIS_MODULE,
  .read  = bbc_read,
  .write = bbc_write,
  .ioctl = bbc_ioctl,
  .open  = bbc_open,
  .release = bbc_release,
};


static int __init bbc_init(void)
{
  int result;
  dev_t dev = 0;
  struct resource  *res;

  if (bbc_major) {
    dev = MKDEV(bbc_major, bbc_minor);
    result = register_chrdev_region(dev, 1, DRV_NAME);
  } else {
    result = alloc_chrdev_region(&dev, bbc_minor, 1, DRV_NAME);
    bbc_major = MAJOR(dev);
  }

  if (result < 0) {
    printk(KERN_WARNING "simbbc: can't get major %d\n", bbc_major);
    return result;
  } 


  // Register this as a character device
  dev = MKDEV(bbc_major, bbc_minor);
  printk(KERN_DEBUG DRV_NAME " major: %d minor: %d dev: %d\n", 
	 bbc_major, bbc_minor, dev);

  cdev_init(&bbc_drv.bbc_cdev, &bbc_fops);
  bbc_drv.bbc_cdev.owner = THIS_MODULE;
  bbc_drv.bbc_cdev.ops   = &bbc_fops;
  result = cdev_add(&bbc_drv.bbc_cdev, dev, 1);
  if (result < 0) {
    printk(KERN_WARNING DRV_NAME " -failed to registed the bbc_pci device\n");
    unregister_chrdev_region(dev, 1);
    return result;
  }

  // Register parallel port
  res = request_region(PARALLEL_BASE, 1, DRV_NAME);
  if(res == NULL) {
    printk(KERN_WARNING DRV_NAME " failed to registed the parallel port\n");
    cdev_del(&bbc_drv.bbc_cdev);
    unregister_chrdev_region(dev, 1);
    return -ENODEV;
  }

  // Initialize data
  bbc_drv.timer_on = 0;
  bbc_drv.use_count = 0;
  bbc_rfifo.status = FIFO_DISABLED;
  InitAuxData();

  printk(KERN_NOTICE "%s: driver initialized\n", DRV_NAME);

  return 0;
}

static void __exit bbc_cleanup(void)
{
  dev_t devno = 0;

  // Release parallel port
  release_region(PARALLEL_BASE, 1);
  
  devno = MKDEV(bbc_major, bbc_minor);
  
  cdev_del(&bbc_drv.bbc_cdev);
  
  unregister_chrdev_region(devno, 1);

  
  printk(KERN_NOTICE "%s: driver removed\n", DRV_NAME); 

}



module_init(bbc_init);
module_exit(bbc_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Blast 2009");
MODULE_DESCRIPTION("bbc_pci: a driver to simulates the pci Blast Bus Controller");
MODULE_ALIAS_CHARDEV_MAJOR(BBC_MAJOR);
MODULE_ALIAS("/dev/bbcpci");

module_param(bbc_major, int, S_IRUGO);
module_param(bbc_minor, int, S_IRUGO);

MODULE_PARM_DESC(bbc_major, " bbc major number");
MODULE_PARM_DESC(bbc_minor, " bbc minor number");
