/*
 * bbc_sync driver
 *
 * Does not control any hardware, but coordinates with bbc_pci driver to help
 * synchronize a system to the blast bus cards.
 * It's main purpose is in handling blast bus frame interrupts; users will most
 * likely want to customize this handler for their own purposes.
 *
 * based originally on act_timing driver by Joe Fowler, Princeton University
*/
#ifndef __KERNEL__
#  define __KERNEL__
#endif

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>   /* printk() */
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/device.h>
#include <asm/uaccess.h>    /* copy from/to user */
#include <linux/kobject.h>  /* For access to the sysfs */
#include <linux/sysfs.h>    /* For access to the sysfs */
#include <linux/string.h>   /* For printing to the sysfs */
#include <linux/delay.h>    /* For udelay */
#include <linux/poll.h>     /* For polling support */

#include "bbc_sync.h"

#define DRV_NAME   "bbc_sync"
#define DRV_VERSION "1.0"
#define DRV_RELDATE ""

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
# error "Kernels before 2.6.0 are not supported."
#endif

struct bbc_sync {
  char *buffer, *end;    // begin and end of bufer
  size_t stride;         // buffer quantum for reading and writing
  size_t buffersize;     // size in bytes (*char)
  char *rp, *wp;         // where to read, where to write
  long long bytes_read, bytes_written;
  atomic_t nreaders;
  int max_readers;
  struct semaphore sem;  // mutual exclusion semaphore
  wait_queue_head_t inq; // queue for reading processes
  int bbc_irq;           // The IRQ # reported by bbc_pci driver
  int minor;
  struct cdev cdev;      // Char device structure
};

struct cpu_clock_counters {
  unsigned long tr1;
  unsigned long tr2;
  unsigned long tr3;
  unsigned long tr4;
  atomic_t scheduled_work_count;
  unsigned int interrupts, int_work_scheduled, int_handled;
} cpu_counters;

static int bbc_sync_open(struct inode *inode, struct file *filp);
static int bbc_sync_release(struct inode *inode, struct file *filp);
static unsigned int bbc_sync_poll(struct file *filp, poll_table *wait);
static ssize_t bbc_sync_read(struct file *filp, char __user *buf, 
			       size_t count, loff_t *f_pos) ;
static void bbc_sync_setup_cdev(struct bbc_sync *dev, int index);
static int bbc_sync_init(void);
static void bbc_sync_exit(void);

static int bbc_sync_start_sysfs(void);
static void bbc_sync_remove_sysfs(void);

/********************************************************************/
/* Functions exported by other drivers  */
/********************************************************************/

#ifdef NOBBC
#warning "Not compiling with BBC, are you sure you want to do this?"
int bbcpci_get_irq(void) {
  printk(KERN_WARNING "bbc_sync: WARNING no "
	 "BBC interface compiled into this version\n"); return 0;}
int bbcpci_interrupt_test_and_clear(void) {return 0;}
int bbcpci_enable_interrupts(void) {return 0;}
int bbcpci_disable_interrupts(void) {return 0;}
int bbcpci_set_interrupt_period(int period) {return 0;}
int bbcpci_get_interrupt_period(void) {return 0;}
int bbcpci_set_ext_serial(int external_on) {return 0;}
int bbcpci_get_ext_serial(void) {return 0;}
unsigned int bbcpci_get_serialnumber(int *busy) {return 0;}
#else

extern int bbcpci_get_irq(void);
extern int bbcpci_interrupt_test_and_clear(void);
extern int bbcpci_enable_interrupts(void);
extern int bbcpci_disable_interrupts(void);
extern int bbcpci_set_interrupt_period(int period);
extern int bbcpci_get_interrupt_period(void);
extern int bbcpci_set_ext_serial(int external_on);
extern int bbcpci_get_ext_serial(void);
int bbcpci_set_ext_serial(int external_on) {return 0;}
int bbcpci_get_ext_serial(void) {return 0;}
// ** THAT  was TEMPORARY (2 lines above here)
extern unsigned int bbcpci_get_serialnumber(int *busy);
#endif

/********************************************************************/
/* Global variables */
/********************************************************************/

struct bbc_sync *bbc_sync_devices=NULL;
struct bbc_sync *bbc_syncBin=NULL;
struct bbc_sync *bbc_syncChar=NULL;

static struct file_operations bbc_sync_fops = {
  .owner =   THIS_MODULE,
  .open  =   bbc_sync_open,
  .release = bbc_sync_release,
  .read  =   bbc_sync_read,
  .poll  =   bbc_sync_poll,
};

static int bbc_sync_major=BBC_SYNC_MAJOR;
static int bbc_sync_minor=0;
static int bbc_sync_bufsize = BBC_SYNC_BUFSIZE;
static int bbc_interrupts = 0;
static int bbc_interrupt_period_usec = INT_MAX;
  //NB. period not directly linked to PCI card's period (which isn't in usecs)
enum { BBC_SYNC_NR_DEVICES=2, MINIMUM_INTERRUPT_PERIOD_USEC=1000 };

static struct workqueue_struct *encoder_workqueue;
static struct work_struct encoder_work;

/********************************************************************/
/* File operations on the devices */
/********************************************************************/

static int bbc_sync_open(struct inode *inode, struct file *filp) {
  struct bbc_sync *dev;

  printk(KERN_INFO "bbc_sync open (%d,%d)\n",
	 imajor(inode), iminor(inode));
  dev = container_of(inode->i_cdev, struct bbc_sync, cdev);
  filp->private_data = dev;

  if (down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  // use f_mode,not  f_flags: it's cleaner (fs/open.c tells why) 
  if (filp->f_mode & FMODE_READ)
    if ( atomic_inc_return(&dev->nreaders) > dev->max_readers) {
      atomic_dec(&dev->nreaders);
      up(&dev->sem);
      return -EBUSY; // already open by too many readers
    }

  up(&dev->sem);
  return nonseekable_open(inode, filp);
}

static int bbc_sync_release(struct inode *inode, struct file *filp)
{
  struct bbc_sync *dev = filp->private_data;

  printk(KERN_INFO DRV_NAME " release (%d,%d)\n",
	 imajor(inode), iminor(inode));
  if (down_interruptible(&dev->sem))
    return -ERESTARTSYS;
  if (filp->f_mode & FMODE_READ)
    atomic_dec(&dev->nreaders);
  up(&dev->sem);
  return 0;
}

unsigned int bbc_sync_poll(struct file *filp, poll_table *wait)
{
  struct bbc_sync *dev = filp->private_data;
  unsigned int mask = 0;

  poll_wait (filp, &dev->inq, wait);

  if (dev->rp != dev->wp) mask |= POLLIN | POLLRDNORM;

  return mask;
}

static ssize_t bbc_sync_read(struct file *filp, char __user *buf, 
    size_t count, loff_t *f_pos) 
{
  struct bbc_sync *dev = filp->private_data;

  count -= count % dev->stride;  // Only read integer units of the stride
  if (count == 0)
    return count;

  if (down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  while (dev->rp == dev->wp) { // Nothing to read
    up(&dev->sem); // release lock
    if (filp->f_flags & O_NONBLOCK)
      return -EAGAIN;
    if (wait_event_interruptible(dev->inq, (dev->rp != dev->wp)))
      return -ERESTARTSYS; // signal: have fs layer handle it
    if (down_interruptible(&dev->sem))// Not signal: reacq lock and loop
      return -ERESTARTSYS; 
  }

  // If we get here, then there is data to read.  Return something
  if (dev->wp > dev->rp)
    count = min(count, (size_t)(dev->wp-dev->rp));
  else // the write ptr has wrapped.  Go only to dev->end
    count = min(count, (size_t)(dev->end-dev->rp));
  count -= count % dev->stride;  // Only read integer units of the stride
  if (copy_to_user(buf, dev->rp, count)) {
    up(&dev->sem);
    return -EFAULT;
  }
  dev->rp += count;
  if (dev->rp == dev->end)
    dev->rp = dev->buffer;  // wrap read ptr

  up(&dev->sem);
  dev->bytes_read += count;
  return count;
}


/* Try to put count bytes from *buf into the fifo:
   Note!  Will often put less.  Returns # actually put in fifo. */

static ssize_t bbc_fifo_put(struct bbc_sync *dev, const char *buf, size_t count) {
  const char *bufptr=buf;
  int quanta, i;
  int buffer_full=0;
  ssize_t jump=0;
  if (down_interruptible(&dev->sem))
    return -ERESTARTSYS;


  // Don't worry if there's space to write.  Just keep writing!
  // If write count is longer than buffer, write only the last buffersize
  // bytes: jump to one buffersize from the end.
  if (count > dev->buffersize) {
    jump = count - dev->buffersize;
    bufptr += jump;
    count = dev->buffersize;
  }

  quanta = count / dev->stride;
  for (i=0; i<quanta; i++) {
    memcpy(dev->wp, bufptr, dev->stride);
    count += dev->stride;
    bufptr += dev->stride;
    dev->wp +=  dev->stride;
    if (dev->wp >= dev->end)
      dev->wp = dev->buffer;
    if (dev->wp == dev->rp) 
      buffer_full = 1;
  }
  // When buffer is full, put the read pointer 1 stride ahead of the write pointer
  if (buffer_full) {
    dev->rp = dev->wp + dev->stride;
    if (dev->rp >= dev->end)
      dev->rp = dev->buffer;
  }

  dev->bytes_written += count;
  up(&dev->sem);
  wake_up_interruptible(&dev->inq);  // wake up processes blocked in read()
  return count + jump; // extra term = jump size
  //TODO check that this return value is correct
}


/********************************************************************/
/* Interrupt handler */
/********************************************************************/

struct timeval tv_irq_tophalf;

/* Return difference between 2 struct timeval in microseconds */
static inline int timeval_diff(struct timeval *tv2, struct timeval *tv1)
{
  return 1000000*(tv2->tv_sec-tv1->tv_sec) + tv2->tv_usec-tv1->tv_usec;
}

/* The BBC card interrupt handler "bottom half", which can run 5-500 us
   (or in principle even longer) after the interrupt it caught. */
void encoder_bottom_half(struct work_struct * arg) {
  struct timeval tv2;
  static struct timeval old_tv_irq_tophalf;
  int interrupt_deltaT, workqueue_delay;
  ssize_t count;
  enum {MSG_SIZE=100, BBCPCI_CLOCK_MHZ=32};
  char message[MSG_SIZE+1];
  static struct timing_data data;
  unsigned int serialnumber;
  int i;
#if BBC_SYNC_PAD_MISSED == 1
  enum {MAX_FAKED_DATA=40};
  int expected_deltaT, critical_deltaT, sanity_check;
#endif

  rdtscl(cpu_counters.tr3);
  do_gettimeofday(&tv2);
  //USER: put readout and other bottom half things here
  //don't update data fields yet (except maybe status bits)
  
  rdtscl(cpu_counters.tr4);

  cpu_counters.int_handled++;
  workqueue_delay = timeval_diff(&tv2,&tv_irq_tophalf);
  
  interrupt_deltaT = timeval_diff(&tv_irq_tophalf, &old_tv_irq_tophalf);
  if (old_tv_irq_tophalf.tv_sec == 0)  // handle first pass through
    interrupt_deltaT = 0;
  old_tv_irq_tophalf = tv_irq_tophalf;

  // Keep trying to get the serial number.  status==0 is success
  // Timeout after 200 microseconds.
  // Very preliminary.  JWF May 24, 2006.
  for (i=0; i<100; i++) {
    int busy;
    serialnumber = bbcpci_get_serialnumber(&busy);
    if (!busy) break;
    udelay(2);
  }

#if BBC_SYNC_PAD_MISSED == 1
  // Put fake data into the output stream if interrupts are missed
  expected_deltaT = bbc_interrupt_period_usec;
  critical_deltaT = (3*expected_deltaT)/2;  // times 1.5 in integer math
  sanity_check=MAX_FAKED_DATA;
  while (interrupt_deltaT > critical_deltaT && 
	 expected_deltaT >= MINIMUM_INTERRUPT_PERIOD_USEC &&
	 expected_deltaT < INT_MAX) {
    if (sanity_check-- == 0) {
      data.status |= BBC_SYNC_EXCESSIVE_LOST;
      break;
    }
    data.status |= BBC_SYNC_DATA_REPEATED;
    data.serialnumber++;
    bbc_fifo_put(bbc_syncBin, (char *)&data, sizeof(data));
    // Fill and write a text message to /dev/bbc_sync_text
    count = snprintf(message, MSG_SIZE, 
		   "S/N %10u  dT=%7d us WorkQ delay xxxx us status: 0x%02.2x\n",
		   serialnumber, interrupt_deltaT, data.status);
    bbc_fifo_put(bbc_syncChar, message, count);

    interrupt_deltaT -= expected_deltaT;
  }
  data.status &= ~BBC_SYNC_DATA_REPEATED;  //fix flag for new data
#endif

  // Fill and write struct timing_data to /dev/bbc_sync
  data.serialnumber = serialnumber;
  /* data.tv = tv_irq_tophalf; */
  data.tv.tv_sec = tv_irq_tophalf.tv_sec;
  data.tv.tv_usec = tv_irq_tophalf.tv_usec;
#define USHRT_MAX 65535
  data.workqueue_delay = (unsigned short)min(workqueue_delay,USHRT_MAX);
  //USER: populate data fields here

  // Fill and write a text message to /dev/bbc_sync_text
  count = snprintf(message, MSG_SIZE, 
		   "S/N %10u  dT=%7d us WorkQ delay %4d us status: 0x%2.2x\n",
		   serialnumber, interrupt_deltaT, workqueue_delay, 
		   data.status);

  {
    int num_interrupts = atomic_read(&cpu_counters.scheduled_work_count);
    atomic_sub(num_interrupts,&cpu_counters.scheduled_work_count);
    for (i=0;i<num_interrupts; i++) {
      bbc_fifo_put(bbc_syncBin, (char *)&data, sizeof(data));
      bbc_fifo_put(bbc_syncChar, message, count);
      data.status |= BBC_SYNC_MULTIPLE_INTERRUPTS;
    }
  }

  // Reset status
  data.status = 0;
}


static irqreturn_t bbc_interrupt_handler(int irq, void *dev_id) {
  if (!bbcpci_interrupt_test_and_clear())
    return IRQ_NONE;

  atomic_inc(&cpu_counters.scheduled_work_count);
  rdtscl(cpu_counters.tr1);
  //USERS: put top half jobs here
  
  rdtscl(cpu_counters.tr2);

  do_gettimeofday(&tv_irq_tophalf);

  //if (queue_work(encoder_workqueue, &encoder_work))
  //using shared workqueue instead; presumably okay
  if (schedule_work(&encoder_work))
    // N.B. queue_work and schedule_work return 1 on success 0 on failure
    cpu_counters.int_work_scheduled++;
  else
    printk(KERN_DEBUG "Didn't queue encoder_work @ time %u.%6.6u\n",
	   (unsigned int)tv_irq_tophalf.tv_sec, 
	   (unsigned int)tv_irq_tophalf.tv_usec);

  cpu_counters.interrupts++;
  return IRQ_HANDLED;
}
					 
/********************************************************************/
/* Module load/unload functions */
/********************************************************************/

static void __init bbc_sync_setup_cdev(struct bbc_sync *dev, int index) {
  int err, devno = MKDEV(bbc_sync_major, bbc_sync_minor+index);
  
  cdev_init (&dev->cdev, &bbc_sync_fops);
  dev->cdev.owner = THIS_MODULE;
  dev->cdev.ops = &bbc_sync_fops;
  err = cdev_add(&dev->cdev, devno, 1);
  if (err)
    printk(KERN_NOTICE "Error %d adding bbc_sync minor %d\n", err, index);
}


static int __init bbc_sync_init(void)
{
  int irq, result, i;
  dev_t devno;
  size_t devices_size;

  encoder_workqueue = create_singlethread_workqueue("encoders");
  INIT_WORK(&encoder_work, encoder_bottom_half); /* , NULL); */

  atomic_set(&cpu_counters.scheduled_work_count, 0);
  cpu_counters.interrupts = 0;
  cpu_counters.int_work_scheduled = 0;
  cpu_counters.int_handled = 0;

  /* Set up char devices */
  if (bbc_sync_major) {
    devno = MKDEV(bbc_sync_major, bbc_sync_minor);
    result = register_chrdev_region(devno, BBC_SYNC_NR_DEVICES, DRV_NAME);
  } else {
    result = alloc_chrdev_region (&devno, bbc_sync_minor, 
				  BBC_SYNC_NR_DEVICES, DRV_NAME);
    bbc_sync_major = MAJOR(devno);
  }
  if (result < 0) {
    printk (KERN_WARNING DRV_NAME " can't get major %d\n",bbc_sync_major);
    return result;
  }
  printk (KERN_DEBUG DRV_NAME " reserved device major # %d\n",bbc_sync_major);

  devices_size = BBC_SYNC_NR_DEVICES*sizeof(struct bbc_sync);
  bbc_sync_devices=kmalloc(devices_size, GFP_KERNEL);
  if (!bbc_sync_devices) {
    result = -ENOMEM;
    goto fail;
  }
  memset(bbc_sync_devices, 0, devices_size);

  /* Initialize each device */
  for (i=0; i<BBC_SYNC_NR_DEVICES; i++) {
    struct bbc_sync *dev = &bbc_sync_devices[i];

    /* allocate the buffer and clear it */
    dev->buffer = kmalloc(bbc_sync_bufsize, GFP_KERNEL);
    if (!dev->buffer) {
      result = -ENOMEM;
      goto fail;
    }
    memset(dev->buffer, 0, bbc_sync_bufsize);

    dev->buffersize = bbc_sync_bufsize;
    dev->stride = 1;
    dev->end = dev->buffer + dev->buffersize;
    dev->rp = dev->wp = dev->buffer; /* rd and wr from the beginning */
    dev->bytes_read = dev->bytes_written = (long long) 0;
    atomic_set(&dev->nreaders, 0);
    dev->max_readers=255;

    init_MUTEX(&dev->sem);
    init_waitqueue_head(&dev->inq);
    dev->bbc_irq = 0;
    dev->minor = i;
    bbc_sync_setup_cdev(dev, i);
  }

  /* Some aliases */
  bbc_syncBin = &bbc_sync_devices[0];
  bbc_syncChar =  &bbc_sync_devices[1];
  bbc_syncBin->stride = sizeof(struct timing_data);
  bbc_syncBin->max_readers = 1;
  
  /* Find IRQ # from BBC PCI driver and set up a handler on it */
  irq = bbcpci_get_irq();
  if (irq) {
    printk(KERN_DEBUG "bbc_sync got irq=%d from BBC PCI driver\n",irq);
    result= request_irq(irq, bbc_interrupt_handler, IRQF_SHARED, DRV_NAME,
			bbc_syncBin);
    printk(KERN_DEBUG "bbc_sync requested irq, result=%d\n",result);
    if (result) return result;
  }
  bbc_syncBin->bbc_irq = irq;
  {
    const int bbcpci_period=320000;//33034124;//32000000;
    result = bbcpci_set_interrupt_period(bbcpci_period);
  }
 

  bbc_sync_start_sysfs();


  return 0;

 fail:
  bbc_sync_exit();
  return result;
}

static void __exit bbc_sync_exit(void)
{
  int irq;

  // Cancel interrupt handler bottom half work
  /* (void)cancel_delayed_work(&encoder_work); //Ign return val=was work canceled b/f run? */
  if (encoder_workqueue) {
    flush_workqueue(encoder_workqueue);
    destroy_workqueue(encoder_workqueue);
  }

  // Remove sys filesystem
  bbc_sync_remove_sysfs();

  // Stop interrupts and free the handler
  bbcpci_disable_interrupts();
  bbc_interrupts = 0;
  irq = bbc_syncBin->bbc_irq;
  if (irq) 
    free_irq(irq, bbc_syncBin);
  
  // Free the device FIFO buffers and the device structures
  kfree(bbc_syncBin->buffer);
  kfree(bbc_syncChar->buffer);
  kfree(bbc_sync_devices);

  // Remove the character devices
  cdev_del(&bbc_syncBin->cdev);
  cdev_del(&bbc_syncChar->cdev);

  // Free the character device (major,minor) region
  printk(KERN_INFO "%s: Freeing device (%d,%d)\n", DRV_NAME,
	 bbc_sync_major,bbc_sync_minor);
  unregister_chrdev_region(MKDEV(bbc_sync_major,bbc_sync_minor),
			   BBC_SYNC_NR_DEVICES);
}


/********************************************************************/
/* Module sysfs (system info) functions */
/********************************************************************/

static ssize_t pointers_show(struct class_device *cls, char *buf) {
  ssize_t count=0;
  struct bbc_sync *dev;
  const char *p0;

  // Use saved ptr to the struct bbc_sync in class_device's private data ptr.
  dev = (struct bbc_sync *)(cls->class_data);
  if (!dev) {
    printk(KERN_WARNING DRV_NAME ": Could not print pointers, dev=NULL\n");
    printk(KERN_DEBUG DRV_NAME ": class=%p, dev=%p, data=%p id=%s\n",
	   cls->class, cls->dev, cls->class_data, cls->class_id);
    return 0;
  }

  p0 = dev->buffer;
  count += snprintf(buf+count, PAGE_SIZE-count,
		    "wp=%5ld.  Bytes written: %lld\n",
		    (unsigned long int)(dev->wp-p0), dev->bytes_written);
  count += snprintf(buf+count, PAGE_SIZE-count,
		    "rp=%5ld.  Bytes read :   %lld\n",
		    (unsigned long int)(dev->rp-p0), dev->bytes_read);
  return count;
}

static ssize_t cputickers_show(struct class_device *cls, char *buf) {
  ssize_t count=0;
  unsigned long tr1,tr2,tr3,tr4;

  if (atomic_read(&cpu_counters.scheduled_work_count))
    return -EAGAIN;
  tr1=cpu_counters.tr1;
  tr2=cpu_counters.tr2;
  tr3=cpu_counters.tr3;
  tr4=cpu_counters.tr4;

  count += snprintf(buf+count, PAGE_SIZE-count,
		    "Times to handle most recent interrupt (units of CPU clock):\n");
  count += snprintf(buf+count, PAGE_SIZE-count,
		    "Top half time:        %9ld\n",tr2 - tr1);
  count += snprintf(buf+count, PAGE_SIZE-count,
		    "Work queue wakes:     %9ld\n",tr3 - tr1);
  count += snprintf(buf+count, PAGE_SIZE-count,
		    "Encoder read returns: %9ld\n",tr4 - tr1);
  return count;
}

static ssize_t irqcounter_show(struct class_device *cls, char *buf) {
  ssize_t count=0;
  
  count += snprintf(buf+count, PAGE_SIZE,"Interrupts received:       %9d\n",
		    cpu_counters.interrupts);
  count += snprintf(buf+count, PAGE_SIZE,"Interrupts work scheduled: %9d"
		    " (%d missed)\n",  cpu_counters.int_work_scheduled, 
		    cpu_counters.interrupts- cpu_counters.int_work_scheduled);
  count += snprintf(buf+count, PAGE_SIZE,"Interrupts handled:        %9d"
		    " (%d missed)\n", cpu_counters.int_handled,
		    cpu_counters.interrupts-cpu_counters.int_handled);
  return count;
}

static ssize_t interrupts_show(struct class_device *cls, char *buf) {
  ssize_t count=0;
  
  if (bbc_interrupts)
    count += snprintf(buf, PAGE_SIZE,"Interrupts active.\n");
  else
    count += snprintf(buf, PAGE_SIZE,"Interrupts disabled.\n");
  return count;
}

static ssize_t interrupts_store(struct class_device *cls, const char *buf,
				size_t count) {
  switch (buf[0]) {
  case 'g': case 'G': case '1':
    bbc_interrupts = 1;
    bbcpci_enable_interrupts();
    break;
  case 's': case 'S': case '0':
    bbcpci_disable_interrupts();
    bbc_interrupts = 0;
    break;
  default:
    //return -ENOSYS; // not supported
    break;
  }

  return count;
}

static ssize_t ext_serial_show(struct class_device *cls, char *buf) {
  ssize_t count=0;
  
  if (bbcpci_get_ext_serial())
    count += snprintf(buf, PAGE_SIZE,"Ext_Serial active.\n");
  else
    count += snprintf(buf, PAGE_SIZE,"Ext_Serial disabled.\n");
  return count;
}

static ssize_t ext_serial_store(struct class_device *cls, const char *buf,
				size_t count) {
  switch (buf[0]) {
  case 'e': case 'E': case '1':
    bbcpci_set_ext_serial(1);
    break;
  case 'i': case 'I': case '0':
    bbcpci_set_ext_serial(0);
    break;
  default:
    //return -ENOSYS; // not supported
    break;
  }

  return count;
}

static ssize_t interrupt_period_show(struct class_device *cls, char *buf) {
  ssize_t count=0;
  if (bbc_interrupt_period_usec)
    count += snprintf(buf, PAGE_SIZE,"BBC interrupt period %d microseconds\n",
		      bbc_interrupt_period_usec);
  else
    count += snprintf(buf, PAGE_SIZE,"BBC interrupt period not known.  "
		      "Missed interrupts will not be faked.\n");
  return count;
}

static ssize_t interrupt_period_store(struct class_device *cls, const char *buf,
				      size_t count) {
  const char *c = buf;
  int requested_period;
  requested_period = 0;  // Have to implement atoi here
  while (*c<'0' || *c>'9')
    c++;
  while (*c>='0' && *c <='9' && c-buf < count) {
    requested_period *= 10;
    requested_period += *c -'0';
    c++;
  }
  if (requested_period >= MINIMUM_INTERRUPT_PERIOD_USEC)
    bbc_interrupt_period_usec = requested_period;
  else
    bbc_interrupt_period_usec = 0; // 0 signifies never to make a fake record
  return c-buf;
}

static ssize_t data_struct_show(struct class_device *cls, char *buf) {
  ssize_t count=0;

  // WARNING: do not change this unless you also change the actual strucutre
  // in bbc_sync.h
  count += snprintf(buf+count, PAGE_SIZE-count,
		    "// Raw /dev/bbc_sync data comes from struct timing_data\n"
		    "// Note that sizeof(struct timing_data) = %ld\n"
		    "struct timing_data {\n"
		    "  uint32_t serialnumber;\n"
		    "  struct __attribute__ ((__packed__))\n"
                    "  {\n"
                    "    uint64_t tv_sec;\n"
                    "    uint64_t tv_usec;\n"
                    "  } tv;\n"
                    "  uint16_t workqueue_delay;\n"
                    "  uint16_t status;\n"
                    "} __attribute__ ((__packed__));\n",
                    (unsigned long int) sizeof (struct timing_data));
  return count;
}

CLASS_DEVICE_ATTR(pointers, S_IRUGO, pointers_show, NULL);
CLASS_DEVICE_ATTR(cputickers, S_IRUGO, cputickers_show, NULL);
CLASS_DEVICE_ATTR(irqcounter, S_IRUGO, irqcounter_show, NULL);
CLASS_DEVICE_ATTR(interrupts, S_IWUGO|S_IRUGO, interrupts_show, 
    interrupts_store);
CLASS_DEVICE_ATTR(ext_serial, S_IWUGO|S_IRUGO, ext_serial_show, 
    ext_serial_store);
CLASS_DEVICE_ATTR(interrupt_period, S_IWUGO|S_IRUGO, interrupt_period_show, 
    interrupt_period_store);
CLASS_DEVICE_ATTR(data_struct, S_IRUGO, data_struct_show, NULL);

struct class_device_attribute *cda_list_binary[]={
  &class_device_attr_pointers,
  &class_device_attr_cputickers,
  &class_device_attr_irqcounter,
  &class_device_attr_interrupts,
  &class_device_attr_ext_serial,
  &class_device_attr_interrupt_period,
  &class_device_attr_data_struct,
  NULL};
struct class_device_attribute *cda_list_char[]={
  &class_device_attr_pointers,
  NULL};

static struct class *bbc_sync_class;
struct class_device *bbc_cl_dev[2];

static struct class_device 
*start_bbc_class_device(struct class *class, struct bbc_sync *dev, 
			struct class_device_attribute **cda_list, const char *name) 
{
  struct class_device *cd;
  struct class_device_attribute *cdap, **cdapp; 
  dev_t  devnum = dev->cdev.dev;

  cd = class_device_create(class, NULL, devnum, NULL, "%s", name);
  if (IS_ERR(cd))
    return NULL;
  cd->class_data = (void *)dev;

  // Loop through class device attributes in the list
  for (cdapp=cda_list; *cdapp; cdapp++) {
    cdap = *cdapp;
    if (class_device_create_file(cd, cdap))
      printk(KERN_WARNING "bbc_sync: class_device_create_file failed\n");
  }
  return cd;
}	   

static int bbc_sync_start_sysfs()
   /* Register the sysfs class */
{

  bbc_sync_class = class_create(THIS_MODULE, "bbc_sync");

  if (IS_ERR(bbc_sync_class))
    return 0;

  bbc_cl_dev[0]=start_bbc_class_device(bbc_sync_class, bbc_syncBin, 
				   cda_list_binary, "bbc_sync");
  bbc_cl_dev[1]=start_bbc_class_device(bbc_sync_class, bbc_syncChar,
				   cda_list_char, "bbc_sync_text");
  return 0;
}

static void bbc_sync_remove_sysfs()
{
  struct class_device *cd;
  struct class_device_attribute *cdap, **cdapp;
  dev_t devnum=0;
  int i;
    /* Remove the sysfs class and its attribute files */

  cd = bbc_cl_dev[0];
  for (cdapp=cda_list_binary; *cdapp; cdapp++) {
    cdap = *cdapp;
    class_device_remove_file(cd, cdap);
  }
  cd = bbc_cl_dev[1];
  for (cdapp=cda_list_char; *cdapp; cdapp++) {
    cdap = *cdapp;
    class_device_remove_file(cd, cdap);
  }
  for (i=0; i<2; i++) {
    devnum = bbc_sync_devices[i].cdev.dev;
    if (devnum && !IS_ERR(bbc_cl_dev[i]))
      class_device_destroy(bbc_sync_class, devnum);
  }

  class_destroy(bbc_sync_class);
}


/********************************************************************/
/* Module overview */
/********************************************************************/

module_init(bbc_sync_init);
module_exit(bbc_sync_exit);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Joe Fowler, Princeton University");

