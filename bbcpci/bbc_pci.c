#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/pci.h>

/* bbc.h includes ioctl definitions, etc - stuff needed to use the bbc device */
#include "bbc_pci.h"

/*******************************************************************/
/* Definitions                                                     */
/*******************************************************************/

/* This major number is reserved for local or experimental use... */
/* See Linux Device Drivers 2nd ed, P 58 footnote */
/* Assuming no one else uses this number, this is more convenient */
/* than dynamic major/minor.  Note though, that the BBC_PCI (250) and the */
/* Decom_PCI (251?) and any other board we make must have different majors. */
#define BBC_MAJOR 250
#define BBC_MINOR 0
#define BI0_MINOR 1

#define BBCPCI_VENDOR 0x5045
#define BBCPCI_ID 0x4243

/* this memsize is the amount of pci space we want.  Wishfull thinking....
 * the card only lets us see 16k... */
#define BBCPCI_MEMSIZE 8388608


/* the size of tx_buffer: a bit bigger than the largest possible frame */
/* seems like a good size:  the largest possible frame is ~555 words/bus */
/* the maximum allowed here is 128k/4/2 = 16384 */
/* this is the number of (add, data) pairs.  */
#define TX_BUFFER_SIZE  16380 

/********************************************************************/
/* Define structures and locally global variables                   */
/********************************************************************/

/* 2.6.x: Declare license to prevent tainting kernel */
MODULE_LICENSE("GPL");

/* Jiffies is a global kernel variable which is incremented at 1000 hz */
extern volatile unsigned long jiffies;

/* The address where the PCI card's memory landed */
void *bbcpci_membase = 0;
unsigned long bbcpci_membase_raw = 0;
unsigned long bbcpci_resourceflags = 0;
int cbcounter = 0;

/* The call back timer: used to call the background process to read the */
/* rx fifo.   */
static struct timer_list bbc_cb_timer;
static int timer_on = 0;

/* structure to hold the tx circular buffer.  NIOS on bbc_pci */
/* can only handle 0x100 words at a time.  This holds the rest */
static struct {
  unsigned int *data;
  unsigned int *add;
  int i_in; // location we are about to write to
  int i_out;// location we are about to read from
  int n;
} tx_buffer;

const int bbc_minor = 0;
const int bi0_minor = 1;

/********************************************************************/
/*                                                                  */
/*  Timer Callback: called at 1 khz to copy data from the tx buf    */
/*    to the bbc_pci card                                           */
/*                                                                  */
/********************************************************************/
static void timer_callback(unsigned long x) {
  unsigned int write_buf_p;
  unsigned int n_written;

  if (readl(bbcpci_membase + ADD_COMREG) & (COMREG_RESET)) {
    cbcounter = 0;
  } else {
    write_buf_p = readl(bbcpci_membase + ADD_WRITE_BUF_P);
  
    if (write_buf_p < ADD_IR_WRITE_BUF) { /* if nios is ready for data */
    
      write_buf_p = ADD_IR_WRITE_BUF;
      n_written = 0;
    
      while ((n_written<IR_WRITE_BUF_SIZE) &&
	     (tx_buffer.i_in != tx_buffer.i_out)) {
      
	writel(tx_buffer.add[tx_buffer.i_out], bbcpci_membase + write_buf_p);
	write_buf_p += sizeof(unsigned int);
	writel(tx_buffer.data[tx_buffer.i_out], bbcpci_membase + write_buf_p);
	write_buf_p += sizeof(unsigned int);
      
	tx_buffer.i_out++;
	if (tx_buffer.i_out >= TX_BUFFER_SIZE) tx_buffer.i_out = 0;

	tx_buffer.n--;
	n_written++;
      }
    
      write_buf_p -= 2 * SIZE_UINT;
      writel((unsigned int)write_buf_p, bbcpci_membase + ADD_WRITE_BUF_P);
    } 
  }
  
  if (timer_on) {
    cbcounter++;

    /* Re-start the timer so this callback gets called again in 1 jiffies. */
    bbc_cb_timer.expires = jiffies + 1;

    add_timer(&bbc_cb_timer);
  }
}

/*******************************************************************/
/*                                                                 */
/*  read_bbc: copies from the memory to the output buffer.         */
/*  Copies just one word, if there are words availible.            */
/*                                                                 */
/*******************************************************************/
static ssize_t read_bbc(struct file * filp, char * buf, size_t count,
                         loff_t *dummy) {
  unsigned int rp, wp, b;
  int minor;
  minor = *((int *)(filp->private_data));

  if (minor == bbc_minor) {
    if (count < SIZE_UINT)
      return 0;
  
    rp = readl(bbcpci_membase + ADD_READ_BUF_RP); // Where pci read last.
    wp = readl(bbcpci_membase + ADD_READ_BUF_WP); // Where nios is about to write.

    rp += SIZE_UINT;
    if (rp >= ADD_READ_BUF_END)
      rp = ADD_READ_BUF;

    if (rp == wp)  // No new data.
      return 0; 
    else {
      b = readl(bbcpci_membase + rp);
      copy_to_user(buf, (void *)&b, SIZE_UINT);
      writel(rp, bbcpci_membase + ADD_READ_BUF_RP); // update read pointer
      return SIZE_UINT;
    }
  } else if (minor == bi0_minor) {
    b = 42;
    copy_to_user(buf, (void *)&b, SIZE_UINT);// FIXME: what should read bi0 do?
    return SIZE_UINT;
  }
  return (0);
}

/*******************************************************************/
/*                                                                 */
/*   write_bbc:  write an (nios address, bbc data) pair to         */
/*     the internal circular buffer                                */
/*     WARNING: we don't check to see if we are over filling       */
/*      we can't see how that could happen...                      */
/*                                                                 */
/*******************************************************************/
static ssize_t write_bbc(struct file * filp, const char * buf,
                         size_t count, loff_t *dummy) {
  unsigned int add, datum;
  int minor;
  minor = *((int *)(filp->private_data));

  if (minor == bbc_minor) {
    if (count<8) return 0;
    if (count>8) count = 8;

    copy_from_user((void *)(&add), buf, SIZE_UINT);
    copy_from_user((void *)(&datum), buf + SIZE_UINT, SIZE_UINT);

    tx_buffer.add[tx_buffer.i_in] = add;
    tx_buffer.data[tx_buffer.i_in] = datum;
  
    if (tx_buffer.i_in+1 >= TX_BUFFER_SIZE) tx_buffer.i_in = 0;
    else tx_buffer.i_in++;
    tx_buffer.n++;
  } else if (minor == bi0_minor) {
    /* FIXME: add bi0 write here */
  }
  
  return count;
}

/*******************************************************************/
/*                                                                 */
/*  open_bbc:                                                      */
/*                                                                 */
/*******************************************************************/
static int open_bbc(struct inode *inode, struct file * filp){
  unsigned int bitfield;
  int minor = MINOR(inode->i_rdev);

  if (minor == bbc_minor) {
    filp->private_data = (void *) &bbc_minor;
    cbcounter = 0;
    /* initialize tx buffer */
    tx_buffer.n = tx_buffer.i_in = tx_buffer.i_out = 0;
    tx_buffer.data =
      (unsigned int *) kmalloc(TX_BUFFER_SIZE *
			       sizeof(unsigned int), GFP_KERNEL);
    tx_buffer.add =
      (unsigned int *) kmalloc(TX_BUFFER_SIZE *
			       sizeof(unsigned int), GFP_KERNEL);
    if (tx_buffer.add == NULL) {
      printk("Error allocating TX_BUFFER\n");
      return -EBUSY;
    }
  
    if (check_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE)) {
      printk("bbc_pci: memory already in use\n");
      return -EBUSY;
    }
    request_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE, "bbc_pci");
  
    bbcpci_membase = ioremap_nocache(bbcpci_membase_raw, BBCPCI_MEMSIZE);

    /* reset the card */
    bitfield = COMREG_RESET;
    writel(bitfield, bbcpci_membase + ADD_COMREG);

    /* Start the timer */
    timer_on = 1;
    init_timer(&bbc_cb_timer);
    bbc_cb_timer.function = timer_callback;
    bbc_cb_timer.expires = jiffies + 1;

    add_timer(&bbc_cb_timer);
            
    return 0;
  } else if (minor == bi0_minor) {
    filp->private_data = (void *) &bi0_minor;
    return 0;
  } else {
    printk("BBC: Invalid minor number\n");
    return -EBUSY;
  }
} 

/*******************************************************************/
/*                                                                 */
/*   release_bbc: free memory, stop timer                          */
/*                                                                 */
/*******************************************************************/
static int release_bbc(struct inode *inode, struct file *filp) {
  int minor = MINOR(inode->i_rdev);

  if (minor == bbc_minor) {
    timer_on = 0;
    del_timer_sync(&bbc_cb_timer);

    kfree(tx_buffer.data);
    kfree(tx_buffer.add);
    iounmap(bbcpci_membase);
    release_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE);
  
    return(0);
  } else { // bi0_monor
    return(0);
  }
}

/*******************************************************************/
/*                                                                 */
/*   ioctl: control bbc card: see bbc.h for definitions            */
/*                                                                 */
/*******************************************************************/
static int ioctl_bbc (struct inode *inode, struct file * filp,
		      unsigned int cmd, unsigned long arg) {
  unsigned int ret = 0;
  unsigned int bitfield;
  unsigned int p[4];
  int i, size = _IOC_SIZE(cmd);
  int minor = MINOR(inode->i_rdev);

  
  if (_IOC_DIR(cmd) & _IOC_READ) {
    if (!access_ok(VERIFY_WRITE, (void *)arg, size))
      return -EFAULT;
  }
  else if (_IOC_DIR(cmd) & _IOC_WRITE) {
    if (!access_ok(VERIFY_READ, (void *)arg, size))
      return -EFAULT;
  }

  if (minor == bbc_minor) {
    switch(cmd) {
    case BBCPCI_IOC_RESET:    /* Reset the BBC board - clear fifo, registers */
      bitfield = COMREG_RESET;
      writel(bitfield, bbcpci_membase + ADD_COMREG);
      break;
    case BBCPCI_IOC_SYNC:     /* Clear read buffers and restart frame. */
      bitfield = COMREG_SYNC;
      writel(bitfield, bbcpci_membase + ADD_COMREG);
      break;
    case BBCPCI_IOC_VERSION:  /* Get the current version. */
      ret = readl(bbcpci_membase + ADD_VERSION);
      break;
    case BBCPCI_IOC_COUNTER:  /* Get the counter. */
      ret = readl(bbcpci_membase + ADD_COUNTER);
      break;
    case BBCPCI_IOC_SECRET:   /* Ssshhh. */
      for (i = 0; i < 4; i++)
	p[i] = readl(bbcpci_membase + (6 + i) * SIZE_UINT);
      ret = copy_to_user((unsigned char *)arg, (unsigned char *)p, 
			 4 * SIZE_UINT);
      break;	
    case BBCPCI_IOC_JIFFIES:  /* A way to read out jiffies... */
      ret = jiffies;  /* doesn't seem to work.  Bo'h? */
      break;
    case BBCPCI_IOC_CBCOUNTER:  /* Callback Counter */
      ret = cbcounter; 
      break;
    case BBCPCI_IOC_WRITEBUF:  /* How many words in the NIOS write buf? */
      ret = readl(bbcpci_membase + ADD_WRITE_BUF_P) - ADD_IR_WRITE_BUF; 
      break;
    case BBCPCI_IOC_COMREG:  /* return the command register */
      /* COMREG_RESET tells if it is resetting */
      ret = readl(bbcpci_membase + ADD_COMREG);
      break;
    case BBCPCI_IOC_READBUF_WP: /* where nios is about to write */
      ret = readl(bbcpci_membase + ADD_READ_BUF_WP);
      break;
    case BBCPCI_IOC_READBUF_RP: /* where the PC is about to read */
      ret = readl(bbcpci_membase + ADD_READ_BUF_RP);
      break;
    case BBCPCI_IOC_WRITEBUF_N: /* how much data still to be read */
      ret = tx_buffer.n;
      break;
    default:
      break;
    }
  } else if (minor == bi0_minor) {
  }
  return ret;
}


/*******************************************************************/
/*                                                                 */
/*   file_operations structure: associates functions with file ops */
/*                                                                 */
/*******************************************************************/
static struct file_operations bbc_fops = {
  owner:   THIS_MODULE,
  llseek:  no_llseek,
  read:  read_bbc,
  write:  write_bbc,
  ioctl:  ioctl_bbc,
  open:  open_bbc,
  release:  release_bbc,
};

/*******************************************************************/
/*                                                                 */
/*   init_bbc: called upon insmod bbc.o                            */
/*                                                                 */
/*******************************************************************/
int init_bbc_pci(void) {
  struct pci_dev *dev = NULL;
  
  /* Register device */
  if (register_chrdev(BBC_MAJOR, "bbc_pci", &bbc_fops) != 0) {
    printk("Unable to get major for bbc_pci device\n");
    return -EIO;
  }
  
  dev = pci_find_device(BBCPCI_VENDOR, BBCPCI_ID, dev);
  if (!dev) {
    printk("Unable to find BBC_PCI card.\n");
    return -EIO;
  }
  pci_enable_device(dev);
  bbcpci_membase_raw = pci_resource_start(dev, 0);
  bbcpci_resourceflags = pci_resource_flags(dev, 0);

  return 0;
}

/*******************************************************************/
/*                                                                 */
/*   cleanup_bbc: called upon rmmod bbc                            */
/*                                                                 */
/*******************************************************************/
void cleanup_bbc_pci(void) { 
  unregister_chrdev(BBC_MAJOR, "bbc_pci");
}

/* Declare module init and exit functions */
module_init(init_bbc_pci);
module_exit(cleanup_bbc_pci);
