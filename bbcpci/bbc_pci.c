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
#include <linux/delay.h>

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

/* This memsize is the amount of PCI space we want.  Wishful thinking . . .
 * The card only lets us see 16k . . . */
#define BBCPCI_MEMSIZE 8388608


/* The size of tx_buffer: a bit bigger than the largest possible frame
 * seems like a good size:  the largest possible frame is ~555 words/bus.
 * The maximum allowed here is 128k/4/2 = 16384.
 * This is the number of (add, data) pairs.  */
#define TX_BUFFER_SIZE  16380 
#define BI0_BUFFER_SIZE (624*25)

#define WRITEL_S(x,y) {writel(x,y);udelay(2);}

/********************************************************************/
/* Define structures and locally global variables                   */
/********************************************************************/

/* 2.6.x: Declare license to prevent tainting kernel. */
MODULE_LICENSE("GPL");

/* Jiffies is a global kernel variable which is incremented at 1000 Hz. */
extern volatile unsigned long jiffies;

/* The address where the PCI card's memory landed. */
void *bbcpci_membase = 0;
unsigned long bbcpci_membase_raw = 0;
unsigned long bbcpci_resourceflags = 0;
int cbcounter = 0;

/* The call back timer: used to call the background process to read the */
/* RX fifo. */
static struct timer_list bbc_cb_timer;
static int timer_on = 0;

/* Structure to hold the tx circular buffer.  NIOS on bbc_pci */
/* can only handle 0x100 words at a time.  This holds the rest. */
static struct {
  unsigned int *data;
  unsigned int *add;
  int i_in;   // Location to which we are about to write.
  int i_out;  // Location from which we are about to read.
  int n;
} tx_buffer;

/* Structure to hold the bi0 circular buffer.  NIOS on bbc_pci */
/* can only handle 0x3ff8 words at a time.  This holds the rest. */
static struct {
  unsigned int *data;
  int i_in;   // Location to which we are about to write.
  int i_out;  // Location from which we are about to read.
  int n;
} bi0_buffer;

char bi0_allocated = 0;
char bbc_allocated = 0;

const int bbc_minor = 0;
const int bi0_minor = 1;

/* Set to 1 the first time a device is opened. (Starts the callback etc.) */
char device_was_opened = 0; 


/********************************************************************/
/*                                                                  */
/*  Timer Callback: called at 1 khz to copy data from the tx buf    */
/*    to the bbc_pci card                                           */
/*                                                                  */
/********************************************************************/

static void timer_callback(unsigned long x) {
  unsigned int write_buf_p;
  unsigned int n_written;

  if (readl(bbcpci_membase + BBCPCI_ADD_COMREG))
    cbcounter = 0;
  else {

    // Write BBC data.
    if (bbc_allocated) { 
      write_buf_p = readl(bbcpci_membase + BBCPCI_ADD_WRITE_BUF_P);
  
      if (write_buf_p < BBCPCI_ADD_IR_WRITE_BUF) { // Is NIOS ready for data?
        write_buf_p = BBCPCI_ADD_IR_WRITE_BUF;
        n_written = 0;
    
        while ((n_written < BBCPCI_IR_WRITE_BUF_SIZE) &&
               (tx_buffer.i_in != tx_buffer.i_out)) {
          WRITEL_S(tx_buffer.add[tx_buffer.i_out], bbcpci_membase + 
                                                   write_buf_p);
          write_buf_p += sizeof(unsigned int);
          WRITEL_S(tx_buffer.data[tx_buffer.i_out], bbcpci_membase + 
                                                    write_buf_p);
          write_buf_p += sizeof(unsigned int);
      
          tx_buffer.i_out++;
          
          if (tx_buffer.i_out >= TX_BUFFER_SIZE)
            tx_buffer.i_out = 0;

          tx_buffer.n--;
          n_written++;
        }
    
        write_buf_p -= 2 * BBCPCI_SIZE_UINT;
        WRITEL_S((unsigned int)write_buf_p, bbcpci_membase +
	                                        BBCPCI_ADD_WRITE_BUF_P);
      }
    }

    // Write BI0 data.
    if (bi0_allocated) {
      while (((write_buf_p = readl(bbcpci_membase + BBCPCI_ADD_BI0_WP)) !=
	           readl(bbcpci_membase + BBCPCI_ADD_BI0_RP)) &&
	           (bi0_buffer.i_in != bi0_buffer.i_out)) {
        WRITEL_S(bi0_buffer.data[bi0_buffer.i_out], bbcpci_membase + 
                                                    write_buf_p);
        write_buf_p += BBCPCI_SIZE_UINT;
        if (write_buf_p >= BBCPCI_IR_BI0_BUF_END)
          write_buf_p = BBCPCI_IR_BI0_BUF;
        WRITEL_S((unsigned int)write_buf_p, bbcpci_membase + BBCPCI_ADD_BI0_WP);
        bi0_buffer.i_out++;
        if (bi0_buffer.i_out >= TX_BUFFER_SIZE)
          bi0_buffer.i_out = 0;

        bi0_buffer.n--;
      }
    } 
  }
  
  if (timer_on) {
    cbcounter++;

    // Re-start the timer so this callback gets called again in 1 jiffies.
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
    if (count < BBCPCI_SIZE_UINT)
      return 0;
  
    rp = readl(bbcpci_membase + BBCPCI_ADD_READ_BUF_RP); // Where pci read last.
    wp = readl(bbcpci_membase + BBCPCI_ADD_READ_BUF_WP); // Where nios is 
                                                         // about to write.

    rp += BBCPCI_SIZE_UINT;
    if (rp >= BBCPCI_ADD_READ_BUF_END)
      rp = BBCPCI_ADD_READ_BUF;

    if (rp == wp)  // No new data.
      return 0; 
    else {
      b = readl(bbcpci_membase + rp);
      copy_to_user(buf, (void *)&b, BBCPCI_SIZE_UINT);
      WRITEL_S(rp, bbcpci_membase + BBCPCI_ADD_READ_BUF_RP); // Update pointer.
      return BBCPCI_SIZE_UINT;
    }
  } 
  else if (minor == bi0_minor) {
    b = 42;
    copy_to_user(buf, (void *)&b, BBCPCI_SIZE_UINT);  // FIXME: what should 
                                                      // read bi0 do?
    return BBCPCI_SIZE_UINT;
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
  int i, i0, nw, rem;
  unsigned short bi0datum;
  static int lastrem = 0;
  static unsigned short lastbi0datum = 0;
  minor = *((int *)(filp->private_data));

  if (minor == bbc_minor) {
    if (count < 8) 
      return 0;
    if (count > 8)
      count = 8;

    copy_from_user((void *)(&add), buf, BBCPCI_SIZE_UINT);
    copy_from_user((void *)(&datum), buf + BBCPCI_SIZE_UINT, BBCPCI_SIZE_UINT);

    tx_buffer.add[tx_buffer.i_in] = add;
    tx_buffer.data[tx_buffer.i_in] = datum;
  
    if (tx_buffer.i_in + 1 >= TX_BUFFER_SIZE)
      tx_buffer.i_in = 0;
    else
      tx_buffer.i_in++;
    tx_buffer.n++;
    
    return count;
  } 
  else if (minor == bi0_minor) {
    if (count < sizeof(unsigned short))
      return 0;
    
    nw = count / sizeof(unsigned short);
    rem = nw % 2;

    /* The remainder variables (rem, lastrem) are to deal with the possibility
     * that an odd frame size is given.  In this case, we must glue together
     * consecutive frames properly. */
    if (lastrem) {
      copy_from_user((void *)(&bi0datum), buf, sizeof(unsigned short));
      buf += sizeof(unsigned short);
      bi0_buffer.data[bi0_buffer.i_in] = ((lastbi0datum << 16) & 0xffff0000);
      bi0_buffer.data[bi0_buffer.i_in] |= (bi0datum & 0x0000ffff);

      if (bi0_buffer.i_in + 1 >= BI0_BUFFER_SIZE)
        bi0_buffer.i_in = 0;
      else 
        bi0_buffer.i_in++;
      bi0_buffer.n++;
      
      if (rem)
        rem = 0;
      else
        rem = 1;
      
      i0 = 1;
    }
    else
      i0 = 0;
   
    for (i = i0; i < nw - rem; i++) {
      if (!((i - i0) % 2)) {
        // MSB
        copy_from_user((void *)(&bi0datum), buf, sizeof(unsigned short));
        buf += sizeof(unsigned short);
        bi0_buffer.data[bi0_buffer.i_in] = ((bi0datum << 16) & 0xffff0000);
      }
      else {
        // LSB
        copy_from_user((void *)(&bi0datum), buf, sizeof(unsigned short));
        buf += sizeof(unsigned short);
        bi0_buffer.data[bi0_buffer.i_in] |= (bi0datum & 0x0000ffff);
      
        if (bi0_buffer.i_in + 1 >= BI0_BUFFER_SIZE)
          bi0_buffer.i_in = 0;
        else 
          bi0_buffer.i_in++;
        bi0_buffer.n++;
      }
    }

    if (rem) {
      lastrem = 1;
      copy_from_user((void *)(&lastbi0datum), buf, sizeof(unsigned short));
    }
    else
      lastrem = 0;

    return nw * sizeof(unsigned short);
  }

  return 0;
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
    if (bbc_allocated) {
      printk("BBC device is busy\n");
      return -EBUSY;
    }
		  
    filp->private_data = (void *) &bbc_minor;
    cbcounter = 0;

    /* Initialize tx buffer. */
    tx_buffer.n = tx_buffer.i_in = tx_buffer.i_out = 0;
    tx_buffer.data = (unsigned int *) kmalloc(TX_BUFFER_SIZE *
        			 sizeof(unsigned int), GFP_KERNEL);
    tx_buffer.add = (unsigned int *) kmalloc(TX_BUFFER_SIZE *
                    sizeof(unsigned int), GFP_KERNEL);
    if (tx_buffer.add == NULL) {
      printk("Error allocating TX_BUFFER\n");
      return -EBUSY;
    }
  
    bbc_allocated = 1;
  } 
  else if (minor == bi0_minor) {
    if (bi0_allocated) {
      printk("BI0 device is busy\n");
      return -EBUSY;
    }
    filp->private_data = (void *) &bi0_minor;

    /* Initialize bi0 buffer. */
    bi0_buffer.n = bi0_buffer.i_in = bi0_buffer.i_out = 0;
    bi0_buffer.data = (unsigned int *)kmalloc(BI0_BUFFER_SIZE *
			                                  sizeof(unsigned int), GFP_KERNEL);
    if (bi0_buffer.data == NULL) {
      printk("Error allocating BI0_BUFFER\n");
      return -EBUSY;
    }
    bi0_allocated = 1;
    
  } 
  else {
    printk("BBC: Invalid minor number\n");
    return -EBUSY;
  }

  if (!device_was_opened) {
    device_was_opened = 1;
    
    /* Get memory region. */
    if (check_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE)) {
      printk("bbc_pci: memory already in use\n");
      return -EBUSY;
    }
    request_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE, "bbc_pci");
      
    /* Reset the card. */
    bbcpci_membase = ioremap_nocache(bbcpci_membase_raw, BBCPCI_MEMSIZE);
    bitfield = BBCPCI_COMREG_RESET;
    WRITEL_S(bitfield, bbcpci_membase + BBCPCI_ADD_COMREG);

    /* Start the call-back timer. */
    timer_on = 1;
    init_timer(&bbc_cb_timer);
    bbc_cb_timer.function = timer_callback;
    bbc_cb_timer.expires = jiffies + 1;
    add_timer(&bbc_cb_timer);
  }
      
  return 0;
} 

/*******************************************************************/
/*                                                                 */
/*   release_bbc: free memory, stop timer                          */
/*                                                                 */
/*******************************************************************/
static int release_bbc(struct inode *inode, struct file *filp) {
  int minor = MINOR(inode->i_rdev);

  if (minor == bbc_minor) {
    kfree(tx_buffer.data);
    kfree(tx_buffer.add);
    bbc_allocated = 0;
  } 
  else {
    kfree(bi0_buffer.data);
    bi0_allocated = 0;
  }

  if (!bbc_allocated && !bi0_allocated) {
    timer_on = 0;
    del_timer_sync(&bbc_cb_timer);
    iounmap(bbcpci_membase);
    release_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE);
    device_was_opened = 0;
  }

  return 0;
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
    case BBCPCI_IOC_RESET:    /* Reset the BBC board - clear fifo, registers. */
      bitfield = BBCPCI_COMREG_RESET;
      WRITEL_S(bitfield, bbcpci_membase + BBCPCI_ADD_COMREG);
      break;
    case BBCPCI_IOC_SYNC:     /* Clear read buffers and restart frame. */
      bitfield = BBCPCI_COMREG_SYNC;
      WRITEL_S(bitfield, bbcpci_membase + BBCPCI_ADD_COMREG);
      break;
    case BBCPCI_IOC_VERSION:  /* Get the current version. */
      ret = readl(bbcpci_membase + BBCPCI_ADD_VERSION);
      break;
    case BBCPCI_IOC_COUNTER:  /* Get the counter. */
      ret = readl(bbcpci_membase + BBCPCI_ADD_COUNTER);
      break;
    case BBCPCI_IOC_SECRET:   /* Ssshhh. */
      for (i = 0; i < 2; i++)
        p[i] = readl(bbcpci_membase + (8 + i) * BBCPCI_SIZE_UINT);
      ret = copy_to_user((unsigned char *)arg, (unsigned char *)p, 
                         2 * BBCPCI_SIZE_UINT);
      break;	
    case BBCPCI_IOC_JIFFIES:  /* A way to read out jiffies... */
      ret = jiffies;  /* doesn't seem to work.  Bo'h? */
      break;
    case BBCPCI_IOC_CBCOUNTER:  /* Callback Counter. */
      ret = cbcounter; 
      break;
    case BBCPCI_IOC_WRITEBUF:  /* How many words in the NIOS write buf? */
      ret = readl(bbcpci_membase + BBCPCI_ADD_WRITE_BUF_P) - 
	          BBCPCI_ADD_IR_WRITE_BUF; 
      break;
    case BBCPCI_IOC_COMREG:  /* Return the command register. */
      /* COMREG_RESET tells if it is resetting. */
      ret = readl(bbcpci_membase + BBCPCI_ADD_COMREG);
      break;
    case BBCPCI_IOC_READBUF_WP: /* Where nios is about to write. */
      ret = readl(bbcpci_membase + BBCPCI_ADD_READ_BUF_WP);
      break;
    case BBCPCI_IOC_READBUF_RP: /* Where the PC is about to read. */
      ret = readl(bbcpci_membase + BBCPCI_ADD_READ_BUF_RP);
      break;
    case BBCPCI_IOC_WRITEBUF_N: /* How much data still to be read. */
      ret = tx_buffer.n;
      break;
    default:
      break;
    }
  } 
  else if (minor == bi0_minor) {
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
  
  /* Register device. */
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
