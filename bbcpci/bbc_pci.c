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

#define BBCPCI_VENDOR 0x5045
#define BBCPCI_ID 0x4243

/* this memsize is the amount of pci space we want.  Wishfull thinking....
 * the card only lets us see 16k... */
#define BBCPCI_MEMSIZE 8388608

/********************************************************************/
/* Define structures and locally global variables                   */
/********************************************************************/

/* 2.6.x: Declare license to prevent tainting kernel */
MODULE_LICENSE("GPL");

/* Jiffies is a global kernel variable which is incremented at 1000 hz */
volatile unsigned long jiffies;

/* The address where the PCI card's memory landed */
void *bbcpci_membase = 0;
unsigned long bbcpci_membase_raw = 0;
unsigned long bbcpci_resourceflags = 0;
int readoffset = 0;


/*******************************************************************/
/*                                                                 */
/*  read_bbc: copies from the memory to the output buffer.         */
/*  Copies just one word, if there are words availible.            */
/*                                                                 */
/*******************************************************************/
static ssize_t read_bbc(struct file * filp, char * buf, size_t count,
                         loff_t *dummy) {
  unsigned int rp, wp, b;

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
}

/*******************************************************************/
/*                                                                 */
/*   write_bbc: directly write the 32 bit words to the pci card    */
/*                                                                 */
/*******************************************************************/
static ssize_t write_bbc(struct file * filp, const char * buf,
                         size_t count, loff_t *dummy) {
  unsigned int add, datum, position;
  int i, num;       

  readoffset = 0;
  
  if (count == 0)
    return 0;
  
  if (count > 2 * SIZE_UINT * IR_WRITE_BUF_SIZE)
    count = 2 * SIZE_UINT * IR_WRITE_BUF_SIZE;
  num = (int)(count / (2 * SIZE_UINT));
 
  /* Wait for int. ram write buffer to empty. */
  while ((position = readl(bbcpci_membase + ADD_WRITE_BUF_P)) >= 
                                            ADD_IR_WRITE_BUF);

  for (i = 0; i < num; i++) {
      position += 2 * SIZE_UINT;
      copy_from_user((void *)(&add), buf + i * (2 * SIZE_UINT), SIZE_UINT);
      copy_from_user((void *)(&datum), buf + i * (2 * SIZE_UINT) + SIZE_UINT, 
                                       SIZE_UINT);
      writel(add, bbcpci_membase + position);
      writel(datum, bbcpci_membase + position + SIZE_UINT);
  } 
  writel((unsigned int)position, bbcpci_membase + ADD_WRITE_BUF_P);
  
  return num * 2 * SIZE_UINT;
}

/*******************************************************************/
/*                                                                 */
/*  open_bbc:                                                      */
/*                                                                 */
/*******************************************************************/
static int open_bbc(struct inode *inode, struct file * filp){
  if (check_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE)) {
    printk("bbc_pci: memory already in use\n");
    return -EBUSY;
  }
  request_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE, "bbc_pci");
  
  bbcpci_membase = ioremap_nocache(bbcpci_membase_raw, BBCPCI_MEMSIZE);

  return 0;
} 

/*******************************************************************/
/*                                                                 */
/*   release_bbc: free memory, stop timer                          */
/*                                                                 */
/*******************************************************************/
static int release_bbc(struct inode *inode, struct file *filp) {

  iounmap(bbcpci_membase);
  release_mem_region(bbcpci_membase_raw, BBCPCI_MEMSIZE);
  
  return(0);
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
  
  if (_IOC_DIR(cmd) & _IOC_READ) {
    if (!access_ok(VERIFY_WRITE, (void *)arg, size))
      return -EFAULT;
  }
  else if (_IOC_DIR(cmd) & _IOC_WRITE) {
    if (!access_ok(VERIFY_READ, (void *)arg, size))
      return -EFAULT;
  }
  
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
    default:
      break;
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

  printk("===> %lx, %lx\n", bbcpci_resourceflags, bbcpci_membase_raw);
  
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
