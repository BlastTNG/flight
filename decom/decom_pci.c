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

#include "decom_pci.h"

/*******************************************************************/
/* Definitions                                                     */
/*******************************************************************/

/* This major number is reserved for local or experimental use... */
/* See Linux Device Drivers 2nd ed, P 58 footnote */
/* Assuming no one else uses this number, this is more convenient */
/* than dynamic major/minor.  Note though, that the BBCPCI (250) and the */
/* Decom_PCI (251?) and any other board we make must have different majors. */
#define DECOM_MAJOR 251

#define DECOMPCI_VENDOR 0x5045
#define DECOMPCI_ID 0x4244

/* Card is programmed to show 32 KB on the PCI. */
#define DECOMPCI_MEMSIZE 0x8000

/********************************************************************/
/* Define structures and locally global variables                   */
/********************************************************************/

/* 2.6.x: Declare license to prevent tainting kernel */
MODULE_LICENSE("GPL");

/* Jiffies is a global kernel variable which is incremented at 1000 hz */
volatile unsigned long jiffies;

/* The address where the PCI card's memory landed */
void *decom_membase = 0;
unsigned long decom_membase_raw = 0;
unsigned long decom_resourceflags = 0;

/*******************************************************************/
/*                                                                 */
/*  read_decom: copies from the memory to the output buffer.         */
/*  Copies just one word, if there are words availible.            */
/*                                                                 */
/*******************************************************************/
static ssize_t read_decom(struct file * filp, char * buf, size_t count,
                         loff_t *dummy) {
  unsigned int rp, wp, b;

  if (count < DECOM_SIZE_UINT)
    return 0;

  rp = readl(decom_membase + DECOM_ADD_READ_BUF_RP); // Where pci read last.
  wp = readl(decom_membase + DECOM_ADD_READ_BUF_WP); // Where NIOS is about to write.

  rp += DECOM_SIZE_UINT;
  if (rp >= DECOM_ADD_READ_BUF_END)
    rp = DECOM_ADD_READ_BUF;

  if (rp == wp)  // No new data.
    return 0;
  else {
    b = readl(decom_membase + rp);
    copy_to_user(buf, (void *)&b, DECOM_SIZE_UINT);
    writel(rp, decom_membase + DECOM_ADD_READ_BUF_RP); // Update read pointer.
    return DECOM_SIZE_UINT;
  }
}

/*******************************************************************/
/*                                                                 */
/*   write_decom: directly write the 32 bit words to the pci card    */
/*                                                                 */
/*******************************************************************/
static ssize_t write_decom(struct file * filp, const char * buf,
                         size_t count, loff_t *dummy) {
  unsigned int datum;
        
  if (count < DECOM_SIZE_UINT)
    return 0;
  
  /* There is really no need to write to the DECOM card.  The only thing PCI
   * controls is the frame length, which is properly done through an ioctl call.
   * So only allow users to write to frame length word on the board here. */
  
  copy_from_user((void *)(&datum), buf, DECOM_SIZE_UINT);
  writel(datum, decom_membase + DECOM_ADD_FRAME_LEN);

  printk("--> %d\n", readl(decom_membase + DECOM_ADD_FRAME_LEN));
  
  return DECOM_SIZE_UINT;
}

/*******************************************************************/
/*                                                                 */
/*  open_decom:                                                    */
/*                                                                 */
/*******************************************************************/
static int open_decom(struct inode *inode, struct file * filp){
  if (check_mem_region(decom_membase_raw, DECOMPCI_MEMSIZE)) {
    printk("Decom_PCI: memory already in use.\n");
    return -EBUSY;
  }
  request_mem_region(decom_membase_raw, DECOMPCI_MEMSIZE, "Decom_PCI");
  
  decom_membase = ioremap_nocache(decom_membase_raw, DECOMPCI_MEMSIZE);

  return 0;
} 

/*******************************************************************/
/*                                                                 */
/*   release_decom: free memory, stop timer                        */
/*                                                                 */
/*******************************************************************/
static int release_decom(struct inode *inode, struct file *filp) {

  iounmap(decom_membase);
  release_mem_region(decom_membase_raw, DECOMPCI_MEMSIZE);
  
  return(0);
}

/*******************************************************************/
/*                                                                 */
/*   ioctl: control decom card: see decom_pci.h for definitions    */
/*                                                                 */
/*******************************************************************/
static int ioctl_decom (struct inode *inode, struct file * filp,
                        unsigned int cmd, unsigned long arg) {
  unsigned int ret = 0;
  unsigned int bitfield;
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
      bitfield = DECOM_COMREG_RESET;
      writel(bitfield, decom_membase + DECOM_ADD_COMREG);
      break;
    case DECOM_IOC_VERSION:
      ret = readl(decom_membase + DECOM_ADD_VERSION);
      break;
    case DECOM_IOC_COUNTER:
      ret = readl(decom_membase + DECOM_ADD_COUNTER);
      break;
    case DECOM_IOC_FRAMELEN:
      writel(arg, decom_membase + DECOM_ADD_FRAME_LEN);
      ret = arg;
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
static struct file_operations decom_fops = {
  owner:   THIS_MODULE,
  llseek:  no_llseek,
  read:  read_decom,
  write:  write_decom,
  ioctl:  ioctl_decom,
  open:  open_decom,
  release:  release_decom,
};

/*******************************************************************/
/*                                                                 */
/*   init_decom: called upon insmod decom.o                        */
/*                                                                 */
/*******************************************************************/
int init_decom_pci(void) {
  struct pci_dev *dev = NULL;
  
  /* Register device */
  if (register_chrdev(DECOM_MAJOR, "decom_pci", &decom_fops) != 0) {
    printk("Unable to get major for decom_pci device\n");
    return -EIO;
  }
  
  dev = pci_find_device(DECOMPCI_VENDOR, DECOMPCI_ID, dev);
  if (!dev) {
    printk("Unable to find DECOM_PCI card.\n");
    return -EIO;
  }
  pci_enable_device(dev);
  decom_membase_raw = pci_resource_start(dev, 0);
  decom_resourceflags = pci_resource_flags(dev, 0);

  printk("===> %lx, %lx\n", decom_resourceflags, decom_membase_raw);
  
  return 0;
}

/*******************************************************************/
/*                                                                 */
/*   cleanup_decom: called upon rmmod decom                        */
/*                                                                 */
/*******************************************************************/
void cleanup_decom_pci(void) { 
  unregister_chrdev(DECOM_MAJOR, "decom_pci");
}

/* Declare module init and exit functions */
module_init(init_decom_pci);
module_exit(cleanup_decom_pci);
