#ifndef _BBC_SYNC
#define _BBC_SYNC

#define BBC_SYNC_MAJOR 0      //means dynamic

// When including in user-space programs, need time.h for timeval
#ifndef __KERNEL__
#  include <sys/time.h>
#endif

// WARNING: do not change this unless you also change the report in function
// data_struct_show()
struct timing_data {
  unsigned int serialnumber;
  struct timeval tv;              //time at which interrupt handled
  unsigned short workqueue_delay; //delay until worqueue handled (usec)
  unsigned short status;
  //USER: add fields here as necessary
};

// The timing buffer is long enough to hold 0x800 = 2048 timing_data structs 
#define BBC_SYNC_BUFSIZE (sizeof(struct timing_data)* 0x800)

//bit fields for the status entry
#define BBC_SYNC_MULTIPLE_INTERRUPTS 0x01  //bot. half handling several irqs
                                             //should never happen

//flag to include code that pads data stream where it looks like interrupts have
//been missed (for whatever reason). Probably not needed.
#define BBC_SYNC_PAD_MISSED 0

#if BBC_SYNC_PAD_MISSED == 1
//more status fields
#define BBC_SYNC_DATA_REPEATED       0x02  //entry is fake, uses some old data
#define BBC_SYNC_EXCESSIVE_LOST      0x04  //too many entries faked
                                             //see MAX_FAKED_DATA
#endif


#endif // _BBC_SYNC


