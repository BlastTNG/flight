/***************************************
* mpc.h
*
* All you need to control the MPC board!
*
* Licensed under the GPL
* Copyright 2004 Matthew Truch
****************************************/

#include <linux/ioctl.h>

#define MPC_IOC_MAGIC 0xbc

/* ioctl's take an unsigned long as their 
   argument unless otherwise indicated */

#define MPC_AUTO_SCAN     _IO(MPC_IOC_MAGIC, 0)
/* enable/disable auto scan mode */

#define MPC_EOC_IRQ       _IO(MPC_IOC_MAGIC, 1)
/* enable IRQA assertion on completion of ADC conversion */
/* otherwise use EOC polling */

#define MPC_IRQB_ENABLE   _IO(MPC_IOC_MAGIC, 2)
/* enable/disable IRQB */

#define MPC_ADC_START     _IO(MPC_IOC_MAGIC, 3)
/* start ADC */

#define MPC_DAC_ENABLE    _IO(MPC_IOC_MAGIC, 4)
/* enable/disable DAC subsystem */

#define MPC_PACER_INTERNAL _IO(MPC_IOC_MAGIC, 5)
/* internal/external pacer clock */

#define MPC_RESET_AS       _IO(MPC_IOC_MAGIC, 6)
/* reset Auto-Scan state machines */

#define MPC_SRAM_LOAD      _IO(MPC_IOC_MAGIC, 7)
/* load data on a channel into SRAM */

#define MPC_NUM_CHANNELS   _IO(MPC_IOC_MAGIC, 8)
/* set number of auto-scan channels */

#define MPC_SCAN_RATE      _IO(MPC_IOC_MAGIC, 9)
/* set ADC scan rate to be 1MHz / arg */
/* (max rate is 200 kHz; arg is 16 bits) */

#define MPC_SET_CHANNEL    _IO(MPC_IOC_MAGIC, 10)
/* set channel according to channel_struct */
/* you must pass an (mpc_channel_struct *) */

#define MPC_WAIT_POLL_EOC  _IO(MPC_IOC_MAGIC, 11)
/* block until EOC has been asserted (poll mode only) */

#define MPC_GET_DATA       _IO(MPC_IOC_MAGIC, 12)
/* returns data from ADC when in basic mode */

#define MPC_DIO_DIR        _IO(MPC_IOC_MAGIC, 13)
/* set which channels are inputs (0) or outputs (1) */

#define MPC_SET_DIO        _IO(MPC_IOC_MAGIC, 14)
/* set values of digital outputs */

#define MPC_GET_DIO        _IO(MPC_IOC_MAGIC, 15)
/* get values of digital inputs */

#define MPC_SET_IRQB_MASK  _IO(MPC_IOC_MAGIC, 16)
/* set IRQB mask (see docs) */

#define MPC_BASIC_RUN      _IO(MPC_IOC_MAGIC, 17)
/* Starts the ADC conversion, waits (blocks) for conversion
   to finish, and returns the result */

#define MPC_SET_CHANNEL_RUN _IO(MPC_IOC_MAGIC, 18)
/* Sets the channel given the mpc_channel_struct,
   starts the ADC conversion, waits (blocks) for conversion
   to finish, and returns the results.
   Identical to MPC_SET_CHANNEL and MPC_BASIC_RUN called
   in sucession */

struct mpc_channel_struct {
  int active;
  int diff;
  int gain_double;
  int filter_low;
  int channel_num;
};

typedef struct mpc_channel_struct mpc_cs;
