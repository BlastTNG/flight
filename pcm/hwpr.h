#ifndef HWPR_H
#define HWPR_H

/* HWP/phytron bus setup paramters */
#define NHWP 6
#define HWP_BUS "/dev/ttySI10"
#define HWP_CHATTER	PH_CHAT_ACT

/* stepper parameters. definition in hwpr.c */
extern const char *hwp_name[NHWP];
extern const char *hwp_id[NHWP];
extern const int hwp_nteeth[NHWP];

void StartHWP();         //create a thread for HWP control/communication
void countHWPEncoder();  //count ticks of the shaft encoder. Fast frame call
void StoreHWPBus();      //store HWP parameters to frame. Slow frame call

#endif
