#ifndef HWPR_H
#define HWPR_H

#include "ezstep.h"

//#define HWPR_STEPS_PER_MOTENC (64)
#define HWPR_REV_PER_MOTREV (24./100.)

#define HWPR_NAME "HWPR Motor"
#define HWPR_ADDR EZ_WHO_S13
#define HWPR_PREAMBLE "aE64000"

#define HWPR_CHECK_NONE 0 
#define HWPR_CHECK_BEFORE 1
#define HWPR_CHECK_AFTER 2
#define HWPR_CHECK_BOTH 3

#define HWPR_POT_MIN 0.1
#define HWPR_POT_MAX 0.9

#define HWPR_DEFAULT_STEP 7292 // used if pot is dead

void DoHWPR(struct ezbus* bus);

#endif
