//
//  actual_cycles.h
//  hct
//
//  Created by Ian Lowe on 6/14/16.
//
//

#ifndef actual_cycles_h
#define actual_cycles_h

#include <stdio.h>
#include <stdlib.h>  // for strtol
#include <math.h>
#include <stdio.h>
#include <string.h>

struct _HeaterData;
typedef struct _HeaterData HeaterData;

HeaterData* initialize_heaters();
void autocycle(HeaterData* hc);

#endif /* actual_cycles_h */
