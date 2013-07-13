#include <string.h>
#include <stdlib.h>
#include "compressstruct.h"

extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];

//*********************************************************
// check if the name referes to a bolometer
//*********************************************************
int isBoloField(char *field) {
  if (field[0]!='n') return 0;
  if (strlen(field)!=6) return 0;
  if (field[3]!='c') return 0;
  if (field[1]<'1') return 0;
  if (field[1]>'3') return 0;

  return 1;
}

//*********************************************************
// return a pointer to a channel struct describing the field.
// if its a bolometer, make a struct with default values.
// WARNING: rw, bus, node, and addr are not set for bolo fields
//*********************************************************
struct ChannelStruct *GetChannelStruct(char *name) {
  int i_ch;

  struct ChannelStruct * cs;

  // search the slow channels
  for (i_ch = 0; SlowChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, SlowChannels[i_ch].field)==0) {
      return (SlowChannels+i_ch);
    }
  }

  // search the slow wide channels
  for (i_ch = 0; WideSlowChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, WideSlowChannels[i_ch].field)==0) {
      return (WideSlowChannels+i_ch);
    }
  }

  // search the fast channels
  for (i_ch = 0; FastChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, FastChannels[i_ch].field)==0) {
      return (FastChannels+i_ch);
    }
  }

  // search the wide fast channels
  for (i_ch = 0; WideFastChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, WideFastChannels[i_ch].field)==0) {
      return (WideFastChannels+i_ch);
    }
  }

  if (isBoloField(name)) {
    cs = (struct ChannelStruct *)malloc(sizeof(struct ChannelStruct));
    strcpy(cs->field, name);
    cs->type = 'U';
    cs->m_c2e = LOCKIN_C2V;
    cs->b_e2e = LOCKIN_OFFSET;
    cs->quantity[0] = '\0';
    cs->units[0] = '\0';
    return(cs);
  }

  return (NULL);
}

