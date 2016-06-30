//
//  actual_cycles.c
//  hct
//
//  Created by Ian Lowe on 6/14/16.
//
//

#include <stdlib.h>  // for strtol
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "actual_cycles.h"
#include "channels_tng.h"
#include "tx.h"
#include "mcp.h"

#define T_ARRAY_CRITICAL 0.3
#define T_CHARCOAL_CRITICAL 40

enum mode { 
  NORMAL, // heat switch on, charcoal heater off
  FRIDGE_CYCLE // heat switch off, charcoal heater on
};

struct _HeaterData {
  int seconds_counter;  
  int array_looper;

  enum mode mode; 
  
  double t250, t350, t500, tcharcoal;
  double* t250_history;
  double* t350_history;
  double* t500_history;

  channel_t* ch_t250, ch_t350, ch_t500, ch_charcoal_heater, ch_heatswitch;
};

// TODO: CryoStatus / Info
// TODO: make the struct global so that heater data may be shared. Be careful about avoiding inconsistent state caused by multithreading

// these are for other heater control things, not used in my autocycle, will be used in heatctrl in future
/*static unsigned short p1 = 128;*/
/*static unsigned short p2 = 256;*/
/*static unsigned short p3 = 512;*/
/*static unsigned short p4 = 1024;*/
/*static unsigned short p5 = 2048;*/
/*static unsigned short p6 = 4096;*/
/*static unsigned short p7 = 8192;*/
/*static unsigned short p8 = 16384;*/
/*static unsigned short p9 = 32768;*/
 /*uncomment when add to MCP*/
/* static channel_t* t250_addr, t350_addr, t500_addr, tcharc_addr; */

/*
   Allocates a HeaterData on the heap, initializes it, and returns a reference 
*/
// TODO: use balloc
HeaterData* initialize_heaters() {
  int i;

  HeaterData* hc = malloc(sizeof(HeaterData));

  // Initialize the heater
  hc->seconds_counter = 0;
  hc->array_looper = 0;
  hc->mode = NORMAL; //* start in normal mode
  hc->t250_history = balloc(sizeof(double) * 200);
  hc->t350_history = balloc(sizeof(double) * 200);
  hc->t500_history = balloc(sizeof(double) * 200);

  // Init all array values to 0
  for (i = 0; i < 200; ++i) {
    hc->t250_history[i] = 0;
    hc->t350_history[i] = 0;
    hc->t500_history[i] = 0;
  }

  // Init the channels
  hc->ch_t250 = channels_find_by_name("tr_250_fpa");
  hc->ch_t350 = channels_find_by_name("tr_350_fpa");
  hc->ch_t500 = channels_find_by_name("tr_500_fpa");
  hc->ch_charcoal_heater = channels_find_by_name("td_charcoal");
  hc->ch_heatswitch = channels_find_by_name("td_charcoal_hs");

  return hc;
}

// need a control for heaters here
static void heater_control(int heater_value) {
    // TODO
}

static void heatswitch_control(int heatswitch_value) {
    // TODO
}

static double average(double my_array[], int size) {
  int i;
  double sum = 0;
  for (i = 0; i < size; ++i) {
    sum += my_array[i];
  }
  return sum / size;
}

static void update_from_channels(HeaterData* hc) {
   /*TODO: double-check how GET_VALUE works*/
  // TODO: check for dirfile read errors
  GET_VALUE(hc->tcharcoal, hc->ch_charcoal_heater);
  GET_VALUE(hc->t250, hc->ch_t250);
  GET_VALUE(hc->t350, hc->ch_t350);
  GET_VALUE(hc->t500, hc->ch_t500);
}

static void record_history(HeaterData* hc) {
  hc->t250_history[hc->array_looper] = hc->t250;
  hc->t350_history[hc->array_looper] = hc->t350;
  hc->t500_history[hc->array_looper] = hc->t500;
  if (hc->array_looper == 200) hc->array_looper = 0;
  hc->array_looper++;
}

// Returns 1 if any temp sensor is too hot, 0 if all are cool
static int is_array_overheating(HeaterData* hc) {
  return 
    average(hc->t250_history, 200) > T_ARRAY_CRITICAL ||
    average(hc->t350_history, 200) > T_ARRAY_CRITICAL ||
    average(hc->t500_history, 200) > T_ARRAY_CRITICAL;
}

static void read_triggers(HeaterData* hc) {
  switch (hc->mode) {
    case FRIDGE_CYCLE:
      hc->seconds_counter++;
      if (hc->seconds_counter >= 2400) { // counts up, check if we heated for 40 mins
          printf("Charcoal heating has timed out, turning off heaters and reengaging heat switch\n");
          hc->mode = NORMAL;
      } else {
          //GET_VALUE(tcharc_addr, tcharcoal);
          if (hc->tcharcoal > T_CHARCOAL_CRITICAL) {
            hc->mode = NORMAL;
          }
      }
      printf("%d seconds have passed, %d remaining of 2400\n", hc->seconds_counter, 2400 - hc->seconds_counter);
      break;
    case NORMAL:
      if (is_array_overheating(hc)) {
          printf("The arrays have warmed up");
          hc->mode = FRIDGE_CYCLE;
          hc->seconds_counter = 0;
      } else {
          printf("The arrays are within normal operating temperature");
      }
      break;
  }
}

// TODO: integrate
static void turn_off (HeaterData* hc) {
  bfree(hc->t250_history);
  bfree(hc->t350_history);
  bfree(hc->t500_history);
  bfree(hc);
}

static void write_to_channels (HeaterData* hc) {
  // Read the heat switch and heater bits
  int heater, heat_switch;
  switch (hc->mode) {
    case NORMAL:
      heater = 0;
      heat_switch = 1;
      break;
    case FRIDGE_CYCLE:
      heater = 1;
      heat_switch = 0;
      break;
  }
  SET_SCALED_VALUE(hc->ch_charcoal_heater, heater);
  SET_SCALED_VALUE(hc->ch_heatswitch, heat_switch);
}

/*
   The passed heat controller -must- be generated first
*/
void autocycle(HeaterData* hc) {
  update_from_channels(hc);
  record_history(hc);
  read_triggers(hc);
  write_to_channels(hc);

  // TODO: let the command data influence the decisions
  // see CommandData.cryo_cmds_t for the different commands
  // NB: how is CommandData made threadsafe? It is read on many different threads (though a ph_mutex limits access on the thread that receives commands)
  // We are only reading CommandData (not setting it). At what point should the command data override the regular cycle steps?
}

