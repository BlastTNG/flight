/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2002-2006 University of Toronto
 
 This file is part of mcp.
 
 mcp is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 at your option) any later version.
 
 mcp is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with mcp; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 
 created by Ian Lowe 11-29-19
 **************************************************************************/


/*************************************************************************
 
 hawkeyeir.c - code to control the hawkeye IR source.
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "blast.h"

#define IR_POWER_ADDR = 2008;
extern labjack_state_t state[NUM_LABJACKS];

typedef struct {
    int go;
    uint16_t length;
    int on;
    int just_swapped;
    int no_pulse;
    int stop;
} ir_control_t

ir_control_t hawkeye={0};

static void update_ir_values(){
    hawkeye.go = ComamandData.IRsource.go;
    hawkeye.length = ComamandData.IRsource.length;
    hawkeye.on = ComamandData.IRsource.on;
    hawkeye.no_pulse = CommandData.IRsource.no_pulse;
}

static void static_ir_load(){
    update_ir_values();
    if (state[10].connected) {
        if (hawkeye.go == 0) {
            labjack_queue_command(LABJACK_IR, IR_POWER_ADDR, 0);
        }
        if (hawkeye.no_pulse == 1 && hawkeye.go == 1) {
            labjack_queue_command(LABJACK_IR, IR_POWER_ADDR, 1);
        }
    }
}

static void run_ir_source(){
    static int counter = 0;
    update_ir_values();
    hawkeye.just_swapped = 0;
    if (state[10].connected) {
        if (hawkeye.go == 0) {
            counter = 0;
            labjack_queue_command(LABJACK_IR, IR_POWER_ADDR, 0);
        }
        if (hawkeye.go == 1) {
            if (hawkeye.on == 1) {
                labjack_queue_command(LABJACK_IR, IR_POWER_ADDR, 1);
                counter++;
                if (counter == length) {
                    hawkeye.just_swapped = 1;
                    counter = 0;
                    hawkeye.on = 0;
                }
            }
            if (hawkeye.on == 0 && hawkeye.just_swapped == 0) {
                labjack_queue_command(LABJACK_IR, IR_POWER_ADDR, 0);
                counter++;
                if (counter == length) {
                    hawkeye.just_swapped = 1;
                    counter = 0;
                    hawkeye.on = 0;
                }
            }
        }
    }
}

static void publish_value(){
    static int first_time = 1;
    static channel_t* hawkeye_Addr;
    if (first_time == 1) {
        hawkeye_Addr = channels_find_by_name("hawkeye");
    }
    if (state[10].connected) {
        if (hawkeye.go == 1) {
            SET_SCALED_VALUE(hawkeye_Addr, 1.0);
        }
        if (hawkeye.go == 0) {
            SET_SCALED_VALUE(hawkeye_Addr, 0.0);
        }
    }
}


void hawkeye_control(int run){
    if (run == 1) {
        static_ir_source();
        run_ir_source();
        publish_value();
    }
}

