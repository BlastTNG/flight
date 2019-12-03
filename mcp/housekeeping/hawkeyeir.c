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
#include "hawkeyeir.h"

extern labjack_state_t state[NUM_LABJACKS];

typedef struct {
    int go;
    uint16_t length;
    int on;
    int just_swapped;
    int no_pulse;
    int stop;
    int_just_received;
} ir_control_t;

ir_control_t hawkeye;

static void update_ir_values() {
    hawkeye.go = CommandData.IRsource.go;
    hawkeye.length = CommandData.IRsource.length;
    hawkeye.just_received = CommandData.IRsource.just_received;
    hawkeye.no_pulse = CommandData.IRsource.no_pulse;
}

static void clear_eio() {
    static int first_time = 1;
    if (first_time && state[10].connected) {
        blast_info("clearing the EIOs");
        labjack_queue_command(10, 2008, 0.0);
        labjack_queue_command(10, 2009, 0.0);
        labjack_queue_command(10, 2010, 0.0);
        labjack_queue_command(10, 2011, 0.0);
        labjack_queue_command(10, 2012, 0.0);
        labjack_queue_command(10, 2013, 0.0);
        labjack_queue_command(10, 2014, 0.0);
        labjack_queue_command(10, 2015, 0.0);
        first_time = 0;
    }
}

static void static_ir_load() {
    update_ir_values();
    if (state[10].connected) {
        if (hawkeye.go == 0) {
            labjack_queue_command(10, 2008, 0.0);
        }
        if (hawkeye.no_pulse == 1 && hawkeye.go == 1) {
            labjack_queue_command(10, 2008, 1.0);
        }
    }
}

static void run_ir_source() {
    static int counter = 0;
    update_ir_values();
    hawkeye.just_swapped = 0;
    if (state[10].connected) {
        if (hawkeye.go == 0) {
            counter = 0;
            labjack_queue_command(10, 2008, 0.0);
        }
        if (hawkeye.go == 1 && hawkeye.no_pulse == 0) {
            if (hawkeye.just_received) {
                CommandData.IRsource.just_received = 0;
                hawkeye.on = 1;
            }
            if (hawkeye.on == 1) {
                labjack_queue_command(10, 2008, 1.0);
                counter++;
                if (counter == hawkeye.length) {
                    hawkeye.just_swapped = 1;
                    counter = 0;
                    hawkeye.on = 0;
                }
            }
            if (hawkeye.on == 0 && hawkeye.just_swapped == 0.0) {
                labjack_queue_command(10, 2008, 0);
                counter++;
                if (counter == hawkeye.length) {
                    hawkeye.just_swapped = 1;
                    counter = 0;
                    hawkeye.on = 1;
                }
            }
        }
    }
}

static void publish_value() {
    static int first_time = 1;
    static channel_t* hawkeye_Addr;
    if (first_time == 1) {
        hawkeye_Addr = channels_find_by_name("hawkeye");
        first_time = 0;
    }
    if (state[10].connected) {
        if (hawkeye.go == 1) {
            SET_SCALED_VALUE(hawkeye_Addr, 1.0);
        }
        if (hawkeye.go == 0) {
            SET_SCALED_VALUE(hawkeye_Addr, 0.0);
        }
    }
    if (state[10].connected == 0) {
        blast_info("lj11 not connected");
    }
}


void hawkeye_spewer() {
    update_ir_values();
    blast_info("go is %d", hawkeye.go);
    blast_info("length is %d", hawkeye.length);
    blast_info("on is %d", hawkeye.on);
    blast_info("no_pulse is %d", hawkeye.no_pulse);
}

void hawkeye_control(int run) {
    if (run == 1) {
        clear_eio();
        static_ir_load();
        run_ir_source();
        publish_value();
    }
}

