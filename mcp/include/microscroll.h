// Copyright 2019 Ian Lowe
//  microscroll.h
//
//
//  Created by Ian Lowe on 11/29/19.
//
//
#ifndef INCLUDE_MICROSCROLL_H
#define INCLUDE_MICROSCROLL_H

#define N_AALBORG_VALVES    3
#define AALBORG_HIGH_LEVEL  8.5
#define AALBORG_LOW_LEVEL   3.5
#define AALBORG_CLOSE_CMD   0.0 // 0 V output on TDAC
#define AALBORG_OPEN_CMD   10.0 // 10 V output on TDAC

#define AALBORG_OPENED     0x01
#define AALBORG_CLOSED     0x02
#define AALBORG_OPENING    0x04
#define AALBORG_CLOSING    0x08
#define AALBORG_NOT_CLOSED 0x10
#define AALBORG_UNK        0x20

// we wait 12 seconds after commanding aalborg to open to declare it is open
// this is because the signal we read is closed/not closed, but we know how it takes about 11 to open
#define AALBORG_WAIT_OPENING 12 // 12s at 1 Hz

#define thermistor_1    6
#define thermistor_2    7
#define thermistor_3    8
#define thermistor_4    9
#define thermistor_5    10
#define thermistor_6    11
#define thermistor_7    12
#define thermistor_8    13
#define relay_12V_on    2000
#define relay_12V_off   2001
#define supply_24Va     2006
#define supply_24Vb     2007
#define SPEED_1_REG     1000
#define SPEED_2_REG     1002
#define SPEED_3_REG     30010
#define VALVE_1_STATUS  0
#define VALVE_2_STATUS  1
#define VALVE_3_STATUS  2
#define VALVE1_DIR      30004
#define VALVE2_DIR      30006
#define VALVE3_DIR      30008

void ControlAalborg(int index);
void execute_microscroll_functions();
void WriteAalborgs();
void TestLjWrites();
#endif
