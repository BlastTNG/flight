// Copyright 2019 Ian Lowe
//  microscroll.h
//
//
//  Created by Ian Lowe on 11/29/19.
//
//
#ifndef INCLUDE_MICROSCROLL_H
#define INCLUDE_MICROSCROLL_H
#define thermistor_1  6
#define thermistor_2  7
#define thermistor_3  8
#define thermistor_4  9
#define thermistor_5  10
#define thermistor_6  11
#define thermistor_7  12
#define thermistor_8  13
#define pump_1_power_val

#define N_AALBORG_VALVES 3
#define AALBORG_HIGH_LEVEL  8.5
#define AALBORG_LOW_LEVEL   3.5

#define AALBORG_OPENED     0x01
#define AALBORG_CLOSED     0x02
#define AALBORG_OPENING    0x04
#define AALBORG_CLOSING    0x08
#define AALBORG_NOT_CLOSED 0x10
#define AALBORG_INTERMED   0x20

// we wait 7 seconds after commanding aalborg to open to declare it is open
// this is because the signal we read is closed/not closed, but we know how it takes about 6 to open
#define AALBORG_WAIT_OPENING 35 // 7s at 5 Hz
void ControlAalborg(int index);
#endif
