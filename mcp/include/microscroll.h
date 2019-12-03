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
#define AALBORG_CLOSE_LEVEL    8.5
#define AALBORG_OPEN_LEVEL     3.5

#define AALBORG_OPENED   0x0001
#define AALBORG_CLOSED   0x0002
#define AALBORG_OPENING  0x0004
#define AALBORG_CLOSING  0x0008
void ControlAalborg(int index);
#endif
