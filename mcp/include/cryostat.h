// Copyright 2016 Ian Lowe
//  cryostat.h
//
//
//  Created by Ian Lowe on 5/13/16.
//
//
#ifndef INCLUDE_CRYOSTAT_H
#define INCLUDE_CRYOSTAT_H
void cal_command(int length);
void test_labjacks(int m_which);
void heater_control(void);
void heater_all_off(void);
void cal_control(void);
void level_control(void);
void store_100hz_cryo(void);
void autocycle(void);
void read_thermometers();
void test_read(void);
void test_frequencies(void);
void tie_up(void);
void heater_read(void);
void test_cycle(void);
void autocycle_ian(void);

#endif /* cryostat_h */
