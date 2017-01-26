// Copyright 2016 Ian Lowe
//  relay_control.h
//
//
//  Created by Ian Lowe on 1/18/17.
//
//
#ifndef INCLUDE_RELAY_CONTROL_H
#define INCLUDE_RELAY_CONTROL_H
void rec_switch(int which);
void rec_control(void);
// Outer Frame Relays
// LABJACK 1 AND 3
#define RELAY_1_ON 2000
#define RELAY_1_OFF 2001
#define RELAY_2_ON 2002
#define RELAY_2_OFF 2003
#define RELAY_3_ON 2004
#define RELAY_3_OFF 2005
#define RELAY_4_ON 2006
#define RELAY_4_OFF 2007
#define RELAY_5_ON 2008
#define RELAY_5_OFF 2009
#define RELAY_6_ON 2010
#define RELAY_6_OFF 2011
#define RELAY_7_ON 2012
#define RELAY_7_OFF 2013
#define RELAY_8_ON 2014 // labjack 2
#define RELAY_8_OFF 2015 // labjack 2
// LABJACK 2
#define RELAY_9_ON 2000
#define RELAY_9_OFF 2001
#define RELAY_10_ON 2002
#define RELAY_10_OFF 2003
#define RELAY_11_ON 2004
#define RELAY_11_OFF 2005
#define RELAY_12_ON 2006
#define RELAY_12_OFF 2007
#define RELAY_13_ON 2008
#define RELAY_13_OFF 2009
#define RELAY_14_ON 2010
#define RELAY_14_OFF 2011
#define RELAY_15_ON 2012
#define RELAY_15_OFF 2013
#define RELAY_16_ON 2014 // labjack 1
#define RELAY_16_OFF 2015 // labjack 1
// LABJACK 3 EXTENDED
#define IF_RELAY_9_ON 2016
#define IF_RELAY_9_OFF 2017
#define IF_RELAY_10_ON 2018
#define IF_RELAY_10_OFF 2019
void of_1_4_switch(int which);
void of_5_8_switch(int which);
void of_9_12_switch(int which);
void of_13_16_switch(int which);
void if_1_5_switch(int which);
void if_6_10_switch(int which);
void of_1_4_control(void);
void of_5_8_control(void);
void of_9_12_control(void);
void of_13_16_control(void);
void if_1_5_control(void);
void if_6_10_control(void);


#endif /* relay_control_h */
