// Copyright 2016 Ian Lowe
//  relay_control.h
//
//
//  Created by Ian Lowe on 1/18/17.
//
//
#ifndef INCLUDE_RELAY_CONTROL_H
#define INCLUDE_RELAY_CONTROL_H
void rec_control(void);
void of_control(void);
void if_control(void);
void relays(int setting);
#define POWER 0
#define CRYO 1
#define ALL_RELAYS 3
// DIO addresses LJ CRYO 2
#define POWER_BOX_ON 2001
#define POWER_BOX_OFF 2000
#define AMP_SUPPLY_ON 2003
#define AMP_SUPPLY_OFF 2002
#define THERM_READOUT_ON 2005
#define THERM_READOUT_OFF 2004
#define HEATER_SUPPLY_ON 2007
#define HEATER_SUPPLY_OFF 2006
// Outer Frame Relays
// LABJACK 1 AND 3
#define RELAY_1_ON 2008
#define RELAY_1_OFF 2009
#define RELAY_2_ON 2010
#define RELAY_2_OFF 2011
#define RELAY_3_ON 2012
#define RELAY_3_OFF 2013
#define RELAY_4_ON 2014
#define RELAY_4_OFF 2015
#define RELAY_5_ON 2000
#define RELAY_5_OFF 2001
#define RELAY_6_ON 2002
#define RELAY_6_OFF 2003
#define RELAY_7_ON 2004
#define RELAY_7_OFF 2005
#define RELAY_8_ON 2006 // labjack 2
#define RELAY_8_OFF 2007 // labjack 2
// LABJACK 2
#define RELAY_9_ON 2008
#define RELAY_9_OFF 2009
#define RELAY_10_ON 2010
#define RELAY_10_OFF 2011
#define RELAY_11_ON 2012
#define RELAY_11_OFF 2013
#define RELAY_12_ON 2014
#define RELAY_12_OFF 2015
#define RELAY_13_ON 2000
#define RELAY_13_OFF 2001
#define RELAY_14_ON 2002
#define RELAY_14_OFF 2003
#define RELAY_15_ON 2004
#define RELAY_15_OFF 2005
#define RELAY_16_ON 2006 // labjack 1
#define RELAY_16_OFF 2007 // labjack 1
// LABJACK 3 EXTENDED
#define IF_RELAY_9_ON 2016
#define IF_RELAY_9_OFF 2017
#define IF_RELAY_10_ON 2018
#define IF_RELAY_10_OFF 2019


#endif /* relay_control_h */
