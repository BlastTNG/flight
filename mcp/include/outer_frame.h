// Copyright 2016 Ian Lowe
//  relay_control.h
//
//
//  Created by Ian Lowe on 1/18/17.
//
//
#ifndef INCLUDE_OUTER_FRAME_H
#define INCLUDE_OUTER_FRAME_H

void update_thermistors(void);
void update_current_sensors(void);
void outer_frame(int setting);
void outer_frame_multiplexed(void);
void update_mult_vac(void);
#endif /* outer_frame_h */
