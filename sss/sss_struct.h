/**************************************************************
 * sss_struct.h
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/

#ifndef SSS_STRUCT_H
#define SSS_STRUCT_H

/**********************************
  sss_packet_data contains the data
 **********************************/

#pragma pack(4)

typedef struct
{
  double az_rel_sun;   //calculated az of the sun [degrees]
  float  amp;          //amplitude of the fit [Counts]
  float  dc_off;       //dc offset of the fit [Counts]
  float  phase;        //phase of the fit [degrees]
  float  chi;          //chi^2 of the fit
  unsigned short iter; //number of iterations fitting loop took

  double sun_time;     //time of measurment (decimal ctime) [seconds]

  float t_cpu;         //cpu temperature [kelvin]
  float t_hdd;         //hard drive (ambient) temperature [kelvin]
  float t_case;        //base plate temperature [kelvin]
  float t_port;        //near module 4 temperature [kelvin]
  float t_starboard;   //near module 10 temperature [kelvin]

  float v5;            //Vcc (output from DC-DC) [volts]
  float v12;           //12V (output from DC-DC) [volts]
  float vbatt;         //Battery (input to DC-DC) [volts]

  unsigned int m01;    //Raw value of each module [Counts]
  unsigned int m02;    //which we could use for better analysis
  unsigned int m03;    //post-flight.  Or not.
  unsigned int m04;    //Save it if you have the room.
  unsigned int m05;    //It's of marginal interest.
  unsigned int m06;    //So are most of the housekeeping 
  unsigned int m07;    //values above.
  unsigned int m08;
  unsigned int m09;
  unsigned int m10;
  unsigned int m11;
  unsigned int m12;
}sss_packet_data;

#pragma pack()

#endif
