/****************************************************
  ss_packet_data contains the data sent to mcp
 ****************************************************/
#ifndef SS_STRUCT_H
#define SS_STRUCT_H


typedef struct
{
  float az_center;  //az pixel centroid [0,DATA0_WIDTH)
  float el_center;  //el pixel centroid [0,DATA1_WIDTH)
  int prin;         //current prin value [0,72]
  float az_snr;     //signal to noise ratio.
  float el_snr;     //~2 when there is no sun; ~30 when there is sun
  float cpu_temp;   //cpu temp from I2C bus (celsius)
  float pc_temp;    //motherboard temp from I2C bus (celsius)
} ss_packet_data;

#endif


