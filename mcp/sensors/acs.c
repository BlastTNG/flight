/* 
 * acs.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Mar 24, 2015 by Seth Hillbrand
 */

#include <channels_tng.h>

void read_5hz_acs(void)
{
  double enc_raw_el;
  double x_comp, y_comp, z_comp;
  double pss1_i1, pss1_i2, pss1_i3, pss1_i4;
  double pss2_i1, pss2_i2, pss2_i3, pss2_i4;
  double pss3_i1, pss3_i2, pss3_i3, pss3_i4;
  double pss4_i1, pss4_i2, pss4_i3, pss4_i4;
  double vel_rw;
  double res_piv;
  int hwpr_pot;

  static channel_t* elRawEncAddr;
  static channel_t* elRawIfClinAddr;
  static channel_t* v11PssAddr;
  static channel_t* v21PssAddr;
  static channel_t* v31PssAddr;
  static channel_t* v41PssAddr;
  static channel_t* v12PssAddr;
  static channel_t* v22PssAddr;
  static channel_t* v32PssAddr;
  static channel_t* v42PssAddr;
  static channel_t* v13PssAddr;
  static channel_t* v23PssAddr;
  static channel_t* v33PssAddr;
  static channel_t* v43PssAddr;
  static channel_t* v14PssAddr;
  static channel_t* v24PssAddr;
  static channel_t* v34PssAddr;
  static channel_t* v44PssAddr;
  static channel_t* potHwprAddr;

  unsigned int rx_frame_index = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elRawEncAddr = channels_find_by_name("el_raw_enc");
    elRawIfClinAddr = channels_find_by_name("el_raw_if_clin");
    v11PssAddr = channels_find_by_name("v1_1_pss");
    v21PssAddr = channels_find_by_name("v2_1_pss");
    v31PssAddr = channels_find_by_name("v3_1_pss");
    v41PssAddr = channels_find_by_name("v4_1_pss");
    v12PssAddr = channels_find_by_name("v1_2_pss");
    v22PssAddr = channels_find_by_name("v2_2_pss");
    v32PssAddr = channels_find_by_name("v3_2_pss");
    v42PssAddr = channels_find_by_name("v4_2_pss");
    v13PssAddr = channels_find_by_name("v1_3_pss");
    v23PssAddr = channels_find_by_name("v2_3_pss");
    v33PssAddr = channels_find_by_name("v3_3_pss");
    v43PssAddr = channels_find_by_name("v4_3_pss");
    v14PssAddr = channels_find_by_name("v1_4_pss");
    v24PssAddr = channels_find_by_name("v2_4_pss");
    v34PssAddr = channels_find_by_name("v3_4_pss");
    v44PssAddr = channels_find_by_name("v4_4_pss");
    potHwprAddr = channels_find_by_name("pot_hwpr");
  }

  ///TODO: Add Clin read functions
  ///TODO: Add PSS read functions

}

void read_100hz_acs(void)
{
    static channel_t* xMagAddr;
    static channel_t* yMagAddr;
    static channel_t* zMagAddr;
    static channel_t* velRWAddr;
    static channel_t* resPivAddr;

    unsigned int rx_frame_index = 0;

    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
        xMagAddr = channels_find_by_name("x_mag");
        yMagAddr = channels_find_by_name("y_mag");
        zMagAddr = channels_find_by_name("z_mag");
        velRWAddr = channels_find_by_name("vel_rw");
        resPivAddr = channels_find_by_name("res_piv");
    }
    ///TODO: Add MAG read functions

}


void read_200hz_acs(void)
{
    double ifel_gy, ifroll_gy, ifyaw_gy;

    static channel_t* ifElgyAddr;
    static channel_t* ifRollgyAddr;
    static channel_t* ifYawgyAddr;

    unsigned int rx_frame_index = 0;

    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
      ifElgyAddr = channels_find_by_name("ifel_gy");
      ifRollgyAddr = channels_find_by_name("ifroll_gy");
      ifYawgyAddr = channels_find_by_name("ifyaw_gy");
    }

    ///TODO: Add if_gyro read functions
}
