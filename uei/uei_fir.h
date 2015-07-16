/* 
 * uei_fir.h: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
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
 * Created on: Aug 11, 2014 by seth
 *
 * This filter provides a 16-tap FIR for 100Hz output with first pole
 * at 50Hz and leakage above 50Hz of -46dB.
 */

#ifndef UEI_FIR_H_
#define UEI_FIR_H_

static double filter_taps[16] = {
  -20662.03603351745,
  242572.50786530972,
  -1317301.5060910601,
  4345357.280211401,
  -9571456.632508202,
  14405442.49607101,
  -13997154.815878868,
  5913203.238231595,
  5913203.238231595,
  -13997154.815878868,
  14405442.49607101,
  -9571456.632508202,
  4345357.280211401,
  -1317301.5060910601,
  242572.50786530972,
  -20662.03603351745
};


typedef struct {
  float history[16];
  unsigned int last_index;
} uei_fir_t;


static inline void uei_fir100_put(uei_fir_t* f, float input) {
  f->history[(f->last_index++) & 15] = input;
}

static inline float uei_fir100_get(uei_fir_t* f) {
  double acc = 0.0;
  int index = f->last_index, i;

  for(i = 0; i < 16; ++i) {
    acc += f->history[(index--) & 15] * filter_taps[i];
  };

  return (float) acc;
}
#endif /* UEI_FIR_H_ */
