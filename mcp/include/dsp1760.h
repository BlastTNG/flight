/* 
 * dsp1760.h: 
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
 * Created on: Dec 18, 2014 by seth
 */

#ifndef INCLUDE_DSP1760_H_
#define INCLUDE_DSP1760_H_
#include <stdbool.h>
#include <stdint.h>

#define GYRO_VAR (0.7E-6)

bool initialize_dsp1760_interface(void);
void dsp1760_reset_gyro(int m_gyrobox);

float dsp1760_getval(int m_box, int m_gyro);
uint8_t dsp1760_get_seq_error_count(int m_box);
uint8_t dsp1760_get_crc_error_count(int m_box);
uint8_t dsp1760_get_seq_number(int m_box);
uint8_t dsp1760_get_gyro_status_count(int m_box, int m_gyro);
uint32_t dsp1760_get_valid_packet_count(int m_box, int m_gyro);
uint32_t dsp1760_get_packet_count(int m_box);
int16_t dsp1760_get_temp(int m_box);

#endif /* DSP1760_H_ */
