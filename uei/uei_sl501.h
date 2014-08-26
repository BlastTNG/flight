/* 
 * uei_sl501.h
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of uei, created for the BLASTPol Project.
 *
 * uei is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * uei is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with uei; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 26, 2014 by seth
 */
 

#ifndef UEI_SL501_H_
#define UEI_SL501_H_

#define GYRO_SERIAL_SLOT 2

int uei_serial_232_initialize(int m_handle, int m_device, int m_channel, uint32_t m_config);

#endif /* UEI_SL501_H_ */
