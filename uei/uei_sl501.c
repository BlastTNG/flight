/* 
 * uei_sl501.c
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

#include <stdint.h>
#include <stdio.h>

#include <PDNA.h>


int uei_serial_232_initialize(int m_handle, int m_device, int m_channel, uint32_t m_config)
{
    int ret = 0;
    // Set channel configuration
    if ((ret = DqAdv501SetChannelCfg(m_handle, m_device, m_channel, m_config))
            < 0) {
        printf("error %d in DqAdv501SetChannelCfg()\n", ret);
        return ret;
    }

    // Disable waiting for specified byte count.  Immediately returns available bytes.
    if ((ret = DqAdv501SetTimeout(m_handle, m_device, m_channel, 0)) < 0) {
        printf("error %d in DqAdv501SetTimeout\n", ret);
        return ret;
    }

    // Disable FIFO queuing.  Any bytes available will be sent
    if ((ret = DqAdv501SetTermLength(m_handle, m_device, m_channel, 0)) < 0) {
        printf("error %d in DqAdv501SetTermLength\n", ret);
        return ret;
    }

    // Sets RX to direct mode which disables interrupt processing and allows us to read directly
    // from the serial port's FIFO without any delay.
    if ((ret = DqAdv501SetWatermark(m_handle, m_device, m_channel,
            DQL_IOCTL501_SETRXWM_DIRECT, 0)) < 0) {
        printf("error %d in DqAdv501SetWatermark\n", ret);
        return ret;
    }

    return ret;
}

