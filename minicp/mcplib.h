/*
 * mcplib:
 * Interface to communicate with mcp and BLAST bus
 *
 * This software is copyright (C) 2007 University of Toronto
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
 */
#ifndef _MCPLIB_H
#define _MCPLIB_H

/*******************************************************************************
 *                        Guide to Using mcplib                                *
 ******************************************************************************/
/* mcplib must be started by calling BB_start()
 *
 * To cleanly shut down the electronics, call BB_stop().
 * While not critical, this is highly recommended (otherwise may need to reset 
 * the PCI card/computer, or at least reinsert its module).
 */
/* Current data from the BLAST bus is obtained with:
 * unsigned char BB_getDigital(short card, short group);
 *   to read a digital input group (from 0-2 inclusive) on an ADC card (0-3)
 * unsigned int BB_getAnalog(short card, short channel);
 *   to read an analog channel (0-24) on a given ADC card (0-3)
 * unsigned int BB_getIndex();
 *   to get the current frame index/serial number
 *
 * To update the data obtained by these functions, call BB_receive()
 * The BLAST bus gets data at 100Hz, so BB_receive is optimally called at 100Hz
 */
/* Set data to send over the BLAST bus using:
 * void BB_setDigital(short card, short group, unsigned char val);
 *   to give a value (val) to a digital group (0-2) on an ADC card (0-3)
 * void BB_setDAC(short card, short dac, unsigned short val);
 *   to give a 16-bit amplitude (P-P for AC bias, level for DC channels) to an
 *   SPI DAC (0-31) on an ADC card (0-3)
 * 
 * To actually send the data, call BB_send().
 * The BLAST bus sends data at 100Hz. Sending faster will skip outputs
 * When BB_send has not been called, the most recently sent data is resent
 */
/* Giving a file pointer to BB_setOutfile will redirect mcp's output (which
 * would normally be sent to stdout). By default, it is NULL (off).
 */
/* Passing a true argument to BB_useFramefile will cause mcp to write frames to
 * a file as it would in standalone mode. This is off by default, but possibly
 * useful for debugging. BB_useFramefile can probably be called to turn writing
 * on and off, but is most useful called once before calling BB_start.
 */

#include <stdio.h>

void BB_start();
void BB_stop();

void BB_receive();
void BB_send();

unsigned char BB_getDigital(short card, short group);
unsigned int BB_getAnalog(short card, short channel);
unsigned int BB_getIndex();

void BB_setDigital(short card, short group, unsigned char val);
void BB_setDAC(short card, short dac, unsigned short val);

void BB_setOutfile(FILE* file);
void BB_useFramefile(int flag);

#endif //_MCPLIB_H

