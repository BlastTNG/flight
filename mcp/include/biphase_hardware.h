/* fcp: the EBEX flight control program
 *
 * This software is copyright (C) 2009 Columbia University
 *                            (C) 2016 University of Pennsylvania
 *
 * This file is part of fcp.
 *
 * fcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_BIPHASE_HARDWARE_H
#define INCLUDE_BIPHASE_HARDWARE_H

#include <stdint.h>
#include "mpsse.h"

void reverse_bits(const size_t bytes_to_write, const uint16_t *msb_data, uint16_t *lsb_data_out);
int setup_mpsse(struct mpsse_ctx **ctx_ptr, const char *serial, uint8_t direction);


#endif /* INCLUDE_BIPHASE_INTERFACE */
