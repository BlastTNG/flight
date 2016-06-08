/**
 * @file crc.h
 *
 * @date Aug 5, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_CRC_H
#define INCLUDE_CRC_H

#include <stdint.h>
#include <portable_endian.h>
#include <blast_compiler.h>
#define CRCPOLY_LE 0xedb88320
#define CRCPOLY_BE 0x04c11db7

extern uint32_t  crc32_le(uint32_t crc, unsigned char const *p, size_t len);
extern uint32_t  crc32_be(uint32_t crc, unsigned char const *p, size_t len);

#if __BYTE_ORDER == __LITTLE_ENDIAN
# define crc32(seed, data, length)  crc32_le(seed, (unsigned char const *)(data), length)
#else
# define crc32(seed, data, length)  crc32_be(seed, (unsigned char const *)(data), length)
#endif

uint16_t __pure crc16(uint16_t crc, void const *p, size_t len);

#endif /* _LINUX_CRC32_H */
