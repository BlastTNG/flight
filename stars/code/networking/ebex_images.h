/**
 * @file ebex_images.h
 *
 * @date Aug 6, 2012
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef EBEX_IMAGES_H_
#define EBEX_IMAGES_H_
#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif
#include <stdint.h>

#define EBEX_IMAGES_PORT 33789
static const uint32_t ebex_images_magic = 0xCDEDBD15;
static const uint32_t ebex_images_max_packet_size = 12 * 1024 * 1024; /// 12MB

#ifdef _MSC_VER
#pragma pack(push, header_packing, 1)
#endif
typedef struct
{
	uint32_t 	magic;
	uint32_t 	length;			///!< length of data in bytes exclusive of header
	char		filename[64];
	uint8_t  	data[];
}
#ifdef __GNUC__
__attribute__((packed))
#elif defined (_MSC_VER)
#pragma pack(pop, header_packing)
#endif
ebex_images_header_t;

#ifdef _MSC_VER
#pragma pack(push, result_packing, 1)
#endif
typedef struct
{
	int32_t		result;
}
#ifdef __GNUC__
__attribute__((packed))
#elif defined (_MSC_VER)
#pragma pack(pop, result_packing)
#endif
ebex_images_result_t;

typedef enum
{
	ebex_images_result_disk_error = -3,
	ebex_images_result_out_of_memory = -2,
	ebex_images_result_incompressible = -1,
	ebex_images_result_success = 0
} e_ebex_images_result;

bool ebex_images_init_server(void);
void ebex_images_shutdown_server(void);

#ifdef __cplusplus
}
#endif
#endif /* EBEX_IMAGES_H_ */
