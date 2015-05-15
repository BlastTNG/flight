/*
 * crc32.h
 */
#ifndef _CRC32_H
#define _CRC32_H

#include <stdint.h>
#include <portable_endian.h>
#define CRCPOLY_LE 0xedb88320
#define CRCPOLY_BE 0x04c11db7

extern uint32_t  crc32_le(uint32_t crc, unsigned char const *p, size_t len);
extern uint32_t  crc32_be(uint32_t crc, unsigned char const *p, size_t len);

#if __BYTE_ORDER == __LITTLE_ENDIAN
# define crc32(seed, data, length)  crc32_le(seed, (unsigned char const *)(data), length)
#else
# define crc32(seed, data, length)  crc32_be(seed, (unsigned char const *)(data), length)
#endif


#endif /* _LINUX_CRC32_H */
