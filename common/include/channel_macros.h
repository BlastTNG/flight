/* 
 * channel_macros.h: 
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
 * Created on: Aug 19, 2014 by seth
 */

#ifndef CHANNEL_MACROS_H_
#define CHANNEL_MACROS_H_

#include <stdint.h>

/**
 * These intermediate types are necessary to allow GCC to use its strict-aliasing
 * optimization without breaking our big-endian/little-endian conversion logic
 */
typedef uint32_t __attribute__((__may_alias__)) u32_a;
typedef uint64_t __attribute__((__may_alias__)) u64_a;

typedef struct channel channel_t;

# ifndef _BSD_SOURCE
#   define _BSD_SOURCE             /* Define this for endian-conversion macros */
# endif
#include <portable_endian.h>

# if __BYTE_ORDER == __LITTLE_ENDIAN
#   define beftoh(x) ({                             \
            float   _tmp;                           \
            u32_a *infloat = (u32_a*)&(x);    \
            u32_a *outfloat = (u32_a*)(&_tmp);\
            *outfloat = be32toh(*infloat);          \
            _tmp;                                   \
    })
#   define bedtoh(x) ({                             \
            double   _tmp;                          \
            u64_a *infloat = (u64_a*)&(x);    \
            u64_a *outfloat = (u64_a*)(&_tmp);\
            *outfloat = be64toh(*infloat);          \
            _tmp;                                   \
    })
#   define htobed(in,out) {                         \
            double   in_dbl = (in);                 \
            u64_a *indouble = (u64_a*)&in_dbl;\
            u64_a *outdouble = (u64_a*)&(out);\
            *outdouble = htobe64(*indouble);        \
    }
#   define htobef(in,out)  {                        \
            float   in_flt = (in);                  \
            u32_a *infloat = (u32_a*)&in_flt; \
            u32_a *outfloat = (u32_a*)&(out); \
            *outfloat = htobe32(*infloat);          \
    }

# ifndef htobe16
#   define htobe16(x)  __bswap_16 (x)
# endif
# ifndef htobe32
#   define htobe32(x)  __bswap_32 (x)
# endif
# ifndef htobe64
#   define htobe64(x)  __bswap_64 (x)
# endif

# else
#   define beftoh(x) (x)
#   define bedtoh(x) (x)
#   define htobef(in,out) ((out)=(in))
#   define htobed(in,out) ((out)=(in))

# ifndef htobe16
#   define htobe16(x)  (x)
# endif
# ifndef htobe32
#   define htobe32(x)  (x)
# endif
# ifndef htobe64
#   define htobe64(x)  (x)
# endif

# endif

/**
 * If you know what type of data you are getting, these macros are preferable to the
 * type-agnostic one below.  However, be warned that if you change the type of the variable
 * in tx_struct, we can't protect you with these.
 */
#define GET_UINT8(_ch) *(uint8_t*)((_ch)->var)
#define GET_INT8(_ch) *(int8_t*)((_ch)->var)
#define GET_UINT16(_ch) be16toh(*(uint16_t*)((_ch)->var))
#define GET_INT16(_ch) (int16_t)be16toh(*(uint16_t*)((_ch)->var))
#define GET_UINT32(_ch) be32toh(*(uint32_t*)((_ch)->var))
#define GET_INT32(_ch) (int32_t)be32toh(*(int32_t*)((_ch)->var))
#define GET_UINT64(_ch) be64toh(*(uint64_t*)((_ch)->var))
#define GET_INT64(_ch) (int64_t)be64toh(*(int64_t*)((_ch)->var))
#define GET_FLOAT(_ch) beftoh(*(u32_a*)_ch->var)
#define GET_DOUBLE(_ch) bedtoh(*(u64_a*)_ch->var)

/**
 * Provides a type-agnostic return function
 */
#define GET_VALUE(channel,out)                      \
({                                                  \
    channel_t *_ch = channel;                       \
    if (!_ch)                                       \
        blast_fatal(#channel " is NULL!  Fix!"); \
    switch (_ch->type)   {                          \
        case TYPE_INT8:                             \
            (out) = *(int8_t*)_ch->var;             \
            break;                                  \
        case TYPE_UINT8:                            \
            (out) = *(uint8_t*)_ch->var;            \
            break;                                  \
        case TYPE_INT16:                            \
            (out) = (int16_t)be16toh(*(uint16_t*)_ch->var);    \
            break;                                  \
        case TYPE_UINT16:                           \
            (out) = be16toh(*(uint16_t*)_ch->var);  \
            break;                                  \
        case TYPE_INT32:                            \
            (out) = (int32_t)be32toh(*(uint32_t*)_ch->var);    \
            break;                                  \
        case TYPE_UINT32:                           \
            (out) = be32toh(*(uint32_t*)_ch->var);  \
            break;                                  \
        case TYPE_INT64:                            \
            (out) = (int64_t)be64toh(*(uint64_t*)_ch->var);    \
            break;                                  \
        case TYPE_UINT64:                           \
            (out) = be64toh(*(uint64_t*)_ch->var);  \
            break;                                  \
        case TYPE_FLOAT:                            \
            (out) = beftoh(*(uint32_t*)_ch->var);   \
            break;                                  \
        case TYPE_DOUBLE:                           \
            (out) = bedtoh(*(uint64_t*)_ch->var);   \
            break;                                  \
        default:                                    \
            out = (typeof(out))-1.0;                \
            blast_err("Invalid type %d", _ch->type);  \
    }                                       \
})

/**
 * If you know what type of data you are setting, these macros are preferable to the
 * type-agnostic one below.  However, be warned that if you change the type of the variable
 * in tx_struct, we can't protect you with these.
 */
#define SET_UINT8(_ch,_val)  (*(uint8_t*)((_ch)->var) = (_val))
#define SET_INT8(_ch,_val)   (*(int8_t*)((_ch)->var) = (_val))
#define SET_UINT16(_ch,_val) ({uint16_t _x = _val;(*(uint16_t*)((_ch)->var) = htobe16(_x));})
#define SET_INT16(_ch,_val)  ({int16_t _x = _val;(*(uint16_t*)((_ch)->var) = htobe16(_x));})
#define SET_UINT32(_ch,_val) ({uint32_t _x = _val;(*(uint32_t*)((_ch)->var) = htobe32(_x));})
#define SET_INT32(_ch,_val)  ({int32_t _x = _val;(*(uint32_t*)((_ch)->var) = htobe32(_x));})
#define SET_UINT64(_ch,_val) ({uint64_t _x = _val;(*(uint64_t*)((_ch)->var) = htobe64(_x));})
#define SET_INT64(_ch,_val)  ({int64_t _x = _val;(*(uint64_t*)((_ch)->var) = htobe64(_x));})
#define SET_FLOAT(_ch,_val) htobef(_val,*(uint32_t*)((_ch)->var))
#define SET_DOUBLE(_ch,_val) htobed(_val,*(uint64_t*)((_ch)->var))

#define SET_SCALED_VALUE(_channel,_val)         \
({                                              \
    channel_t *__ch = (_channel);               \
    double _new_val = (_val) - __ch->b_e2e;     \
    _new_val = _new_val / __ch->m_c2e;          \
    SET_VALUE(__ch,_new_val);                	\
})

/**
 * Provides a type-agnostic value setting function
 */
#define SET_VALUE(channel,in)               \
({                                          \
    channel_t *_ch = channel;               \
    typeof(in) _in = (in);                  \
    uint32_t in32u = (uint32_t)_in;         \
    int32_t in32i = (int32_t)_in;           \
    uint16_t in16u = (uint16_t)_in;         \
    int16_t in16i = (int16_t)_in;           \
    switch (_ch->type)   {                  \
        case TYPE_INT8:                     \
            *(int8_t*)_ch->var = (_in);     \
            break;                          \
        case TYPE_UINT8:                    \
            *(uint8_t*)_ch->var = (_in);    \
            break;                          \
        case TYPE_INT16:                    \
            *(uint16_t*)_ch->var = htobe16(in16i);     \
            break;                          \
        case TYPE_UINT16:                   \
            *(uint16_t*)_ch->var = htobe16(in16u);    \
            break;                          \
        case TYPE_INT32:                    \
            *(uint32_t*)_ch->var = htobe32(in32i);     \
            break;                          \
        case TYPE_UINT32:                   \
            *(uint32_t*)_ch->var = htobe32(in32u);    \
            break;                          \
        case TYPE_INT64:                    \
            *(uint64_t*)_ch->var = htobe64(in);     \
            break;                          \
        case TYPE_UINT64:                   \
            *(uint64_t*)_ch->var = htobe64(in);    \
            break;                          \
        case TYPE_FLOAT:                    \
            htobef(_in, _ch->var);          \
            break;                          \
        case TYPE_DOUBLE:                   \
            htobed(_in, _ch->var);          \
            break;                          \
        default:                            \
            blast_err("Invalid type %d", _ch->type);  \
    }                                   \
})
#endif /* CHANNEL_MACROS_H_ */
