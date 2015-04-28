/* derived.h: a list of derived channels
 *
 * This software is copyright (C) 2004-2010 University of Toronto
 *
 * This file is part mcp licensed under the GNUGeneral Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef DERIVED_H
#define DERIVED_H

#include "channels_tng.h"

#pragma pack(push,1)

typedef struct {
    char type;
    union {
      struct {                      /* BITS */
                                    /* type Should = 'b' for bits */
        char field[FIELD_LEN];      /* Derived Channel Name */
        char source[FIELD_LEN];     /* Source Channel Name */
        char offset;                /* Fisrt Bit */
        char length;                /* Number of Bits */
      } bitword;
      struct {                      /* LINTERP */
                                    /* type Should = 't' for linterp */
        char field[FIELD_LEN];      /* Derived Channel Name */
        char source[FIELD_LEN];     /* Source Channel Name */
        char lut[256];              /* Look-up table filename */
      } linterp;
      struct {                      /* LINCOM */
                                    /* type Should = 'c' for lincom */
        char field[FIELD_LEN];      /* Derived Channel Name */
        char source[FIELD_LEN];     /* Source Channel Name */
        double m_c2e;               /* slope */
        double b_e2e;               /* intercept */
      } lincom;
      struct {                      /* 2 FIELD LINCOM */
                                    /* type Should = '2' for lincom2 */
        char field[FIELD_LEN] ;     /* Derived Channel Name */
        char source[FIELD_LEN];     /* Source Channel Name */
        char source2[FIELD_LEN];    /* Source Channel Name */
        double m_c2e;               /* slope */
        double b_e2e;               /* intercept */
        double m2_c2e;              /* slope */
        double b2_e2e;              /* intercept */
      } lincom2;
      struct {
                                    /* type Should = '#' for comment */
        char text[343];             /* comment */
      } comment;
      struct {
                                    /* type should be 'u' for units */
        char source[FIELD_LEN];     /* the channel we are adding metadata for */
        char quantity[UNITS_LEN];   /* the quantity -eg, "Temperature" */
        char units[UNITS_LEN];      /* the Units -eg "^oC" */
      } units;
      struct {
                                    /* type should be 'p' for phase */
        char field[FIELD_LEN];      /* Derived Channel Name */
        char source[FIELD_LEN];     /* Source Channel Name */
        int shift;                  /* Phase shift in frames */
      } phase;
      struct {
                                    /* type should be 'r' for reciprocal */
        char field[FIELD_LEN];      /* Derived Channel Name */
        char source[FIELD_LEN];     /* Source Channel Name */
        double dividend;            /* Dividend */
      } recip;
      struct {                      /* MULTIPLY or DIVIDE */
                                    /* type '*' for multiply, '/' for divide */
        char field[FIELD_LEN];      /* Derived Channel Name */
        char source[FIELD_LEN];     /* 1st Source Channel, dividend for divide*/
        char source2[FIELD_LEN];    /* 2nd Source Channel, divisor for divide */
      } math;
      struct {                      /* MPLEX */
                                    /* type 'x'  for mplex */
        char field[FIELD_LEN];      /* derived channel name */
        char source[FIELD_LEN];     /* source channel */
        char index[FIELD_LEN];      /* multiplex index channel */
        int value;                  /* mulitplex index value to extract */
        int max;                    /* max multiplex index range (0 if unknown) */
      } mplex;
    };
} derived_tng_t;

typedef struct {
    uint32_t        magic;
    uint8_t         version;
    uint32_t        length;
    uint32_t        crc;
    derived_tng_t   data[0];
} derived_header_t;
#pragma pack(pop)

#define DERIVED_EOC_MARKER '!'
#define LINCOM(f,s,m,b) {.type = 'c', .lincom = { f , s , m , b }}
#define LINCOM2(f,s1,m1,b1,s2,m2,b2) {.type = '2', .lincom2 = { f, s1, s2, m1, b1 ,\
  m2 , b2 }}
#define LINTERP(f,s,l) {.type = 't', .linterp = { f , s , l }}
#define BITWORD(f,s,o,l) {.type = 'w', .bitword = { f , s , o, l }}
#define COMMENT(c) {.type = '#', .comment = { c }}
#define UNITS(s,q,u) {.type = 'u', .units = { s , q , u}}
#define PHASE(f,s,p) {.type = 'p', .phase = { f, s, p }}
#define RECIP(f,s,d) {.type = 'r', .recip = { f, s, d }}
#define DIVIDE(f,s1,s2) {.type = '/', .math = { f, s1, s2 }}
#define MULTIPLY(f,s1,s2) {.type = '*', .math = { f, s1, s2 }}
#define MPLEX(f,s,i,v,m) {.type = 'x', .mplex = { f, s, i, v, m }}
#define END_OF_DERIVED_CHANNELS {.type = DERIVED_EOC_MARKER}

#endif
