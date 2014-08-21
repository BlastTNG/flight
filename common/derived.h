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

#pragma pack(4)

union DerivedUnion {
  struct {                      /* BIT FIELD */
    char type;                  /* Should = 'b' for bitfield */
    char source[FIELD_LEN];     /* Source Channel Name */
    char field[16][FIELD_LEN];  /* Derived Channel Names */
  } bitfield;
  struct {                      /* BITWORD */
    char type;                  /* Should = 'w' for bitword */
    char field[FIELD_LEN];      /* Derived Channel Name */
    char source[FIELD_LEN];     /* Source Channel Name */
    char offset;                /* Fisrt Bit */
    char length;                /* Number of Bits */
  } bitword;
  struct {                      /* LINTERP */
    char type;                  /* Should = 't' for linterp */
    char field[FIELD_LEN];      /* Derived Channel Name */
    char source[FIELD_LEN];     /* Source Channel Name */
    char lut[256];              /* Look-up table filename */
  } linterp;
  struct {                      /* LINCOM */
    char type;                  /* Should = 'c' for lincom */
    char field[FIELD_LEN];      /* Derived Channel Name */
    char source[FIELD_LEN];     /* Source Channel Name */
    double m_c2e;               /* slope */
    double b_e2e;               /* intercept */
  } lincom;
  struct {                      /* 2 FIELD LINCOM */
    char type;                  /* Should = '2' for lincom2 */
    char field[FIELD_LEN] ;     /* Derived Channel Name */
    char source[FIELD_LEN];     /* Source Channel Name */
    char source2[FIELD_LEN];    /* Source Channel Name */
    double m_c2e;               /* slope */
    double b_e2e;               /* intercept */
    double m2_c2e;              /* slope */
    double b2_e2e;              /* intercept */
  } lincom2;
  struct {
    char type;                  /* Should = '#' for comment */
    char text[343];             /* comment */
  } comment;
  struct {
    char type;                  /* should be 'u' for units */
    char source[FIELD_LEN];     /* the channel we are adding metadata for */
    char quantity[155];         /* the quantity -eg, "Temperature" */
    char units[155];            /* the Units -eg "^oC" */
  } units;
  struct {
    char type;                  /* should be 'p' for phase */
    char field[FIELD_LEN];      /* Derived Channel Name */
    char source[FIELD_LEN];     /* Source Channel Name */
    int shift;                  /* Phase shift in frames */
  } phase;
  struct {
    char type;                  /* should be 'r' for reciprocal */
    char field[FIELD_LEN];      /* Derived Channel Name */
    char source[FIELD_LEN];     /* Source Channel Name */
    double dividend;            /* Dividend */
  } recip;
  struct {                      /* MULTIPLY or DIVIDE */
    char type;                  /* '*' for multiply, '/' for divide */
    char field[FIELD_LEN];      /* Derived Channel Name */
    char source[FIELD_LEN];     /* 1st Source Channel, dividend for divide*/
    char source2[FIELD_LEN];    /* 2nd Source Channel, divisor for divide */
  } math;
  struct {                      /* MPLEX */
    char type;                  /* 'x'  for mplex */
    char field[FIELD_LEN];      /* derived channel name */
    char source[FIELD_LEN];     /* source channel */
    char index[FIELD_LEN];      /* multiplex index channel */
    int value;                  /* mulitplex index value to extract */
    int max;                    /* max multiplex index range (0 if unknown) */
  } mplex;
};

#pragma pack()

#define DERIVED_EOC_MARKER '!'
#define LINCOM(f,s,m,b) {.lincom = { 'c' , f , s , m , b }}
#define LINCOM2(f,s1,m1,b1,s2,m2,b2) {.lincom2 = { '2' , f, s1, s2, m1, b1 ,\
  m2 , b2 }}
#define LINTERP(f,s,l) {.linterp = { 't' , f , s , l }}
#define BITWORD(f,s,o,l) {.bitword = { 'w' , f , s , o, l }}
#define BITFIELD(s, ...) {.bitfield = { 'b' , s , { __VA_ARGS__ }}}
#define COMMENT(c) {.comment = { '#' , c }}
#define UNITS(s,q,u) {.units = { 'u' , s , q , u}}
#define PHASE(f,s,p) {.phase = { 'p', f, s, p }}
#define RECIP(f,s,d) {.recip = { 'r', f, s, d }}
#define DIVIDE(f,s1,s2) {.math = { '/', f, s1, s2 }}
#define MULTIPLY(f,s1,s2) {.math = { '*', f, s1, s2 }}
#define MPLEX(f,s,i,v,m) {.mplex = { 'x', f, s, i, v, m }}
#define END_OF_DERIVED_CHANNELS {.comment = { DERIVED_EOC_MARKER , "" }}

#endif
