/* derived.h: a list of derived channels
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "channels.h"

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
    double m_c2e;               /* slope */
    double b_e2e;               /* intercept */
    char source2[FIELD_LEN];    /* Source Channel Name */
    double m2_c2e;              /* slope */
    double b2_e2e;              /* intercept */
  } lincom2;
  struct {
    char type;                  /* Should = '#' for comment */
    char text[256];             /* comment */
  } comment;
};

#define DERIVED_EOC_MARKER '!'
#define LINCOM(f,s,m,b) {.lincom = { 'c' , f , s , m , b }}
#define LINCOM2(f,s1,m1,b1,s2,m2,b2) {.lincom2 = { '2' , f , s1 , m1 , b1 ,\
  s2 , m2 , b2 }}
#define LINTERP(f,s,l) {.linterp = { 't' , f , s , l }}
#define BITWORD(f,s,o,l) {.bitword = { 'w' , f , s , o, l }}
#define BITFIELD(s, ...) {.bitfield = { 'b' , s , { __VA_ARGS__ }}}
#define COMMENT(c) {.comment = { '#' , c }}
#define END_OF_DERIVED_CHANNELS {.comment = { DERIVED_EOC_MARKER , "" }}
