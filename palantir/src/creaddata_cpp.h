/*** Header file for creaddata.c, which reads barth's frame files ***/

/* define error codes */
#ifndef _READDATA_H
#include <readdata_cpp.h>
#endif

#define E_OPEN_CSFILE      10
#define E_CSFILE_FORMAT    11
#define E_SIZE_MISMATCH	   12
#define E_OPEN_LINFILE     13
#define E_RECURSE_LEVEL    14

/* Define conversion rule types */
#define CST_END 0
#define CST_LINCOM  1
#define CST_LINFILE 2
#define CST_MPLEX   3
#define CST_FRAMETIME 4
#define CST_BITFIELD 5
#define CST_SIN      6
#define CST_COS      7
#define CST_ASIN     8
#define ATAN2        9

extern char *CRD_ERROR_CODES[15];

extern "C" int CReadData(char *filename, char *field_code,
             int first_sframe, int first_samp,
             int num_sframes, int num_samp,
             char return_type, void *data_out,
             int *error_code);
