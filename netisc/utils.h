/***************************************************************************
*  FILENAME: utils.h
*
*  PURPOSE: Contains function prototypes for the utility functions defined
*           in utils.c.
*
*  DATE CREATED: 2-2-04    AUTHOR: W. Moleski
****************************************************************************
*  REVISION HISTORY
*  DATE    AUTHOR                   DESCRIPTION
*
****************************************************************************/
#ifndef __UTILS_H   /* eliminates redundant includes */
#define __UTILS_H

//#include <stdint.h>
typedef unsigned short uint16_t;

float wordsToFloat(uint16_t *, uint16_t *);
void floatToWords(float, uint16_t *, uint16_t *);
int wordsToInt (uint16_t *, uint16_t *);
void intToWords (int, uint16_t *, uint16_t *);
double wordsToDouble(uint16_t *, uint16_t *,uint16_t *, uint16_t *);
void doubleToWords(double, uint16_t *, uint16_t *,uint16_t *, uint16_t *);

extern void logPupErr (uint16_t errCode);

#endif /* #ifndef __UTILS_H */