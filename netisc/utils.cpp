/***************************************************************************
*  FILENAME: utils.c
*
*  PURPOSE: Contains misc utility functions used for the Pointing Control 
*           process.
*
*  DATE CREATED: 2/2/04        AUTHOR: D. Hardison
****************************************************************************
*  REVISION HISTORY
*  DATE    AUTHOR                   DESCRIPTION
*  4/26/04 W. Moleski       Added wordsToDouble routine needed to convert
*                           the GPS seconds of the week value being put
*                           in word 10 (housekeeping word).
****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>

#include "utils.h"

/***************************************************************************
*  FUNCTION: wordsToFloat
*
*  PURPOSE: Takes pointers to two 16 bit words and returns them 
*           as a single floating point value. Needed because all the
*           telemetery data parameters are stored as 16 bit words.
*
****************************************************************************/
float wordsToFloat(uint16_t *pMswSrc,  /* Most Significant Word   */
                    uint16_t *pLswSrc)  /* Least Significant Word  */
{
/* union to make the data manipulation a little easier */
union convert {
    uint16_t wordVals[2];     
    float floatVal;     
};

/* variable of union type */
union convert toFloat;

/* Read in the data */
toFloat.wordVals[0] = *pLswSrc;  /* x86 architecture is little endian    */
toFloat.wordVals[1] = *pMswSrc;  /* so lower order bits go in lower word */

return (toFloat.floatVal);
}

/***************************************************************************
*  FUNCTION: floatToWords
*
*  PURPOSE: Takes a floating point value and returns them as two 16 bit 
*           words. Needed because all the telemetery data parameters are 
*           stored as 16 bit words.
*
****************************************************************************/
void floatToWords(float floatNum,    /* Float number to split up            */
                  uint16_t *pMswDest, /* Where to put Most Significant Word  */
                  uint16_t *pLswDest) /* Where to put Least Significant Word */
{
/* union to make the data manipulation a little easier */
union convert {
    uint16_t wordVals[2];     
    float floatVal;     
};

/* variable of union type */
union convert toWords;

/* Read in the data */
toWords.floatVal = floatNum;

/* Store away the pieces */
*pLswDest = toWords.wordVals[0];  /* x86 architecture is little endian     */
*pMswDest = toWords.wordVals[1];  /* so lower order bits are in lower word */
}

/***************************************************************************
*  FUNCTION: wordsToInt
*
*  PURPOSE: Takes pointers to two 16 bit words and returns them 
*           as a single integer value. Needed because all the
*           telemetery data parameters are stored as 16 bit words.
*
****************************************************************************/
int wordsToInt (uint16_t *pMswSrc,  /* Most Significant Word   */
                uint16_t *pLswSrc)  /* Least Significant Word  */
{
/* union to make the data manipulation a little easier */
union convert {
    uint16_t wordVals[2];     
    int intVal;     
};

/* variable of union type */
union convert toInt;

/* Read in the data */
toInt.wordVals[0] = *pLswSrc;  /* x86 architecture is little endian    */
toInt.wordVals[1] = *pMswSrc;  /* so lower order bits go in lower word */

return (toInt.intVal);
}

/***************************************************************************
*  FUNCTION: intToWords
*
*  PURPOSE: Takes a integer value and returns them as two 16 bit 
*           words. Needed because all the telemetery data parameters are 
*           stored as 16 bit words.
*
****************************************************************************/
void intToWords( int intNum,         /* Integer number to split up          */
                  uint16_t *pMswDest, /* Where to put Most Significant Word  */
                  uint16_t *pLswDest) /* Where to put Least Significant Word */
{
/* union to make the data manipulation a little easier */
union convert {
    uint16_t wordVals[2];     
    int intVal;     
};

/* variable of union type */
union convert toWords;

/* Read in the data */
toWords.intVal = intNum;

/* Store away the pieces */
*pLswDest = toWords.wordVals[0];  /* x86 architecture is little endian     */
*pMswDest = toWords.wordVals[1];  /* so lower order bits are in lower word */
}

/***************************************************************************
*  FUNCTION: wordsToDouble
*
*  PURPOSE: Takes pointers to four 16 bit words and returns them 
*           as a double-precision floating point value.
*
****************************************************************************/
double wordsToDouble (uint16_t *pMsw1Src, uint16_t *pMsw2Src,  /* Most Significant Words   */
                      uint16_t *pLsw1Src, uint16_t *pLsw2Src)  /* Least Significant Words  */
{
/* union to make the data manipulation a little easier */
union convert {
    uint16_t wordVals[4];     
    double dVal;
};

/* variable of union type */
union convert toDouble;

    /* Read in the data */
    toDouble.wordVals[0] = *pLsw1Src;
    toDouble.wordVals[1] = *pLsw2Src;   /* x86 architecture is little endian    */
    toDouble.wordVals[2] = *pMsw1Src;   /* so lower order bits go in lower word */
    toDouble.wordVals[3] = *pMsw2Src;

    return (toDouble.dVal);
}

/***************************************************************************
*  FUNCTION: doubleToWords
*
*  PURPOSE: Takes pointers to four 16 bit words and returns them 
*           as a double-precision floating point value.
*
****************************************************************************/
void doubleToWords (double dNum, 
        uint16_t *pMsw1Dest, uint16_t *pMsw2Dest,  /* Most Significant Words   */
        uint16_t *pLsw1Dest, uint16_t *pLsw2Dest)  /* Least Significant Words  */
{
/* union to make the data manipulation a little easier */
union convert {
    uint16_t wordVals[4];     
    double dVal;
};

/* variable of union type */
union convert toWords;

    /* Read in the data */
    toWords.dVal = dNum;

    /* Store away the pieces */
    *pLsw1Dest = toWords.wordVals[0];
    *pLsw2Dest = toWords.wordVals[1];  /* x86 architecture is little endian     */
    *pMsw1Dest = toWords.wordVals[2];  /* so lower order bits are in lower word */
    *pMsw2Dest = toWords.wordVals[3];
}