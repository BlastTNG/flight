#ifndef __CRC__
#define __CRC__
#define CRC_SEED 0xEB90
unsigned short CalculateCRC(unsigned int initword, void *buffer, 
                            unsigned int buflen);
#endif 
