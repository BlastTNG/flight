/* --------------------------------------------------------------------------
                                       ACSMAIN                                 
   crc.h

   A program for the BOOMERanG ACS Ground Station

   (crc check sum)

   Enzo Pascale Jan. 25 1997
--------------------------------------------------------------------------- */
#ifndef CRC_h_
#define CRC_h_

#define Lo(byte_) (byte_ & 0x00ff)
#define Hi(byte_) ( (byte_ >> 8) & 0x00ff )

unsigned short UpdateCRCArc(unsigned short InitCRC, void *buffer,
			    unsigned int InLen);

#endif 
