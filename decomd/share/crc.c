/* crc.c: performs a crc checksum
 *
 * This software is copyright (C) 1997 Enzo Pascale
 * Updated by Adam Hincks, August 2004, for BLAST
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

unsigned short crctab[0x100] = {
  0x00000, 0x0C0C1, 0x0C181, 0x00140, 0x0C301, 0x003C0, 0x00280, 0x0C241,
  0x0C601, 0x006C0, 0x00780, 0x0C741, 0x00500, 0x0C5C1, 0x0C481, 0x00440,
  0X0CC01, 0x00CC0, 0x00D80, 0x0CD41, 0x00F00, 0x0CFC1, 0x0CE81, 0x00E40,
  0x00A00, 0x0CAC1, 0x0CB81, 0x00B40, 0x0C901, 0x009C0, 0x00880, 0x0C841,
  0x0D801, 0x018C0, 0x01980, 0x0D941, 0x01B00, 0x0DBC1, 0x0DA81, 0x01A40,
  0x01E00, 0x0DEC1, 0x0DF81, 0x01F40, 0x0DD01, 0x01DC0, 0x01C80, 0x0DC41,
  0x01400, 0x0D4C1, 0x0D581, 0x01540, 0x0D701, 0x017C0, 0x01680, 0x0D641,
  0x0D201, 0x012C0, 0x01380, 0x0D341, 0x01100, 0x0D1C1, 0x0D081, 0x01040,
  0x0F001, 0x030C0, 0x03180, 0x0F141, 0x03300, 0x0F3C1, 0x0F281, 0x03240,
  0x03600, 0x0F6C1, 0x0F781, 0x03740, 0x0F501, 0x035C0, 0x03480, 0x0F441,
  0x03C00, 0x0FCC1, 0x0FD81, 0x03D40, 0x0FF01, 0x03FC0, 0x03E80, 0x0FE41,
  0x0FA01, 0x03AC0, 0x03B80, 0x0FB41, 0x03900, 0x0F9C1, 0x0F881, 0x03840,
  0x02800, 0x0E8C1, 0x0E981, 0x02940, 0x0EB01, 0x02BC0, 0x02A80, 0x0EA41,
  0x0EE01, 0x02EC0, 0x02F80, 0x0EF41, 0x02D00, 0x0EDC1, 0x0EC81, 0x02C40,
  0x0E401, 0x024C0, 0x02580, 0x0E541, 0x02700, 0x0E7C1, 0x0E681, 0x02640,
  0x02200, 0x0E2C1, 0x0E381, 0x02340, 0x0E101, 0x021C0, 0x02080, 0x0E041,
  0x0A001, 0x060C0, 0x06180, 0x0A141, 0x06300, 0x0A3C1, 0x0A281, 0x06240,
  0x06600, 0x0A6C1, 0x0A781, 0x06740, 0x0A501, 0x065C0, 0x06480, 0x0A441,
  0x06C00, 0x0ACC1, 0x0AD81, 0x06D40, 0x0AF01, 0x06FC0, 0x06E80, 0x0AE41,
  0x0AA01, 0x06AC0, 0x06B80, 0x0AB41, 0x06900, 0x0A9C1, 0x0A881, 0x06840,
  0x07800, 0x0B8C1, 0x0B981, 0x07940, 0x0BB01, 0x07BC0, 0x07A80, 0x0BA41,
  0x0BE01, 0x07EC0, 0x07F80, 0x0BF41, 0x07D00, 0x0BDC1, 0x0BC81, 0x07C40,
  0x0B401, 0x074C0, 0x07580, 0x0B541, 0x07700, 0x0B7C1, 0x0B681, 0x07640,
  0x07200, 0x0B2C1, 0x0B381, 0x07340, 0x0B101, 0x071C0, 0x07080, 0x0B041,
  0x05000, 0x090C1, 0x09181, 0x05140, 0x09301, 0x053C0, 0x05280, 0x09241,
  0x09601, 0x056C0, 0x05780, 0x09741, 0x05500, 0x095C1, 0x09481, 0x05440,
  0x09C01, 0x05CC0, 0x05D80, 0x09D41, 0x05F00, 0x09FC1, 0x09E81, 0x05E40,
  0x05A00, 0x09AC1, 0x09B81, 0x05B40, 0x09901, 0x059C0, 0x05880, 0x09841,
  0x08801, 0x048C0, 0x04980, 0x08941, 0x04B00, 0x08BC1, 0x08A81, 0x04A40,
  0x04E00, 0x08EC1, 0x08F81, 0x04F40, 0x08D01, 0x04DC0, 0x04C80, 0x08C41,
  0x04400, 0x084C1, 0x08581, 0x04540, 0x08701, 0x047C0, 0x04680, 0x08641,
  0x08201, 0x042C0, 0x04380, 0x08341, 0x04100, 0x081C1, 0x08081, 0x04040 };


/*-----------------------------------------------------------------------------
 * CalculateCRC
 * 
 * This function computes the CRC used by SEA's ARC utility.  
 *----------------------------------------------------------------------------*/

unsigned short CalculateCRC(unsigned int initword, void *buffer, 
                            unsigned int buflen) {
  unsigned int k;
  unsigned short crc;
  unsigned char *b;

  if (buflen)
    crc = 0; 
  else 
    return 0;

  b = (unsigned char *)buffer;

  for (k = 0; k < buflen; k++)
   crc = ((crc >> 8) & 0x00ff) ^ crctab[(crc ^ b[k]) & 0x00ff];

  return crc;
}
