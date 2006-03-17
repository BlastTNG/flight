/* crc.c: performs a crc checksum
 *
 * This software is copyright (C) 1997 Enzo Pascale
 * Updated by Adam Hincks and D.V. Wiebe for BLAST, 2004
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __CRC__
#define __CRC__
#define CRC_SEED 0xEB90
unsigned short CalculateCRC(unsigned int initword, void *buffer,
                            unsigned int buflen);
#endif
