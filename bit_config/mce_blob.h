/* pcm: the Spider master control program
 *
 * mceserv.c: the MCE flight computer network server
 *
 * This software is copyright (C) 2012-2013 D. V. Wiebe
 *
 * pcm is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * pcm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pcm; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* 0xEB90 is the 16-bit Maury-Style optimum synchronisation sequence; see:
 *
 *   Maury, J. L. and Style, F. J., "Development of optimum frame
 *   synchronisation codes for Goddard space flight center PCM telemetry
 *   standards" in Proceedings of the National Telemetring Conference,
 *   Los Angeles, June 1964.
 *
 * 0xFAF320 is the 24-bit Maury-Style optimum synchronisation sequence.
 *
 * 0x146F is the inverse of 0xEB90.
 *
 * After the leadin is the blob serial number and then a 16-bit CRC of the
 * payload (repeated before the leadout)
 */
#define BLOB_LEADIN_LEN 12
#define BLOB_LEADIN {0, 0, 0, 0, 0, 0, 0, 0, 0xEB90, 0xFAF3, 0x2000, 0x146F}

/* 0xFE6B2840 is the 32-bit Muary-Style optimum synchronisation sequence.
 *
 * Before the leadout is a 16-bit CRC of the payload (a duplicate of the one
 * following the leadin)
 */
#define BLOB_LEADOUT_LEN 2
#define BLOB_LEADOUT {0xFE6B, 0x2840};

/* MCE_BLOB_MAX (defined in mpc_proto.h) is the payload size from MPC's point of
 * view.  The PCM payload is three words larger (for the mce#, type and size)
 */
#define MCE_BLOB_PAYLOAD_MAX (MCE_BLOB_MAX + 3)

/* In addition to the payload, the blob envelope contains the leadin and leadout
 * plus two CRCs, and the blob serial number
 */
#define MCE_BLOB_ENVELOPE_MAX (MCE_BLOB_PAYLOAD_MAX + BLOB_LEADIN_LEN + 3 + \
    BLOB_LEADOUT_LEN)

/* blob types */
#define BLOB_NONE    0
#define BLOB_EXPCFG  1
#define BLOB_IV      2
#define BLOB_TUNECFG 3
#define BLOB_TUNESQ_SA   4
#define BLOB_TUNESQ_SQ2  5
#define BLOB_TUNESQ_SQ1S 6
#define BLOB_TUNESQ_SQ1R 7
