/* udp: generic UDP stuff for flight code
 *
 * Copyright (c) 2013, D. V. Wiebe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * This software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. In no event shall the copyright holder or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */

#include <sys/types.h>

/* maximum length of a hostname */
#define UDP_MAXHOST 1025

/* maximum length of a datagram */
#define UDP_MAXSIZE 65536

/* udp_bind_port: create and bind a socket to a UDP port on the local interfaces
 *
 * Inputs:
 * - port: the port to bind to
 * - bcast: pass non-zero to enable datagram broadcasting
 *
 * Returns:
 * - the socket descriptor of the bound port or -1 on error
 */
int udp_bind_port(int port, int bcast);

/* udp_recv: wait (with optional timeout) for a datagram on a bound socket
 *
 * Inputs:
 *  - sock: bound socket
 *  - msec: milliseconds before timeout; pass 0 to wait forever
 *  - peer: a pointer to a buffer of length UDP_MAXHOST bytes, or NULL if the
 *          caller doesn't care about that
 *  - port: a pointer to an int into which to store the source port, or NULL if
 *          the caller doesn't care about that
 *  - len:  the length of the data buffer
 *  - data: a pointer to a memory location of legnth 'len' bytes
 * 
 * Outputs:
 *  - peer: the sender's hostname
 *  - port: the sender's port
 *  - data: the datagram
 *
 * Returns:
 *  - datagram size, 0 on timeout, or -1 on error
 */
ssize_t udp_recv(int sock, int msec, char *peer, int *port, size_t len,
    char *data);

/* udp_bcast: broadcast a datagram
 *
 * Inputs:
 * - sock: bound socket
 * - port: destination port
 * - len : the length of the datagram
 * - data: a pointer to the datagram
 * - veto: if non-zero, do nothing.
 *
 * Returns:
 * - 0 on success, non-zero on error
 */
int udp_bcast(int sock, int port, size_t len, const char *data, int veto);
