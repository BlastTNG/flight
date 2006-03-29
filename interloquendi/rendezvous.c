/* interloquendi: copies a mcp frame file to a TCP port
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of interloquendi.
 * 
 * interloquendi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * interloquendi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with interloquendi; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "quendiclient.h"
#include "blast.h"

int InitRendezvous(const char* host, int port, const char* masq)
{
  struct sockaddr_in addr;
  const char* herr;

  /* No rendezvous host means no rendezvousing */
  if (host[0] == '\0')
    return 0;

  if ((herr = ResolveHost(host, &addr, 0)) != NULL) 
    bprintf(fatal, "Unable to resolve upstream rendezvous server: %s", herr);

  return 0;
}
