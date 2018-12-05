/* elog: elog config for groundstation BLAST command daemon
 *
 * This software is copyright (C) 2012 University of Toronto
 * 
 * This file is part of blastcmd.
 * 
 * blastcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * blastcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ELOG_H
#define ELOG_H

#define ENABLE_ELOG

#ifdef __MINI__
#undef ENABLE_ELOG
#endif

void ElogBeforeRoute(const char *buffer, const char *user);
void ElogAfterRoute(int result, int last_sock);

#endif
