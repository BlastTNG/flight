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

//remove this to disable use of elog (or don't unclude this header)
#define ENABLE_ELOG

#if defined __BLAST__
#define ELOG_HOST   "elog.blastpol.com"
#define ELOG_PORT   "8000"
#define ELOG_LOG    "blastcmd2012"
#define ELOG_AUTH   "blastcmd-daemon"
#elif defined __SPIDER__
#define ELOG_HOST   "someplace.update.me"
#define ELOG_PORT   "8080"
#define ELOG_LOG    "LogName"
#define ELOG_AUTH   "spidercmd-daemon"
#endif

#if defined ENABLE_ELOG && defined ELOG_HOST
#define ELOG_CMD "/usr/local/bin/elog -h " ELOG_HOST " -p " ELOG_PORT \
 " -l " ELOG_LOG " -a Author=" ELOG_AUTH " -a Source=blastcmd "
#endif


#endif
