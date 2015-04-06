/* 
 * defricher_utils.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of defricher, created for the BLASTPol Project.
 *
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 6, 2015 by Seth Hillbrand
 */

#ifndef DEFRICHER_UTILS_H_
#define DEFRICHER_UTILS_H_

#include <stdbool.h>
#include <blast.h>

#define defricher_fatal(fmt,...)                     \
    do {                                            \
        bprintf(fatal, fmt, ##__VA_ARGS__);    \
    }while(0)
#define defricher_tfatal(fmt,...)                    \
    do {                                            \
        bprintf(tfatal, fmt, ##__VA_ARGS__);   \
    }while(0)
#define defricher_err(fmt,...)                       \
    do {                                            \
        bprintf(err, fmt, ##__VA_ARGS__);      \
    }while(0)
#define defricher_info(fmt,...)                      \
    do {                                            \
        bprintf(info, fmt, ##__VA_ARGS__);     \
    }while(0)
#define defricher_warn(fmt,...)                      \
    do {                                            \
        bprintf(warning,fmt,  ##__VA_ARGS__);  \
    }while(0)
#define defricher_startup(fmt,...)                   \
    do {                                            \
        bprintf(startup, fmt, ##__VA_ARGS__);  \
    }while(0)
#define defricher_nolog(fmt,...)                     \
    do {                                            \
        bprintf(nolog,fmt,  ##__VA_ARGS__);    \
    }while(0)
#define defricher_mem(fmt,...)                       \
    do {                                            \
        bprintf(mem, fmt, ##__VA_ARGS__);      \
    }while(0)
#define defricher_dbg(fmt,...)                       \
    do {                                            \
        bprintf(dbg, fmt, ##__VA_ARGS__);      \
    }while(0)

#define defricher_strerr(fmt,...)                   \
    do {                                            \
        berror(err, fmt, ##__VA_ARGS__);      \
    }while(0)

int defricher_mkdir_file(const char *m_filename, bool m_file_appended);


#endif /* DEFRICHER_UTILS_H_ */
