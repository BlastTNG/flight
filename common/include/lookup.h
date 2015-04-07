/**
 * @file lookup.h
 *
 * @date Sep 1, 2010
 * @author Seth Hillbrand
 *
 * @brief This file was part of FCP, created for the EBEX project
 *
 * @details This file defines macros that create standardized
 * and expandable lookup tables.
 *
 * This software is copyright (C) 2010 Columbia University, 2014
 * California State University, Sacramento
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/**
 * We purposefully avoid the file-wide #ifndef BLAST_LOOKUP_H
 */

#ifndef BLAST_LOOKUP_TABLE_TEXT_SIZE
#	define BLAST_LOOKUP_TABLE_TEXT_SIZE 32
#endif

/**
 * There are three basic versions:
 * 1) Standard - This is just an enum and descriptive string.  For many applications, this
 * 				is sufficient.  You will get an array of enumerations, each of which define
 * 				the array index to the corresponding string.
 * 2) Function - This extends the Standard version to include a call-back function whose
 * 				definition is required.  The call-back function can handle processing specific
 * 				actions that should be associated with each enumeration.  This would be useful
 * 				in command processing, for example.
 * 3) Bitfield - This extends the Standard version to include a longer description field and
 * 				an bitfield integer (user-defined).  You should choose an integer representation
 * 				sufficiently long to represent all possible values.  Thus, if you have 16 or
 * 				more entries, you should choose uint32_t.  32 or more entries: uint64_t.
 *
 */

#ifndef _BLAST_ENUM_TYPEDEF
#define _BLAST_ENUM_TYPEDEF(_prefix)												\
	typedef enum _prefix														\
	{																			\
		_ ## _prefix ## S(_prefix,_BLAST_ENUM_LIST)								\
		_BLAST_ENUM_LIST(_prefix,END)											\
	} E_ ## _prefix
#endif

#ifndef _BLAST_LOOKUP_TABLE_DECLARATION
#define _BLAST_LOOKUP_TABLE_DECLARATION(_prefix, _struct_list, _type)			\
	_type _prefix ##_LOOKUP_T __attribute__((unused))							\
		_prefix ##_LOOKUP_TABLE[_prefix ## _END + 1] = 						\
	{_ ## _prefix ## S(_prefix,_struct_list)									\
	_BLAST_BASE_STRUCT_LIST(_prefix,END)										\
	}
#endif

#ifndef _BLAST_LOOKUP_TABLE_TYPEDEF
#define _BLAST_LOOKUP_TABLE_TYPEDEF(_prefix, _extras)							\
	typedef struct _prefix ## _LOOKUP  											\
	{																			\
		const char text[BLAST_LOOKUP_TABLE_TEXT_SIZE];							\
		E_ ## _prefix position;													\
		_extras																	\
	} _prefix ##_LOOKUP_T
#endif

#ifndef BLAST_GENERIC_LOOKUP_TABLE
#define BLAST_GENERIC_LOOKUP_TABLE(_prefix, _type, _tabledef, _listdef, _extras)	\
	_BLAST_ENUM_TYPEDEF(_prefix);												\
	_BLAST_LOOKUP_TABLE_TYPEDEF(_prefix,_tabledef);								\
	_extras																		\
	_BLAST_LOOKUP_TABLE_DECLARATION(_prefix,_listdef, _type)
#endif

#ifndef BLAST_FUNCTION_LOOKUP_TABLE
#define BLAST_FUNCTION_LOOKUP_TABLE(_prefix, _type)								\
	_ ## _prefix ## S(_prefix, _BLAST_FUNCTION_DEF )								\
	BLAST_GENERIC_LOOKUP_TABLE(	_prefix, 										\
								_type,											\
								void* (*function) (void*);, 					\
								_BLAST_FUNCTION_STRUCT_LIST,  )
#endif

#ifndef BLAST_HASH_LOOKUP_TABLE
#define BLAST_HASH_LOOKUP_TABLE(_prefix, _type)	BLAST_GENERIC_LOOKUP_TABLE(_prefix, _type, hash_t hash;, _BLAST_HASH_STRUCT_LIST, )
#endif

#ifndef BLAST_BITFIELD_LOOKUP_TABLE
#define BLAST_BITFIELD_LOOKUP_TABLE(_prefix, _fieldtype, _type)	BLAST_GENERIC_LOOKUP_TABLE(_prefix, _type, char desc[128];_fieldtype bitfield;, _BLAST_BITFIELD_STRUCT_LIST, )
#endif

/**
 * @param _prefix is a unique identifier for the table and enumeration names.  You should have a
 * #define already created with the _prefix name but pre-pended by an underscore '_' and
 * suffixed with an 'S'.
 * @param _type is an optional specifier that is valid for a struct.  Examples might be 'const' or
 * 'static' or some combination.  This will be prepended to the lookup table declaration
 *
 */
#ifndef BLAST_LOOKUP_TABLE
#define BLAST_LOOKUP_TABLE(_prefix, _type,...)									\
	_BLAST_ENUM_TYPEDEF(_prefix);												\
	_BLAST_LOOKUP_TABLE_TYPEDEF(_prefix, );										\
	_BLAST_LOOKUP_TABLE_DECLARATION(_prefix,_BLAST_BASE_STRUCT_LIST, _type)
#endif

#ifndef _BLAST_FUNCTION_DEF
#	define _BLAST_FUNCTION_DEF(_prefix,_ref) void *_prefix ## _ ## _ref ## _CALLBACK(void *);
#endif

#ifndef _BLAST_ENUM_LIST
#	define _BLAST_ENUM_LIST(_prefix,_ref,...) _prefix ## _ ## _ref,
#endif

#ifndef _BLAST_BASE_STRUCT_ENTRY
#	define _BLAST_BASE_STRUCT_ENTRY(_prefix,_ref) .text= #_ref "\0", .position= _BLAST_ENUM_LIST(_prefix, _ref)
#endif

#ifndef _BLAST_BASE_STRUCT_LIST
#	define _BLAST_BASE_STRUCT_LIST(_prefix,_ref,...) {_BLAST_BASE_STRUCT_ENTRY(_prefix,_ref)},
#endif

#ifndef _BLAST_FUNCTION_STRUCT_LIST
#	define _BLAST_FUNCTION_STRUCT_LIST(_prefix,_ref,...) {_BLAST_BASE_STRUCT_ENTRY(_prefix,_ref) _prefix ## _ ## _ref ## _CALLBACK},
#endif

#ifndef _BLAST_HASH_STRUCT_LIST
#	define _BLAST_HASH_STRUCT_LIST(_prefix,_ref,...) {_BLAST_BASE_STRUCT_ENTRY(_prefix,_ref) .hash = 0},
#endif

#ifndef _BLAST_BITFIELD_STRUCT_LIST
#	define _BLAST_BITFIELD_STRUCT_LIST(_prefix,_ref,_desc,...) {_BLAST_BASE_STRUCT_ENTRY(_prefix,_ref) .desc= #_desc "\0", .bitfield= 1<<_BLAST_ENUM_LIST(_prefix, _ref)},
#endif
