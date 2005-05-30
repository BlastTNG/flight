/*                           (C) 2003 C. Barth Netterfield */
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
/***************************************************************/
/*                                                             */
/*    Structures for describing dirfile formats                */
/*                                                             */
/***************************************************************/
#ifndef GETDATA_STRUCT_H
#define GETDATA_STRUCT_H

#define FIELD_LENGTH 16
#define MAX_FILENAME_LENGTH 180
#define MAX_LINE_LENGTH 180
#define MAX_LINCOM 3

struct RawEntryType {
  char field[FIELD_LENGTH+1];
  int fp;
  char type;
  int size;
  int samples_per_frame;
};

struct LincomEntryType {
  char field[FIELD_LENGTH+1];
  int n_infields;
  char in_fields[MAX_LINCOM][FIELD_LENGTH+1];
  double m[MAX_LINCOM];
  double b[MAX_LINCOM];
};

struct LinterpEntryType {
  char field[FIELD_LENGTH+1];
  char raw_field[FIELD_LENGTH+1];
  char linterp_file[MAX_FILENAME_LENGTH];
  int n_interp;
  double *x;
  double *y;
};

struct MplexEntryType {
  char field[FIELD_LENGTH+1];
  char cnt_field[FIELD_LENGTH+1];
  char data_field[FIELD_LENGTH+1];
  int i;
  int max_i;
};

struct BitEntryType {
  char field[FIELD_LENGTH+1];
  char raw_field[FIELD_LENGTH+1];
  int bitnum;
  int numbits;
};

struct FormatType {
  char FileDirName[MAX_FILENAME_LENGTH];
  int frame_offset;
  struct RawEntryType *rawEntries;
  int n_raw;
  struct LincomEntryType *lincomEntries;
  int n_lincom;
  struct LinterpEntryType *linterpEntries;
  int n_linterp;
  struct MplexEntryType *mplexEntries;
  int n_mplex;
  struct BitEntryType *bitEntries;
  int n_bit;
};

#endif
