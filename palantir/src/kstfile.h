/***************************************************************************
                          kstfile.h  -  description
                             -------------------
    begin                : Thu Aug 24 2000
    copyright            : (C) 2000 by Barth Netterfield
    email                :
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef KSTFILE_H
#define KSTFILE_H

#include <qstring.h>
#include <qtextstream.h>
#include <qdom.h>

typedef enum {UNKNOWN, EMPTY, FRAME, ASCII, DIRFILE} KstFileType;

/**File structure for KST
  *@author Barth Netterfield
  */

class KstFile {
public:
  KstFile(const char *filename_in,
          const KstFileType newType = UNKNOWN);
  KstFile(QDomElement &e);
  ~KstFile();

  /** Updates number of samples.
      For ascii files, it also reads
      and writes to a temporary binary file.
      It returns 1 if there was new data */
  bool update();

  /** Reads a field from the file.  Data is returned in the
      double Array V[] */
  int readField(double *V, const char *field, int s, int n);

  /** Returns true if the field is valid, or false if it is not */
  bool isValidField(const char *field);

  /** Returns samples per frame for field <field>.  For
      ascii column data, this is always 1.  For frame data
      this could greater than 1. */
  int samplesPerFrame(const char *field);

  /** Returns the size of the file (in frames) as of last update */
  int numFrames();

  /** Returns the file name.
      The string is stored in a separate static variable, so changes
      to this are ignored.  It is updated each time the fn is called */
  char *fileName();

  /** Returns the file type or an error message in a static string
      The string is stored in a separate static variable, so changes
      to this are ignored.  It is updated each time the fn is called */
  QString fileType();

  /** Increments rvector useage for this file */
  void incUsage();

  /** Decrements rvector useage for this file */
  void decUsage();

  /** Returns useage for this file */
  int getUsage();

  /** Save file description info into stream ts */
  void save(QTextStream &ts);

private: // Private methods
  /** common part of the constructor: */
  void commonConstructor(const char *filename_in,
                         const KstFileType newType);

  /** Determines the file type */
  int determineType();

  /** Read filename from indirect file: return 1 if file changes */
  int readIFile();

  /** initialize files */
  void init();

  /** Read a field from an Ascii file.  n is number of frames, unless
   it is negative, in which case, read one sample, not entire frames */
  int asciiReadField(double *V, const char *field, int s, int n);

  /** Update an Ascii file: read lines, write to temp_file */
  bool asciiUpdate();

  /** Initializations for ascii files: */
  bool asciiInitFile();

  /** Returns true if the field is valid, or false if it is not */
  bool asciiIsValidField(const char *field);

  /** Read a field from a frame file */
  int frameReadField(double *V, const char *field, int s, int n);

  /** Update Frame Data file: determine length */
  bool frameUpdate();

  /** Initialization for Frame Files */
  bool frameInitFile();

  /** Returns true if the field is valid, or false if it is not */
  bool frameIsValidField(const char *field);

  /** Determine samples per frame for Frame Files */
  int frameSampsPerFrame(const char *);

  /** Read a field from a dirfile file */
  int dirfileReadField(double *V, const char *field, int s, int n);

  /** Update Dirfile Data file: determine length */
  bool dirfileUpdate();

  /** Initialization for Dirfile Files */
  bool dirfileInitFile();

  /** Returns true if the field is valid, or false if it is not */
  bool dirfileIsValidField(const char *field);

  /** Determine samples per Frame for Dirfile Files */
  int dirfileSampsPerFrame(const char *);

  /** Update stdin temp file - true if more data is read */
  bool UpdateStdin();

private: // Private attributes
  /** Name of current file file */
  char IndirectFilename[255];
  bool IsIndirect;

  /** Name of the File */
  char Filename[255];

  /** File Type */
  KstFileType Type;

  /** How many RVectors are using this file */
  int NumUsed;

  /** Number of Frames availible */
  int NumFrames;

  /** Used by stdin files */
  bool IsStdin;
  char StdinFilename[64];

  /** used by ASCII files */
  int ByteLength;
  int *RowIndex;
  int NumLinesAlloc;

  /** Used by frame files */
  int BytePerFrame;
  int FramePerFile;
  char RootFileName[255];  // Name of file without hex extention
  int RootExt;             // Root Extention (number): -1 if no extention
  int MaxExt;              // The largest value the extention
};

#endif
