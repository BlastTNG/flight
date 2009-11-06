/***************************************************************************
                          kstfile.cpp  -  A file class for KST
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

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <iostream>

#include <getdata.h>


#include "kstfile.h"


/************************************************************************/
/*                                                                      */
/*                Public Methods                                        */
/*                                                                      */
/************************************************************************/
KstFile::KstFile(const char *filename_in,
                 const KstFileType &newType) {
  commonConstructor(filename_in, newType);
}

void KstFile::commonConstructor(const char *filename_in,
                                const KstFileType &newType) {
  int len;

  NumFrames = 0;
  IsIndirect = false;
  ByteLength = NumLinesAlloc = 0;
  BytePerFrame = FramePerFile = 0;

  /* determine if this is an indirect file                               */
  /* an indirect file lists the name of the actual file to get data from */
  /* This is often useful in real-time data aquisition                   */
  len = strlen(filename_in);
  if (strcmp(filename_in+len-4, ".cur")==0) {
    strncpy(IndirectFilename, filename_in, 254);
    IsIndirect=true;
    Filename[0] = '\0';
    readIFile();
  } else {
    IsIndirect=0;
    strncpy(Filename, filename_in, 254);
    Filename[255]='\0';
  }

  Type = newType;
  init();
}


KstFile::~KstFile(){
  switch (Type) {
  case ASCII:
    free(RowIndex);
    break;
  case DIRFILE:
  case EMPTY:
  case UNKNOWN:
  default:
    break;
  }
}

/** Reads a field from the file.  Data is returned in the
double Array V[].  Returns number of samples read.  If n<0, read 1 sample,
not entire frames */
int KstFile::readField(double *V, const char *field, const int &s,
                       const int &in_n){

  int n;

  n = in_n;

  if (s > NumFrames-1) return(0);
  if (s+n > NumFrames) n=NumFrames-s;

  switch (Type) {
  case ASCII:
    return (asciiReadField(V, field, s,n));
  case DIRFILE:
    return (dirfileReadField(V, field, s,n));
  case EMPTY:
  case UNKNOWN:
    return (0);
  default:
    fprintf(stderr,"kst internal error: unknown type in ReadField\n");
    return(0);
  }
}

/** Returns the size of the file as of last update */
int KstFile::numFrames(){
  if (Type==EMPTY) return(0);
  else return(NumFrames);
}

/** Returns true if the field is valid, or false if it is not */
bool KstFile::isValidField(const char *field) {
  switch (Type) {
  case ASCII:
    return(asciiIsValidField(field));
  case DIRFILE:
    return (dirfileIsValidField(field));
  case EMPTY:
  case UNKNOWN:
    printf("empty or unknown\n");
    return (false);
  default:
    fprintf(stderr,"kst internal error: unknown type in sampsperframe\n");
   return(0);
  }
}


/** Returns samples per frame for field <field>.  For
ascii column data, this is always 1.  For frame data
this could greater than 1. */
int KstFile::samplesPerFrame(const char *field){

  switch (Type) {
  case ASCII:
    return(1);
  case DIRFILE:
    return (dirfileSampsPerFrame(field));
  case EMPTY:
  case UNKNOWN:
    return (0);
  default:
    fprintf(stderr,"kst internal error: unknown type in sampsperframe\n");
   return(0);
  }
}

/** Updates the size of the file.  For ASCII files, also reads and writes
    to temp binary file.  Return 1 if there was
    new data */
bool KstFile::update(){

  if (IsIndirect) {
    if (readIFile()==1) init();
  }

  if (Type==EMPTY) init();

  switch (Type) {
  case ASCII:
    return (asciiUpdate());
  case DIRFILE:
    return (dirfileUpdate());
  case EMPTY:
  case UNKNOWN:
    return (false);
  default:
    fprintf(stderr,"kst internal error: unknown type in Update\n");
    exit(0);
  }
}

/** Returns the file name.  It is stored in a separate static variable,
    so changes to this are ignored */
char *KstFile::fileName() {
  static char tmpstr[255];

  if (IsIndirect) {
    strcpy(tmpstr, IndirectFilename);
  } else {
    strcpy(tmpstr, Filename);
  }

  return (tmpstr);
}

/************************************************************************/
/*                                                                      */
/*                Private Methods: General                              */
/*                                                                      */
/************************************************************************/
/** Determines the file type : returns file type */
int KstFile::determineType(){
  int error_code;
  int i, nread, non_ascii;
  char readbuf[1024];
  FILE *tmp_fp;
  struct stat stat_buf;
  char *lfilename;

  lfilename = Filename;

  /**********************************************************/
  /* check to see if the file is a dirfile file (directory) */
  NumFrames = GetNFrames(lfilename, &error_code, NULL);
  if ((NumFrames>0) && (error_code == GD_E_OK)) {
    Type = DIRFILE;
    return (DIRFILE);
  }

  /**********************************************************/
  /* verify that the file exists; determine length in bytes */
  if (stat(lfilename, &stat_buf)==0) { /* file exists */
    ByteLength=stat_buf.st_size; // Used by ascii file type
  } else {
    Type = EMPTY;
    return (EMPTY);
  }

  /********************************************/
  /* test to see if the file is ascii collums */
  /* This is pretty unreliable so do it last  */
  /* Just search the first 1024 bytes for a   */
  /* non character.                           */
  tmp_fp = fopen(lfilename, "r");
  if (ByteLength>1024) nread = 1024;
  else nread = ByteLength;
  fread(readbuf, 1, nread, tmp_fp);

  non_ascii=0;
  for (i=0; i<nread; i++) {
    if (!(isprint(readbuf[i]) || readbuf[i]=='\r' || readbuf[i]=='\n' ||
        readbuf[i]=='\t')) {
      non_ascii = 1;
    }
  }
  fclose(tmp_fp);

  if (!non_ascii) {
    Type = ASCII;
    return (ASCII);
  } else {
    Type = UNKNOWN;
    return (UNKNOWN);
  }
}

/** Read Indirect File */
/** Return 1 if filename changes */
int KstFile::readIFile() {
  char tmpstr[255];
  FILE *fp;

  fp = fopen(IndirectFilename, "r");
  if (fp==NULL) {
    Type = EMPTY;
    Filename[0] = '\0';
    return (0);
  } else {
    fscanf(fp, "%255s", tmpstr);
    fclose(fp);
    if (strcmp(tmpstr,Filename)==0) {
      return(0);
    } else {
      strncpy(Filename, tmpstr, 254);
      return(1);
    }
  }
}


/** Initializes files */
void KstFile::init() {

  if ((Type == UNKNOWN) || (Type == EMPTY)) {
    determineType();
  }
  switch (Type) {
  case ASCII:
    asciiInitFile();
    break;
  case DIRFILE:
    dirfileInitFile();
    break;
  case EMPTY:
  case UNKNOWN:
    return;
  default:
    fprintf(stderr,"kst internal error: unknown type in inializer\n");
    exit(0);
  }

}

/************************************************************************/
/*                                                                      */
/*               Private Methods: ASCII Files                           */
/*                                                                      */
/************************************************************************/

/** Initializations for ascii files: */
bool KstFile::asciiInitFile(){

  RowIndex = (int *)malloc(32768 * sizeof(int));
  NumLinesAlloc = 32768;
  NumFrames=0;
  RowIndex[0] = 0;

  return(asciiUpdate());

}

/** Update an Ascii file: read lines and fill row index array
 Return 1 if there is new data.  Each read is MAXBUFREADLEN long
 and we repeat this at least READREPS times before returning.
 There may be performance problems (interactions with kstdoc's
 update) if the ascii file has extremely long lines or lots of
 comments.  If so, increase READREPS. This may decrease UI
 responsiveness while the file is being read however */
#define MAXBUFREADLEN 32768
#define READREPS 40
bool KstFile::asciiUpdate(){
  static char *tmpbuf=NULL;
  int i_buf;
  bool is_comment, has_dat;
  int bufstart, bufread;
  bool new_data = false;
  struct stat stat_buf;
  int fp;
  char *lfilename;
  int repeat=0;

  lfilename = Filename;

  if (stat(lfilename, &stat_buf)==0) { /* file exists */
    ByteLength=stat_buf.st_size;
  }  else {
    Type = EMPTY;
    return(false);
  }

  fp = open(lfilename, O_RDONLY);
  if (fp<0) {
    Type = EMPTY;
    printf("Unexpected error updating file %s; was the file just deleted?\n",
	   lfilename);
    return(false);
  }

  if (tmpbuf==NULL) {
    tmpbuf = (char*)malloc(MAXBUFREADLEN*sizeof(char));
  }

  do {
    /* Read the tmpbuffer, starting at row_index[NumFrames] */
    if (ByteLength-RowIndex[NumFrames]>MAXBUFREADLEN) bufread = MAXBUFREADLEN;
    else bufread = ByteLength-RowIndex[NumFrames];

    bufstart = RowIndex[NumFrames];
    lseek(fp, bufstart, SEEK_SET);
    read(fp, tmpbuf, bufread);

    is_comment = has_dat = false;
    for (i_buf=0; i_buf<bufread; i_buf++) {
      if (tmpbuf[i_buf] == '\n') {
        if (has_dat) {
          NumFrames++;
          if (NumFrames>=NumLinesAlloc) {
            RowIndex = (int *)realloc(RowIndex,
                                      (NumLinesAlloc + 32768)*sizeof(int));
            NumLinesAlloc += 32768;
          }
          new_data=true;
        }
        RowIndex[NumFrames] = bufstart + i_buf+1;
        has_dat = is_comment = false;
      } else if ((tmpbuf[i_buf] == '#') || (tmpbuf[i_buf] == '!') ||
                 (tmpbuf[i_buf] == '/') || (tmpbuf[i_buf] == ';') ||
                 (tmpbuf[i_buf] == 'c')) {
        is_comment = true;
      } else if (isdigit(tmpbuf[i_buf])) {
        if (!is_comment) {
          has_dat = true;
        }
      }
    }
  } while ((bufread==MAXBUFREADLEN) && (repeat++<READREPS));

  close(fp);
  return (new_data);
}

/** Read a field from an Ascii file */
int KstFile::asciiReadField(double *V, const char *field, const int &s,
                            const int &in_n){
  int i_row, i_char, i, n;
  char *tmpbuf;
  int bufread;
  int bufstart, col;
  bool done;
  bool incol;
  int i_col;
  int fp;
  char *lfilename;

  n = in_n;

  lfilename = Filename;

  if (n<0) n=1; /* n<0 means read one sample, not frame.... */
  if ((strcmp(field, "FILEFRAM") == 0) ||
      (strcmp(field, "INDEX") == 0)){
    for (i_row = s, i=0; i<n; i++) {
      V[i] = (double) i_row+i;
    }
    return(n);
  }

  /* make sure it is a valid col name */
  for (i=0; i<(int)strlen(field); i++) {
    if (!isdigit(field[i])) return(0);
  }
  col = atoi(field);

  if (col<1) return(0);

  bufstart = RowIndex[s];
  bufread = RowIndex[s+n] - RowIndex[s];
  fp = open(lfilename, O_RDONLY);
  if (fp<0) {
    Type = EMPTY;
    printf("Unexpected error opening file %s; assuming the file was deleted\n",
	   lfilename);
    return(0);
  }

  tmpbuf = (char *)malloc(bufread * sizeof(char));

  lseek(fp, bufstart, SEEK_SET);
  read(fp, tmpbuf, bufread);

  for (i_row = s, i=0; i<n; i++, i_row++) {
    done = false;
    incol=false;
    i_col=0;
    for (i_char = RowIndex[i_row]-bufstart; !done; i_char++) {
      if ((tmpbuf[i_char] == '#') || (tmpbuf[i_char] == '!') ||
	       (tmpbuf[i_char] == '/') || (tmpbuf[i_char] == ';') ||
	       (tmpbuf[i_char] == 'c')) {
	done=true;
	V[i]=0;
      } else if (tmpbuf[i_char]=='\n') {
	done=true;
	V[i]=0;
      } else if (isspace(tmpbuf[i_char])) {
	incol = false;
      } else {
	if (!incol) {
	  incol = true;
	  i_col++;
	  if (i_col == col) {
	    done = true;
	    V[i] = atof(tmpbuf + i_char);
	  }
	}
      }
    }
  }
  free(tmpbuf);
  close(fp);
  return (n);
}

bool KstFile::asciiIsValidField(const char *field) {
  return (1);
}

/************************************************************************/
/*                                                                      */
/*               Private Methods: Dirfile Files                           */
/*                                                                      */
/************************************************************************/

/** Initialization for Dirfile Files */
bool KstFile::dirfileInitFile(){

  NumFrames=0;

  return(dirfileUpdate());
}

/** Update Dirfile Data file: determine length return 1 if new data */
bool KstFile::dirfileUpdate(){
  int newNF;
  bool isnew;
  int error_code;

  newNF = GetNFrames(Filename, &error_code, NULL);


  if (newNF == NumFrames) isnew = false;
  else isnew = true;

  NumFrames = newNF;

  return(isnew);
}

/** Returns true if the field is valid, or false if it is not */
bool KstFile::dirfileIsValidField(const char *field) {
  int error_code;
  char tmpstr[57];
  int i;

  strncpy(tmpstr, field, 56);

  i = GetSamplesPerFrame(Filename, tmpstr, &error_code);

  if (error_code==0) return(true);
  else return(false);
}

/** determine samples per frame */
int KstFile::dirfileSampsPerFrame(const char *field) {
  int spf, error_code;
  char tmpstr[57];

  strncpy(tmpstr, field, 56);

  spf=GetSamplesPerFrame(Filename,tmpstr,&error_code);

  return(spf);
}


/** Read a field from a dirfile file.  The request is for a number of
frames:  the number of samples is n * SampsPerFrame(field).  if n<0,
read just one sample, not an entire frame.  The number
of samples read is returned */
int KstFile::dirfileReadField(double *V, const char *field,
                              const int &s, const int &n){
  int error_code;
  int n_read;
  char tmpstr[57];

  strncpy(tmpstr, field, 56);


  if (n<0) {
    n_read = GetData(Filename,tmpstr,
                       s,0, /* 1st sframe, 1st samp */
                       0,1, /* num sframes, num samps */
                       'd',(void*)V,
                       &error_code);
  } else {
    n_read = GetData(Filename,tmpstr,
                       s,0, /* 1st sframe, 1st samp */
                       n,0, /* num sframes, num samps */
                       'd',(void*)V,
                       &error_code);
  }

  return(n_read);

}

