#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "getdata.h"

char *GD_ERROR_CODES[15] = {"OK",
                            "Could not open Format file",
                            "Error in Format file",
                            "Could not open Data file",
                            "Field name too long",
                            "Field code not found in File Format",
                            "Unrecognized return type",
                            "Could not open field file",
			    " ",
			    " ",
                            " ",
                            "Could not open Field File",
                            "Size mismatch in linear combination",
                            "Could not open interpolation file ",
                            "Too many levels of recursion"
};

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
};

struct FormatType {
  char FileDirName[MAX_FILENAME_LENGTH];
  char lastRawField[FIELD_LENGTH+1];
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

static struct {
  int n;
  struct FormatType *F;
} Formats;

int recurse_level = 0;
int first_time = 1;

int DoField(struct FormatType *F, char *field_code,
	    int first_frame, int first_samp,
	    int num_frames, int num_samp, 
	    char return_type, void *data_out,
	    int *error_code);

/***************************************************************************/
/*                                                                         */
/*    GetLine: read non-comment line from format file                      */
/*        The line is placed in *line.                                     */
/*        Returns 1 if succesful, 0 if unsuccesful                         */
/*                                                                         */
/***************************************************************************/
int GetLine(FILE *fp, char *line) {
  char *ret_val;
  int first_char;

  do {
    ret_val=fgets(line, MAX_LINE_LENGTH, fp);
    first_char=0;
    while ((line[first_char]==' ') || (line[first_char]=='\t')) first_char++;
    line += first_char;
  } while (((line[0] =='#') || (strlen(line)<2)) && (ret_val!=NULL));

  if (ret_val!=NULL) {
    return (1); /* a line was read */
  } else {
    return(0);  /* there were no valid lines */
  }
}


/***************************************************************************/
/*                                                                         */
/*   FreeF: free any entries that have been allocated in F                 */
/*                                                                         */
/***************************************************************************/
void FreeF(struct FormatType *F) {
  if (F->n_raw>0) free(F->rawEntries);
  if (F->n_lincom > 0) free(F->lincomEntries);
  if (F->n_linterp >0) free(F->linterpEntries);
  if (F->n_mplex > 0) free(F->mplexEntries);
  if (F->n_bit > 0) free(F->bitEntries);
}

/***************************************************************************/
/*                                                                         */
/*   ParseRaw: parse a RAW data type in the formats file                   */
/*                                                                         */
/***************************************************************************/
void ParseRaw(char in_cols[15][MAX_LINE_LENGTH],
	      int n_cols, struct RawEntryType *R, int *error_code) {
  strcpy(R->field, in_cols[0]); // field
  R->fp = -1; // file not opened yet
  switch (in_cols[2][0]) {
  case 'c':
    R->size = 1;
    break;
  case 's': case 'u':
    R->size = 2;
    break;
  case 'S': case 'U': case 'f': case 'i':
    R->size = 4;
    break;
  case 'd':
    R->size = 8;
    break;
  default:
    *error_code = GD_E_FORMAT;
    return;
  }
  R->type = in_cols[2][0];
  R->samples_per_frame = atoi(in_cols[3]);
  if (R->samples_per_frame<=0) {
    *error_code = GD_E_FORMAT;
    return;
  }    
}

/***************************************************************************/
/*                                                                         */
/*  ParseLincom: parse a LINCOM data type in the formats file              */
/*                                                                         */
/***************************************************************************/
void ParseLincom(char in_cols[15][MAX_LINE_LENGTH],
	      int n_cols, struct LincomEntryType *L, int *error_code) {
  int i;
  strcpy(L->field, in_cols[0]); // field
  L->n_infields = atoi(in_cols[2]);
  if ((L->n_infields<1) || (L->n_infields>MAX_LINCOM) ||
      (n_cols < L->n_infields*3 + 3)) {
    *error_code = GD_E_FORMAT;
    return;
  }
  for (i=0; i<L->n_infields; i++) {
    strncpy(L->in_fields[i], in_cols[i*3+3], FIELD_LENGTH);
    L->m[i] = atof(in_cols[i*3+4]);
    L->b[i] = atof(in_cols[i*3+5]);
  } 
}
/***************************************************************************/
/*                                                                         */
/*  ParseLinterp: parse a LINTERP data type in the formats file            */
/*                                                                         */
/***************************************************************************/
void ParseLinterp(char in_cols[15][MAX_LINE_LENGTH],
		  int n_cols, struct LinterpEntryType *L,
		  int *error_code) {
  strcpy(L->field, in_cols[0]); // field
  strncpy(L->raw_field, in_cols[2], FIELD_LENGTH);
  strcpy(L->linterp_file, in_cols[3]);
  L->n_interp = -1; // linterp file not read yet
  
}

/***************************************************************************/
/*                                                                         */
/*   ParseMplex: parse MPLEX data type entry in formats file               */
/*                                                                         */
/***************************************************************************/
void ParseMplex(char in_cols[15][MAX_LINE_LENGTH],
		int n_cols, struct MplexEntryType *M,
		int *error_code) {

  if (n_cols<6) {
    *error_code = GD_E_FORMAT;
    return;
  }
  
  strcpy(M->field, in_cols[0]); // field
  strncpy(M->cnt_field, in_cols[2], FIELD_LENGTH);
  strncpy(M->data_field, in_cols[3], FIELD_LENGTH);
  M->i = atoi(in_cols[4]);
  M->max_i = atoi(in_cols[5]);
  if ((M->max_i<1) || (M->max_i < M->i)) {
    *error_code = GD_E_FORMAT;
    return;
  }
}

/***************************************************************************/
/*                                                                         */
/*   ParseBit: parse BIT data type entry in formats file                   */
/*                                                                         */
/***************************************************************************/
void ParseBit(char in_cols[15][MAX_LINE_LENGTH],
		int n_cols, struct BitEntryType *B,
		int *error_code) {
  strcpy(B->field, in_cols[0]); // field
  strncpy(B->raw_field, in_cols[2], FIELD_LENGTH); // field

  B->bitnum=atoi(in_cols[3]);
  if ((B->bitnum<0) || B->bitnum>31) {
    *error_code=GD_E_FORMAT;
    return;
  }
}

/***************************************************************************/
/*                                                                         */
/*   Compare functions for sorting the lists (using stdlib qsort)          */
/*                                                                         */
/***************************************************************************/
int RawCmp(const void *A, const void *B) {
  return (strcmp(((struct RawEntryType *)A)->field,
		 ((struct RawEntryType *)B)->field));
}

int LincomCmp(const void *A, const void *B) {
  return (strcmp(((struct LincomEntryType *)A)->field,
		 ((struct LincomEntryType *)B)->field));
}

int LinterpCmp(const void *A, const void *B) {
  return (strcmp(((struct LinterpEntryType *)A)->field,
		 ((struct LinterpEntryType *)B)->field));
}

int MplexCmp(const void *A, const void *B) {
  return (strcmp(((struct MplexEntryType *)A)->field,
		 ((struct MplexEntryType *)B)->field));
}

int BitCmp(const void *A, const void *B) {
  return (strcmp(((struct BitEntryType *)A)->field,
		 ((struct BitEntryType *)B)->field));
}


/***************************************************************************/
/*                                                                         */
/*   GetFormat: Read format file and fill structure.  The format           */
/*      is cached.                                                         */
/*                                                                         */
/***************************************************************************/
struct FormatType *GetFormat(char *filedir, int *error_code) {
  int i_format;
  FILE *fp;
  char format_file[MAX_FILENAME_LENGTH+6];
  char instring[MAX_LINE_LENGTH];
  struct FormatType *F;
  char in_cols[15][MAX_LINE_LENGTH];
  int n_cols;

  /** first check to see if we have already read it **/
  for (i_format=0; i_format<Formats.n; i_format++) {
    if (strncmp(filedir,
		Formats.F[i_format].FileDirName, MAX_FILENAME_LENGTH) == 0) {
      return(Formats.F + i_format);
    }
  }

  /** if we get here, the file has not yet been read */
  /** Allocate the memory, then fill.  If we have an error, */
  /*  we will have to free the memory... */
  Formats.n++;
  Formats.F = realloc(Formats.F, Formats.n * sizeof(struct FormatType));

  F = Formats.F+Formats.n-1;

  /***** open the format file (if there is one) ******/
  sprintf(format_file,"%s/format", filedir);
  fp = fopen(format_file, "r");
  if (fp ==NULL) {
    *error_code = GD_E_OPEN_FORMAT;
    Formats.n--; // no need to free.  The next realloc will just do nothing
    return (NULL);
  }

  strcpy(F->FileDirName, filedir);
  F->n_raw = F->n_lincom = F->n_linterp = F->n_mplex = F->n_bit = 0;
  F->rawEntries = NULL;
  F->lincomEntries = NULL;
  F->linterpEntries = NULL;
  F->mplexEntries = NULL;
  F->bitEntries = NULL;
  
  /***** start parsing ****/
  while (GetLine(fp, instring)) {
    // ok, brute force parse...  slow and ugly but convenient...
    n_cols = sscanf(instring, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s",
                    in_cols[0], in_cols[1], in_cols[2], in_cols[3], 
                    in_cols[4], in_cols[5], in_cols[6], in_cols[7], 
                    in_cols[8], in_cols[9], in_cols[10], in_cols[11], 
                    in_cols[12], in_cols[13], in_cols[14]);
    if (n_cols<4) {
      *error_code = GD_E_FORMAT;
      FreeF(F);
      return(NULL);
    }
    if (strlen(in_cols[0])>FIELD_LENGTH) {
      *error_code = GD_E_FIELD;
      FreeF(F);
      return(NULL);
    }
    
    if (strcmp(in_cols[1], "RAW")==0) {
      F->n_raw++;
      F->rawEntries =
	realloc(F->rawEntries, F->n_raw*sizeof(struct RawEntryType));
      ParseRaw(in_cols, n_cols, F->rawEntries+F->n_raw - 1, error_code);
    } else if (strcmp(in_cols[1], "LINCOM")==0) {
      F->n_lincom++;
      F->lincomEntries =
	realloc(F->lincomEntries,
		F->n_lincom*sizeof(struct LincomEntryType));
      ParseLincom(in_cols, n_cols, F->lincomEntries+F->n_lincom - 1,
		  error_code); 
    } else if (strcmp(in_cols[1], "LINTERP")==0) {
      F->n_linterp++;
      F->linterpEntries =
	realloc(F->linterpEntries,
		F->n_linterp*sizeof(struct LinterpEntryType));
      ParseLinterp(in_cols, n_cols, F->linterpEntries+F->n_linterp - 1,
		  error_code); 
    } else if (strcmp(in_cols[1], "MPLEX")==0) {
      F->n_mplex++;
      F->mplexEntries =
	realloc(F->mplexEntries,
		F->n_mplex*sizeof(struct MplexEntryType));
      ParseMplex(in_cols, n_cols, F->mplexEntries+F->n_mplex - 1,
		  error_code); 
    } else if (strcmp(in_cols[1], "BIT")==0) {
      F->n_bit++;
      F->bitEntries =
	realloc(F->bitEntries,
		F->n_bit*sizeof(struct BitEntryType));
      ParseBit(in_cols, n_cols, F->bitEntries+F->n_bit - 1,
		  error_code); 
    } else {
      FreeF(F);
      *error_code = GD_E_FORMAT;
      return(NULL);
    }
    if (*error_code!=GD_E_OK) {
      FreeF(F);
      return(NULL);
    }
  }
  /** copy last rawEntries.field to lastRawField */
  if (F->n_raw>0) {
    strncpy(F->lastRawField, F->rawEntries[F->n_raw-1].field,
	    FIELD_LENGTH);
  }
  
  /** Now sort the lists */
  if (F->n_raw > 1) {
    qsort(F->rawEntries, F->n_raw, sizeof(struct RawEntryType),
	  RawCmp);
  }
  
  if (F->n_lincom > 1) {
    qsort(F->lincomEntries, F->n_lincom, sizeof(struct LincomEntryType),
	  LincomCmp);
  }
  if (F->n_linterp > 1) {
    qsort(F->linterpEntries, F->n_linterp, sizeof(struct LinterpEntryType),
	  LinterpCmp);
  }
  if (F->n_mplex > 1) {
    qsort(F->mplexEntries, F->n_mplex, sizeof(struct MplexEntryType),
	  MplexCmp);
  }
  if (F->n_bit > 1) {
    qsort(F->bitEntries, F->n_bit, sizeof(struct BitEntryType),
	  BitCmp);
  }

  return(F);
}

/***************************************************************************/
/*                                                                         */
/*     File File Frame numbers into dataout                                */
/*                                                                         */
/***************************************************************************/
static void FillFileFrame(void *dataout, char rtype, int s0, int n) {
  int i;
  
  switch(rtype) {
  case 'c':
    for (i=0; i<n; i++) {
      ((char*)dataout)[i] = (char)i+s0;
    }
    break;
  case 'i': // for compatibility with creaddata. (depricated)
  case 'S':
        for (i=0; i<n; i++) {
      ((int*)dataout)[i] = (int)i+s0;
    }
    break;
  case 's':
        for (i=0; i<n; i++) {
      ((short*)dataout)[i] = (short)i+s0;
    }
    break;
  case 'U':
        for (i=0; i<n; i++) {
      ((unsigned int *)dataout)[i] = (unsigned int)i+s0;
    }
    break;
  case 'u':
        for (i=0; i<n; i++) {
      ((unsigned short *)dataout)[i] = (unsigned short)i+s0;
    }
    break;
  case 'f':
        for (i=0; i<n; i++) {
      ((float*)dataout)[i] = (float)i+s0;
    }
    break;
  case 'd':
        for (i=0; i<n; i++) {
      ((double*)dataout)[i] = (double)i+s0;
    }
    break;
  }
}
/***************************************************************************/
/*                                                                         */
/*    ConvertType: copy data to output buffer while converting type        */
/*           Returns error code                                            */
/*                                                                         */
/***************************************************************************/
static int ConvertType(unsigned char *data_in, char in_type,
                       void *data_out, char out_type, int n) {
  int i;

  if (out_type=='n') { /* null return type: don't return data */
    return(0);
  }

  switch (in_type) {
  case 'c':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=data_in[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=data_in[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=data_in[i];
      break;
    case 'i': case 'S':
      for (i=0;i<n;i++) ((int*)data_out)[i]=data_in[i];
      break;
    case 'U':
      for (i=0;i<n;i++)  ((unsigned int*)data_out)[i]=data_in[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=data_in[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=data_in[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
    }
    break;
  case 's':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=((short *)data_in)[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=((short *)data_in)[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=((short *)data_in)[i];
      break;
    case 'S': case 'i':
      for (i=0;i<n;i++) ((int*)data_out)[i]=((short *)data_in)[i];
      break;
    case 'U':
      for (i=0;i<n;i++) ((unsigned int*)data_out)[i]=((short *)data_in)[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=((short *)data_in)[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=((short *)data_in)[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
    }
    break;
  case 'u':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=
			  ((unsigned short *)data_in)[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=
			  ((unsigned short *)data_in)[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=
			  ((unsigned short *)data_in)[i];
      break;
    case 'i': case 'S':
      for (i=0;i<n;i++) ((int*)data_out)[i]=
			  ((unsigned short *)data_in)[i];
      break;
    case 'U':
      for (i=0;i<n;i++)  ((unsigned int*)data_out)[i]=
			   ((unsigned short *)data_in)[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=
			  ((unsigned short *)data_in)[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=
			  ((unsigned short *)data_in)[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
    }
    break;
  case 'i':
  case 'S':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=((int *)data_in)[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=((int *)data_in)[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=((int *)data_in)[i];
      break;
    case 'i': case 'S':
      for (i=0;i<n;i++) ((int*)data_out)[i]=((int *)data_in)[i];
      break;
    case 'U':
      for (i=0;i<n;i++)  ((unsigned int*)data_out)[i]=((int *)data_in)[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=((int *)data_in)[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=((int *)data_in)[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
    }
    break;
  case 'U':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=
			  ((unsigned *)data_in)[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=
			  ((unsigned *)data_in)[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=
			  ((unsigned *)data_in)[i];
      break;
    case 'i': case 'S':
      for (i=0;i<n;i++) ((int*)data_out)[i]=
			  ((unsigned *)data_in)[i];
      break;
    case 'U':
      for (i=0;i<n;i++)  ((unsigned int*)data_out)[i]=
			   ((unsigned *)data_in)[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=
			  ((unsigned *)data_in)[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=
			  ((unsigned *)data_in)[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
    }
    break;
  case 'f':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=((float *)data_in)[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=((float *)data_in)[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=((float *)data_in)[i];
      break;
    case 'i': case 'S':
      for (i=0;i<n;i++) ((int*)data_out)[i]=((float *)data_in)[i];
      break;
    case 'U':
      for (i=0;i<n;i++)  ((unsigned int*)data_out)[i]=((float *)data_in)[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=((float *)data_in)[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=((float *)data_in)[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
    }
    break;
  case 'd':
    switch (out_type) {
    case 'c':
      for (i=0;i<n;i++) ((unsigned char*) data_out)[i]=((double *)data_in)[i];
      break;
    case 's':
      for (i=0;i<n;i++) ((short*)data_out)[i]=((double *)data_in)[i];
      break;
    case 'u':
      for (i=0;i<n;i++) ((unsigned short*)data_out)[i]=((double *)data_in)[i];
      break;
    case 'i': case 'S':
      for (i=0;i<n;i++) ((int*)data_out)[i]=((double *)data_in)[i];
      break;
    case 'U':
      for (i=0;i<n;i++)  ((unsigned int*)data_out)[i]=((double *)data_in)[i];
      break;
    case 'f':
      for (i=0;i<n;i++) ((float*)data_out)[i]=((double *)data_in)[i];
      break;
    case 'd':
      for (i=0;i<n;i++) ((double*)data_out)[i]=((double *)data_in)[i];
      break;
    default:
      return (GD_E_BAD_RETURN_TYPE);
      break;
    }
    break;
  default:
    printf("internal getdata bug: unknown type shouldn't make it here!\n");
    return (GD_E_BAD_RETURN_TYPE);
  }
  
  return(GD_E_OK);
}

/***************************************************************************/
/*                                                                         */
/*   Get samples per frame for field, given FormatType *F                  */
/*                                                                         */
/***************************************************************************/
int GetSPF(char *field_code, struct FormatType *F, int *error_code) {
  struct RawEntryType tR;
  struct RawEntryType *R;
  struct LincomEntryType tL;
  struct LincomEntryType *L;
  struct BitEntryType tB;
  struct BitEntryType *B;
  struct LinterpEntryType tI;
  struct LinterpEntryType *I;
  int spf;
  
  if (recurse_level>10) {
    *error_code = GD_E_RECURSE_LEVEL;
    return(0);
  }
  
  if ((strcmp(field_code,"FILEFRAM")==0) ||
      (strcmp(field_code,"INDEX")==0)) {
    return(1);
  }

  /***************************************/
  /** Check to see if it is a raw entry **/
  /* binary search for the field */
  /* make a RawEntry we can compare to */
  strncpy(tR.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  R = bsearch(&tR, F->rawEntries, F->n_raw,
	      sizeof(struct RawEntryType), RawCmp);
  if (R!=NULL) {
    spf = R->samples_per_frame;
    return(spf);
  }

  /***************************************/
  /** Check to see if it is a lincom entry **/
  /* binary search for the field */
  /* make a RawEntry we can compare to */
  strncpy(tL.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  L = bsearch(&tL, F->lincomEntries, F->n_lincom,
	      sizeof(struct LincomEntryType), LincomCmp);
  if (L!=NULL) {
    recurse_level++;
    spf = GetSPF(L->in_fields[0], F, error_code);
    recurse_level--;
    return(spf);
  }

  /***************************************/
  /** Check to see if it is a bit entry **/
  /* binary search for the field */
  /* make a BitEntry we can compare to */
  strncpy(tB.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  B = bsearch(&tB, F->bitEntries, F->n_bit,
	      sizeof(struct BitEntryType), BitCmp);
  if (B!=NULL) {
    recurse_level++;
    spf = GetSPF(B->raw_field, F, error_code);
    recurse_level--;
    return(spf);
  }

  /***************************************/
  /** Check to see if it is a linterp entry **/
  /* binary search for the field */
  /* make a LinterpEntry we can compare to */
  strncpy(tI.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  I = bsearch(&tI, F->linterpEntries, F->n_linterp,
	      sizeof(struct LinterpEntryType), LinterpCmp);
  if (I!=NULL) {
    recurse_level++;
    spf = GetSPF(I->raw_field, F, error_code);
    recurse_level--;
    return(spf);
  }

  *error_code = GD_E_BAD_CODE;
  return(0);
}

/***************************************************************************/
/*                                                                         */
/*   Look to see if the field code belongs to a raw.  If so, parse it.     */
/*                                                                         */
/***************************************************************************/
int DoIfRaw(struct FormatType *F, char *field_code,
	    int first_frame, int first_samp,
	    int num_frames, int num_samp, 
	    char return_type, void *data_out,
	    int *error_code, int *n_read) {

  struct RawEntryType tR;
  struct RawEntryType *R;
  int s0, ns, bytes_read;
  char datafilename[MAX_FILENAME_LENGTH+FIELD_LENGTH + 1];
  unsigned char *databuffer;

  /******* binary search for the field *******/
  /* make a RawEntry we can compare to */
  strncpy(tR.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  R = bsearch(&tR, F->rawEntries, F->n_raw,
	      sizeof(struct RawEntryType), RawCmp);
  if (R==NULL) return(0);

  /** if we got here, we found the field! **/
  s0 = first_samp + first_frame*R->samples_per_frame;
  ns = num_samp + num_frames*R->samples_per_frame;

  /** open the file (and cache the fp) if it hasn't been opened yet. */
  if (R->fp <0) {
    sprintf(datafilename, "%s/%s", F->FileDirName, field_code);
    R->fp = open(datafilename, O_RDONLY);
    if (R->fp<0) {
      *n_read = 0;
      *error_code = GD_E_OPEN_RAWFIELD;
      return(1);
    }
  }
  
  databuffer = (unsigned char *)malloc(ns*R->size);

  lseek(R->fp, s0*R->size, SEEK_SET);
  bytes_read = read(R->fp, databuffer, ns*R->size);
  *n_read = bytes_read/R->size;

  *error_code =
    ConvertType(databuffer, R->type, data_out, return_type, *n_read);
  
  return(1);
}


/***************************************************************************/
/*                                                                         */
/*            AllocTmpbuff: allocate a buffer of the right type and size   */
/*                                                                         */
/***************************************************************************/
static void *AllocTmpbuff(char type, int n) {
  void *buff=NULL;
  switch(type) {
  case 'n':
    buff = NULL;
    break;
  case 'c':
    buff = malloc(n*sizeof(char));
    break;
  case 'i':
  case 'S':
  case 'U':
    buff = malloc(n*sizeof(int));
    break;
  case 's':
  case 'u':
    buff = malloc(n*sizeof(short));
    break;
  case 'f':
    buff = malloc(n*sizeof(float));
    break;
  case 'd':
    buff = malloc(n*sizeof(double));
    break;
  default:
    printf("Unexpected bad type error in AllocTmpbuff (%c)\n",type);
    exit(0);
    break;
  }
  if ((type != 'n') && (buff==NULL)) {
    printf("Memory Allocation error in AllocTmpbuff\n");
    exit(0);
  }
  return(buff);
}

/***************************************************************************/
/*                                                                         */
/*   ScaleData: out = m*in+b                                               */
/*                                                                         */
/***************************************************************************/
static void ScaleData(void *data, char type, int npts, double m, double b) {
  char *data_c;
  short *data_s;
  unsigned short *data_u;
  unsigned *data_U;
  int *data_i;
  float *data_f;
  double *data_d;  

  int i;

  switch(type) {
  case 'n':
    break;
  case 'c':
    data_c = (char *)data;
    for (i=0; i<npts; i++) {
      data_c[i] =(char)((double)data_c[i] * m + b);
    }
    break;
  case 's':
    data_s = (short *)data;
    for (i=0; i<npts; i++) {
      data_s[i] =  (short)((double)data_s[i] * m + b);
    }
    break;
  case 'u':
    data_u = (unsigned short *)data;
    for (i=0; i<npts; i++) {
      data_u[i] = (unsigned short)((double)data_u[i] * m + b);
    }
    break;
  case 'S': case 'i':
    data_i = (int *)data;
    for (i=0; i<npts; i++) {
      data_i[i] = (int)((double)data_i[i] * m + b);
    }
    break;
  case 'U':
    data_U = (unsigned*)data;
    for (i=0; i<npts; i++) {
      data_U[i] = (unsigned)((double)data_U[i] * m + b);
    }
    break;
  case 'f':
    data_f = (float *)data;
    for (i=0; i<npts; i++) {
      data_f[i] = (float)((double)data_f[i] * m + b);
    }
    break;
  case 'd':
    data_d = (double *)data;
    for (i=0; i<npts; i++) {
      data_d[i] = data_d[i] * m + b;
    }
    break;
  default:
    printf("Another impossible error\n"); exit(0);
    break;
  } 
}

/***************************************************************************/
/*                                                                         */
/*            AddData: add B to A.  A is unchanged                         */
/*                                                                         */
/***************************************************************************/
static void AddData(void *A, void *B, char type, int n) {
  int i;
  
  switch(type) {
  case 'n': /* null read */
    break;
  case 'c':
    for (i=0; i<n; i++) {
      ((char*)A)[i]+=((char*)B)[i];
    }
    break;
  case 'S': case 'i':
    for (i=0; i<n; i++) {
      ((int*)A)[i]+=((int*)B)[i];
    }
    break;
  case 's':
    for (i=0; i<n; i++) {
      ((short*)A)[i]+=((short*)B)[i];
    }
    break;
  case 'u':
    for (i=0; i<n; i++) {
      ((unsigned short*)A)[i]+=((unsigned short*)B)[i];
    }
    break;
  case 'U':
    for (i=0; i<n; i++) {
      ((unsigned*)A)[i]+=((unsigned*)B)[i];
    }
    break;
  case 'f':
    for (i=0; i<n; i++) {
      ((float*)A)[i]+=((float*)B)[i];
    }
    break;
  case 'd':
    for (i=0; i<n; i++) {
      ((double*)A)[i]+=((double*)B)[i];
    }
    break;
  default:
    printf("Unexpected bad type error in AddData\n");
    exit(0);
    break;
  }
}
    

/***************************************************************************/
/*                                                                         */
/*   Look to see if the field code belongs to a lincom.  If so, parse it.  */
/*                                                                         */
/***************************************************************************/
int DoIfLincom(struct FormatType *F, char *field_code,
	    int first_frame, int first_samp,
	    int num_frames, int num_samp, 
	       char return_type, void *data_out,
	    int *error_code, int *n_read) {
  struct LincomEntryType tL;
  struct LincomEntryType *L;
  void *tmpbuf;
  int i;

  /******* binary search for the field *******/
  /* make a LincomEntry we can compare to */
  strncpy(tL.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  L = bsearch(&tL, F->lincomEntries, F->n_lincom,
	      sizeof(struct LincomEntryType), LincomCmp);
  if (L==NULL) return(0);

  /*****************************************/
  /** if we got here, we found the field! **/
  /** read into dataout and scale the first element **/
  recurse_level++;
  *n_read = DoField(F, L->in_fields[0],
		    first_frame, first_samp,
		    num_frames, num_samp,
		    return_type, data_out,
		    error_code);
  
  recurse_level--;
  if (*error_code != GD_E_OK) return(1);
  ScaleData(data_out, return_type, *n_read, L->m[0], L->b[0]);

  if (L->n_infields > 1) {
    tmpbuf = AllocTmpbuff(return_type, *n_read);
    for (i=0; i<L->n_infields; i++) {
      recurse_level++;
      if (DoField(F, L->in_fields[i],
		  first_frame, first_samp,
		  num_frames, num_samp,
		  return_type, tmpbuf,
		  error_code) != *n_read) {
	free(tmpbuf);
	*error_code = GD_E_SIZE_MISMATCH;
	*n_read = 0;
	recurse_level--;
	return(1);
      }
      recurse_level--;
      ScaleData(tmpbuf, return_type, *n_read, L->m[i], L->b[i]);
      AddData(data_out, tmpbuf, return_type, *n_read);      
    }
    free(tmpbuf);
  }
  
  return(1);
}

/***************************************************************************/
/*                                                                         */
/*   Look to see if the field code belongs to a bitfield.  If so, parse it.*/
/*                                                                         */
/***************************************************************************/
int DoIfBit(struct FormatType *F, char *field_code,
	    int first_frame, int first_samp,
	    int num_frames, int num_samp, 
	    char return_type, void *data_out,
	    int *error_code, int *n_read) {
  struct BitEntryType tB;
  struct BitEntryType *B;
  unsigned *tmpbuf;
  int i;
  int spf;
  int ns;

  /******* binary search for the field *******/
  /* make a BitEntry we can compare to */
  strncpy(tB.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  B = bsearch(&tB, F->bitEntries, F->n_bit,
	      sizeof(struct BitEntryType), BitCmp);
  if (B==NULL) return(0);

  /*****************************************/
  /** if we got here, we found the field! **/
  recurse_level++;
  spf = GetSPF(B->raw_field, F, error_code);
  recurse_level--;
  if (*error_code!=GD_E_OK) return(1);

  ns = num_samp + num_frames*spf;
  tmpbuf = (unsigned *)malloc(ns*sizeof(unsigned));

  recurse_level++;
  *n_read = DoField(F, B->raw_field,
		    first_frame, first_samp,
		    num_frames, num_samp,
		    'U', tmpbuf,
		    error_code);
  recurse_level--;
  if (*error_code!=GD_E_OK) {
    free(tmpbuf);
    return(1);
  }

  for (i=0; i<*n_read; i++) {
    tmpbuf[i] = (tmpbuf[i]>>B->bitnum) & 0x0001;
  }

  *error_code = ConvertType((unsigned char *)tmpbuf, 'U',
			    data_out, return_type, *n_read);

  return(1);
}

/***************************************************************************/
/*                                                                         */
/*   ReadLinterpFile: Read in the linterp data for this field              */
/*                                                                         */
/***************************************************************************/
static int ReadLinterpFile(struct LinterpEntryType *E) {
  FILE *fp;
  int i;
  char line[255];

  fp = fopen(E->linterp_file, "r");
  if (fp==NULL) {
    return (GD_E_OPEN_LINFILE);
  }
  
  /* first read the file to see how big it is */
  i=0;
  while (GetLine(fp, line)) {
    i++;
  }
  if (i<2) {
    return (GD_E_OPEN_LINFILE);
  }
  E->n_interp = i;
  E->x = (double *)malloc(i*sizeof(double));
  E->y = (double *)malloc(i*sizeof(double));
  /* now read in the data */
  rewind(fp);
  for (i=0; i<E->n_interp; i++) {
    GetLine(fp, line);
    sscanf(line, "%lg %lg",&(E->x[i]), &(E->y[i]));
  }
  return (GD_E_OK);
}

/***************************************************************************/
/*                                                                         */
/*   GetIndex: just linearly search - we are probably right to start with  */
/*                                                                         */
/***************************************************************************/
static int GetIndex(double x, double lx[], int index, int n) {
  /* increment until we are bigger */
  while ((index<n-2 ) && (x>lx[index])) {
    index++;
  }
  /* decrement until we are smaller */
  while ((index>0) && (x<lx[index])) {
    index--;
  }

  return(index);
}

/***************************************************************************/
/*                                                                         */
/*   LinterpData: calibrate data using lookup table lx and ly              */
/*                                                                         */
/***************************************************************************/
static void LinterpData(void *data, char type, int npts,
			double *lx, double *ly, int n_ln) {
  int i, index=0;
  double x;

  for (i=0; i<npts; i++) {
    switch(type) {
    case 'n':
      return;
      break;
    case 'c':
      x = ((char *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((char *)data)[i] = (char)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    case 's':
      x = ((short *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((short *)data)[i] = (short)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    case 'u':
      x = ((unsigned short *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((unsigned short *)data)[i] =
        (unsigned short)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    case 'S': case 'i':
      x = ((int *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((int *)data)[i] = (int)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    case 'U':
      x = ((unsigned *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((unsigned *)data)[i] =
        (unsigned)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    case 'f':
      x = ((float *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((float *)data)[i] = (float)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    case 'd':
      x = ((double *)data)[i];
      index = GetIndex(x, lx, index,n_ln);
      ((double *)data)[i] = (double)(ly[index] + (ly[index+1]-ly[index])/
        (lx[index+1]-lx[index]) * (x-lx[index]));
      break;
    default:
      printf("Another impossible error\n"); exit(0);
      break;
    } 
  }
}

/***************************************************************************/
/*                                                                         */
/*   Look to see if the field code belongs to a bitfield.  If so, parse it.*/
/*                                                                         */
/***************************************************************************/
int DoIfLinterp(struct FormatType *F, char *field_code,
		int first_frame, int first_samp,
		int num_frames, int num_samp, 
		char return_type, void *data_out,
		int *error_code, int *n_read) {
  struct LinterpEntryType tI;
  struct LinterpEntryType *I;

  /******* binary search for the field *******/
  /* make a LinterpEntry we can compare to */
  strncpy(tI.field, field_code, FIELD_LENGTH); 
  /** use the stdlib binary search */
  I = bsearch(&tI, F->linterpEntries, F->n_linterp,
	      sizeof(struct LinterpEntryType), LinterpCmp);
  if (I==NULL) return(0);

  /*****************************************/
  /** if we got here, we found the field! **/
  if (I->n_interp<0) {
    *error_code = ReadLinterpFile(I);
    if (*error_code != GD_E_OK) return(1);
  }
  recurse_level++;
  *n_read = DoField(F, I->raw_field,
		    first_frame, first_samp,
		    num_frames, num_samp,
		    return_type, data_out,
		    error_code);
  recurse_level--;
  if (*error_code!=GD_E_OK) return(1);
  LinterpData(data_out, return_type, *n_read, I->x, I->y, I->n_interp);
  return(1);
  
}

/***************************************************************************/
/*                                                                         */
/*  DoField: Doing one field once F has been identified                    */
/*                                                                         */
/***************************************************************************/
int DoField(struct FormatType *F, char *field_code,
	    int first_frame, int first_samp,
	    int num_frames, int num_samp, 
	    char return_type, void *data_out,
	    int *error_code) {
  int n_read;

  if (recurse_level>10) {
    *error_code = GD_E_RECURSE_LEVEL;
    return(0);
  }
  

  /********************************************/
  /* if Asking for "FILEFRAM" or "INDEX", just return it */
  if ((strcmp(field_code,"FILEFRAM")==0) ||
      (strcmp(field_code,"INDEX")==0)) {
    n_read = num_frames + num_samp;
    if (data_out!=NULL) {
      FillFileFrame(data_out, return_type, first_frame+first_samp, n_read);
    }
    *error_code=GD_E_OK;
    return(n_read);
  }

  if (DoIfRaw(F, field_code,
	      first_frame, first_samp,
	      num_frames, num_samp,
	      return_type, data_out,
	      error_code, &n_read)) {
    return(n_read);
  } else if (DoIfLincom(F, field_code,
		       first_frame, first_samp,
		       num_frames, num_samp,
		       return_type, data_out,
		       error_code, &n_read)) {
    return(n_read);
  } else if (DoIfBit(F, field_code,
		     first_frame, first_samp,
		     num_frames, num_samp,
		     return_type, data_out,
		     error_code, &n_read)) {
    return(n_read);
  } else {
    *error_code = GD_E_BAD_CODE;
    return(0);
  }
}

/***************************************************************************/
/*                                                                         */
/*  GetData: read BLAST format files.                                      */
/*    filename_in: the name of the file directory (raw files are in here)  */
/*    field_code: the name of the field you want to read                   */
/*    first_frame, first_samp: the first sample read is                    */
/*              first_samp + samples_per_frame*first_frame                 */
/*    num_frames, num_samps: the number of samples read is                 */
/*              num_samps + samples_per_frame*num_frames                   */
/*    return_type: data type of *data_out.  's': 16 bit signed             */
/*              'u' 16bit unsiged. 'S' 32bit signed 'U' 32bit unsigned     */
/*              'c' 8 bit signed                                           */
/*    void *data_out: array to put the data                                */
/*    *error_code: error code is returned here. If error_code==null,       */
/*               GetData prints the error message and exits                */
/*                                                                         */
/*    return value: returns number of samples actually read into data_out  */
/*                                                                         */
/***************************************************************************/
int GetData(char *filename_in, char *field_code,
	    int first_frame, int first_samp,
	    int num_frames, int num_samp,
	    char return_type, void *data_out,
	    int *error_code) {

  struct FormatType *F;
  int n_read=0;
  char filename[MAX_FILENAME_LENGTH+1];

  *error_code = GD_E_OK;
  
  if (first_time) {
    Formats.n = 0;
    Formats.F = NULL;
    first_time = 0;
  }

  strncpy(filename, filename_in, MAX_FILENAME_LENGTH);
  if (filename[strlen(filename)-1]=='/') filename[strlen(filename)-1]='\0';
  F = GetFormat(filename, error_code);
  if (*error_code!=GD_E_OK) {
    return(0);
  }

  n_read = DoField(F, field_code,
		   first_frame, first_samp,
		   num_frames, num_samp,
		   return_type, data_out,
		   error_code);
  
  return(n_read);
}

/***************************************************************************/
/*                                                                         */
/*    Get the number of frames availible                                   */
/*                                                                         */
/***************************************************************************/
int GetNFrames(char *filename_in, int *error_code) {
  struct FormatType *F;
  char filename[MAX_FILENAME_LENGTH+1];
  char raw_data_filename[MAX_FILENAME_LENGTH+FIELD_LENGTH+2];
  struct stat statbuf;
  int nf;

  *error_code = GD_E_OK;
  
  if (first_time) {
    Formats.n = 0;
    Formats.F = NULL;
    first_time = 0;
  }

  strncpy(filename, filename_in, MAX_FILENAME_LENGTH);
  if (filename[strlen(filename)-1]=='/') filename[strlen(filename)-1]='\0';
  F = GetFormat(filename, error_code);
  if (*error_code!=GD_E_OK) {
    return(0);
  }

  if (F->n_raw==0) {
    *error_code = GD_E_FORMAT;
    return(0);
  }

  sprintf(raw_data_filename,"%s/%s", filename, F->lastRawField);
  stat(raw_data_filename, &statbuf);

  nf = statbuf.st_size/
	 (F->rawEntries[0].size*F->rawEntries[0].samples_per_frame);

  nf-=2;
  if (nf<0) nf = 0;
  return(nf);
}

/***************************************************************************/
/*                                                                         */
/*    Get the number of samples for each frame for the given field         */
/*                                                                         */
/***************************************************************************/
int GetSamplesPerFrame(char *filename_in, char *field_name, int *error_code) {
  struct FormatType *F;
  char filename[MAX_FILENAME_LENGTH+1];

  *error_code = GD_E_OK;
  
  if (first_time) {
    Formats.n = 0;
    Formats.F = NULL;
    first_time = 0;
  }

  strncpy(filename, filename_in, MAX_FILENAME_LENGTH);
  if (filename[strlen(filename)-1]=='/') filename[strlen(filename)-1]='\0';
  F = GetFormat(filename, error_code);
  if (*error_code!=GD_E_OK) {
    return(0);
  }

  if (F->n_raw==0) {
    *error_code = GD_E_FORMAT;
    return(0);
  }

  return (GetSPF(field_name, F, error_code));
}
