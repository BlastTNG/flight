#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 
#include <time.h>

#include "tx_struct.h"

char filedirname[200];

#define MAXBUF 1200

struct FieldType {
  short i0;   // start of field in rxframe, words
  short size; // size in words
  int i_in;   // index in elements
  int i_out;  // index in elements
  int fp;
  void *b;    // buffer
};

struct FieldType normal_fast[N_FASTCHLIST];
struct FieldType slow_fields[N_SLOW][FAST_PER_SLOW];
struct FieldType bolo_fields[DAS_CARDS][DAS_CHS];

unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

int n_fast, bolo_i0;

char *StringToLower(char *s);
char *StringToUpper(char *s);

extern unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

/*********************************************************************/
/*                                                                   */
/*     Initialize dirfile                                            */
/*                                                                   */
/*********************************************************************/
void InitializeDirfile(char type) {
  time_t t;
  FILE *fp;
  int i_slow, i_ch, i;
  int i_card;
  char filename[200];
  char field[20];

  /* Make the file name */
  t = time(NULL);
  
  sprintf(filedirname,"/data/rawdir/%u.%c", (unsigned)t, type);
  mkdir(filedirname, 00755);

  fprintf(stderr, "Writing to dirfile %s\n", filedirname);
  
  /***********************************/
  /* create and fill the format file */
  sprintf(filename, "%s/format", filedirname);
  fp = fopen(filename, "w");
  fprintf(fp, "FASTSAMP         RAW    U 20\n");
  n_fast = 0;
  sprintf(filename, "%s/FASTSAMP", filedirname);
  normal_fast[n_fast].fp = creat(filename, 00644);
  normal_fast[n_fast].i0 = 1;
  normal_fast[n_fast].size = 2; 
  normal_fast[n_fast].i_in = normal_fast[n_fast].i_out = 0;
  normal_fast[n_fast].b = malloc(MAXBUF*sizeof(int));
  n_fast++;

  /* slow chs */
  fprintf(fp, "\n## SLOW CHANNELS:\n");
  for (i_slow = 0; i_slow<N_SLOW; i_slow++) {
    for (i_ch = 0; i_ch<FAST_PER_SLOW; i_ch++) {
      if (strlen(SlowChList[i_slow][i_ch].field) > 0) {
	fprintf(fp, "%-16s RAW    %c 1\n",
		StringToLower(SlowChList[i_slow][i_ch].field),
		SlowChList[i_slow][i_ch].type);
	fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
		StringToUpper(SlowChList[i_slow][i_ch].field),
		StringToLower(SlowChList[i_slow][i_ch].field),
		SlowChList[i_slow][i_ch].m_c2e,
		SlowChList[i_slow][i_ch].b_e2e);
	fflush(fp);
	sprintf(filename, "%s/%s", filedirname,
		SlowChList[i_slow][i_ch].field);
	slow_fields[i_slow][i_ch].fp = creat(filename, 00644);
      } else {
	slow_fields[i_slow][i_ch].fp = -1;
      }
      slow_fields[i_slow][i_ch].i_in = slow_fields[i_slow][i_ch].i_out = 0;
      slow_fields[i_slow][i_ch].b = malloc ( 2 * MAXBUF);
      slow_fields[i_slow][i_ch].i0 = 4 + i_slow;
    }
  }

  /* normal fast chs */
  fprintf(fp, "\n## FAST CHANNELS:\n");
  fflush(fp);
  for (i = 0; i<N_FASTCHLIST; i++) {
    if (strcmp(FastChList[i].field, "n5c0lo") == 0) {
      bolo_i0 = i + FAST_OFFSET;
      break;
    } else if (strlen(FastChList[i].field) > 0) {
      switch (FastChList[i].type) {
      case 's': case 'u':
        normal_fast[n_fast].size = 1;
        break;
      case 'S': case 'U': case 'I': case 'f':
        normal_fast[n_fast].size = 2;
        break;
      default:
        printf("error: bad type in initdirfile: %s %c %d\n",
            FastChList[i].field, FastChList[i].type, i);
        exit(0);
      }
      sprintf(filename, "%s/%s", filedirname,FastChList[i].field);
      normal_fast[n_fast].fp = creat(filename, 00644);
      normal_fast[n_fast].i0 = i + FAST_OFFSET;
      normal_fast[n_fast].i_in = normal_fast[n_fast].i_out = 0;
      normal_fast[n_fast].b = malloc(MAXBUF * 2 * normal_fast[n_fast].size);
      n_fast++;
      fprintf(fp, "%-16s RAW    %c %d\n",
          StringToLower(FastChList[i].field),
          FastChList[i].type, FAST_PER_SLOW);
      fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
          StringToUpper(FastChList[i].field),
          StringToLower(FastChList[i].field),
          FastChList[i].m_c2e, FastChList[i].b_e2e);
      fflush(fp);
    }
  }

  /* special (bolo) fast chs */
  fprintf(fp, "\n## BOLOMETERS:\n");
  fflush(fp);
  for (i_card = 0; i_card < DAS_CARDS; i_card++) {
    for (i_ch = 0; i_ch < DAS_CHS; i_ch++) {
      sprintf(field, "n%dc%d", i_card + 5, i_ch);
      sprintf(filename, "%s/%s", filedirname,field);
      bolo_fields[i_card][i_ch].fp = creat(filename, 00644);
      bolo_fields[i_card][i_ch].size = 2;
      bolo_fields[i_card][i_ch].i_in = bolo_fields[i_card][i_ch].i_out = 0;
      bolo_fields[i_card][i_ch].b = malloc(MAXBUF * 4);
      bolo_fields[i_card][i_ch].i0 = bolo_i0 + i_card * (DAS_CARDS * 3 / 2)
                + i_ch;
      fprintf(fp, "%-16s RAW    U %d\n",
          StringToLower(field), FAST_PER_SLOW);
      fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
          StringToUpper(field), StringToLower(field),
          LOCKIN_C2V, LOCKIN_OFFSET);
    }
  }

  /* derived channels */
  fprintf(fp,
      "### Bias Generator Bitfield ###\n"
      "BIAS_IS_DC       BIT biasin 1\n"
      "BIAS_CLK_IS_INT  BIT biasin 2\n"
      "BIAS_IS_INT      BIT biasin 3\n"
      "### Cryo State Bitfield ###\n"
      "HE_LEV_SENS      BIT cryostate 0\n"
      "CHARC_HEATER     BIT cryostate 1\n"
      "COLDP_HEATER     BIT cryostate 2\n"
      "CALIBRATOR       BIT cryostate 3\n"
      "LN_VALVE         BIT cryostate 4\n"
      "LN_DIREC         BIT cryostate 5\n"
      "LHE_VALVE        BIT cryostate 6\n"
      "LHE_DIREC        BIT cryostate 7\n"
      "### Cryo Table Lookups ###\n"
      "# Diodes\n"
      "T_vcs1	LINTERP	T_VCS1	/data/etc/dt600.txt\n"
      "T_vcs_fet   LINTERP T_VCS_FET	/data/etc/dt600.txt\n"
      "T_he4   LINTERP T_HE4    /data/etc/dt600.txt\n"
      "T_ln2   LINTERP T_LN2    /data/etc/dt600.txt\n"
      "T_jfet   LINTERP T_JFET    /data/etc/dt600.txt\n"
      "T_mirror3   LINTERP T_MIRROR3    /data/etc/dt600.txt\n"
      "T_mirror5   LINTERP T_MIRROR5    /data/etc/dt600.txt\n"
      "T_lyot_stim   LINTERP T_LYOT_STIM    /data/etc/dt600.txt\n"
      "T_heatswitch   LINTERP T_HEATSWITCH    /data/etc/dt600.txt\n"
      "T_charcoal   LINTERP T_CHARCOAL    /data/etc/dt600.txt\n"
      "T_he4pot   LINTERP T_HE4POT    /data/etc/dt600.txt\n"
      "T_mirror4   LINTERP T_MIRROR4    /data/etc/dt600.txt\n"
      "T_ln_filt   LINTERP T_LN_FILT    /data/etc/dt600.txt\n"
      "T_vcs_filt   LINTERP T_VCS_FILT    /data/etc/dt600.txt\n"
      "T_he_filt   LINTERP T_HE_FILT    /data/etc/dt600.txt\n"
      "# GRTs (ROX)\n"
      "#T_he3	LINTERP	T_HE3	/data/etc/rox102a.txt\n"
      "#T_bda1    LINTERP T_BDA1   /data/etc/rox102a.txt\n"
      "#T_bda2   LINTERP T_BDAA   /data/etc/rox102a.txt\n"
      "#T_bda3    LINTERP T_BDA3   /data/etc/rox102a.txt\n"
      "# Level Sensor\n"
      "he4_litre    LINTERP HE4_LEVEL   /data/etc/he4_level.txt\n"
      "# Control Bits\n"
      "# To Be Filled In At A Later Date\n"
      "# TEMPORARY GRT Calibration\n"
      "Res_He3	       LINCOM  1       N10C0   1.9416E04       -8.4574\n"
      "Res_horn_500   LINCOM  1       N10C1   1.9416E04       -8.4574\n"
      "Res_pot        LINCOM  1       N10C2   1.9416E04       -8.4574\n"
      "Res_base_500   LINCOM  1       N10C4   1.9416E04       -8.4574\n"
      "Res_ring_250   LINCOM  1       N10C7   1.9416E04       -8.4574\n"
      "T_He3	   LINTERP N10C0        /data/etc/rox102a.txt\n"
      "T_horn_500  LINTERP N10C1       /data/etc/rox102a.txt\n"
      "T_pot       LINTERP N10C2       /data/etc/rox102a.txt\n"
      "T_base_500  LINTERP N10C4       /data/etc/rox102a.txt \n"
      "T_ring_250  LINTERP N10C7       /data/etc/rox102a.txt\n"
      "#\n"
      "# Nice CPU Values\n"
      "CPU_SEC LINCOM  1       cpu_time        1       -%i\n"
      "CPU_MIN LINCOM  1       CPU_SEC 0.016666666 0\n"
      "CPU_HOUR LINCOM 1       CPU_SEC 0.000277777 0\n"
      "CPU_DAY LINCOM  1       CPU_SEC 1.15741E-5  0\n"
      "CPU_WEEK LINCOM 1       CPU_SEC 1.65344E-6  0\n"
      "CPU_MONTH LINCOM 1      CPU_SEC 3.85803E-7  0\n"
      "CPU_YEAR LINCOM 1       CPU_SEC 3.17099E-8  0\n", (unsigned)t
      );
  fclose(fp);
  
  fp = fopen("/data/etc/datafile.cur","w");
  if (fp == NULL) {
    printf("Error opening curfile.\n");
    exit(0);
  }

  fprintf(fp,filedirname);
  
  fclose(fp);

  
  
}

// called from main thread: puts rxframe into individual buffers
void pushDiskFrame(unsigned short *Rxframe) {
  unsigned int i_slow, i_in;
  int i_card, i_ch;
  static int i_count = 0;
  unsigned B0, B1;
  static int n_frames = 0;

  /*************/
  /* slow data */
  i_ch = Rxframe[3];
  if (i_ch < FAST_PER_SLOW) {
    for (i_slow = 0; i_slow<N_SLOW; i_slow++) {
      slow_data[i_slow][i_ch] = Rxframe[slow_fields[i_slow][i_ch].i0];
    }
  }

  /** push if FAST_PER_SLOW fasts are done... */
  i_count++;
  if (i_count>=FAST_PER_SLOW) {
    i_count = 0;
    for (i_slow = 0; i_slow<N_SLOW; i_slow++) {
      for (i_ch = 0; i_ch<FAST_PER_SLOW; i_ch++) {
        i_in = slow_fields[i_slow][i_ch].i_in;
        ((unsigned short *)slow_fields[i_slow][i_ch].b)[i_in] = 
          slow_data[i_slow][i_ch];

        if(++i_in == slow_fields[i_slow][i_ch].i_out) {
          // Buffer overflow: do something
        }
        if (i_in>=MAXBUF) i_in = 0;
        slow_fields[i_slow][i_ch].i_in = i_in;
      }
    }
    n_frames++;
    //    if (n_frames % 5 == 0) { 
    //      printf("file: %s frame: %d\r", filedirname, n_frames);
    //      fflush(stdout);
    //    } 
  }

  /********************/
  /* normal fast data */
  for (i_ch = 0; i_ch < n_fast; i_ch++) {
    i_in = normal_fast[i_ch].i_in;
    if (normal_fast[i_ch].size == 2) {
      ((unsigned *)normal_fast[i_ch].b)[i_in] =
        *((unsigned *)(Rxframe+normal_fast[i_ch].i0));      
    } else {
      ((unsigned short*)normal_fast[i_ch].b)[i_in] =
        ((unsigned short*)Rxframe)[normal_fast[i_ch].i0];      
    }
    if(++i_in == normal_fast[i_ch].i_out) {
      // Buffer overflow: do something
    }

    if (i_in>=MAXBUF) i_in = 0;
    normal_fast[i_ch].i_in = i_in;
  };

  /********************\
    |* fast bolo data   *|
    \********************/
  for (i_card = 0; i_card<DAS_CARDS; i_card++) {
    for (i_ch = 0; i_ch<DAS_CHS; i_ch += 2) {
      B0 = (unsigned)Rxframe[boloIndex[i_card][i_ch][0]] +
        (((unsigned)Rxframe[boloIndex[i_card][i_ch][1]] & 0xff00) << 8);
      B1 = Rxframe[boloIndex[i_card][i_ch + 1][0]] +
        ((Rxframe[boloIndex[i_card][i_ch + 1][1]] & 0x00ff) << 16);

      i_in = bolo_fields[i_card][i_ch].i_in;
      ((unsigned *)bolo_fields[i_card][i_ch].b)[i_in] = B0;
      ((unsigned *)bolo_fields[i_card][i_ch + 1].b)[i_in] = B1;

      if(++i_in == bolo_fields[i_card][i_ch].i_out) {
        // Buffer overflow: do something
      }	

      if (i_in>=MAXBUF) i_in = 0;
      bolo_fields[i_card][i_ch].i_in = bolo_fields[i_card][i_ch+1].i_in = i_in;
    }
  }
}

// separate thread: writes each field to disk
void DirFileWriter(void) {
  unsigned short buffer[MAXBUF];
  unsigned int ibuffer[MAXBUF];
  int i_ch, i_slow, i_card;
  int i_in, i_out, i_buf;
  int k;
  int d_field = 0;
  static int max_d_field = 0;
  static int min_d_field = 10000;

  while (1) {
    d_field = 0;

    /** slow data **/
    for (i_slow = 0; i_slow<N_SLOW; i_slow++) {
      for (i_ch = 0; i_ch<FAST_PER_SLOW; i_ch++) {

        if (slow_fields[i_slow][i_ch].fp >= 0) {
          i_in = slow_fields[i_slow][i_ch].i_in;
          i_out = slow_fields[i_slow][i_ch].i_out;
          i_buf = 0;
          while (i_in != i_out) {
            if ((SlowChList[i_slow][i_ch].type=='U') ||
                (SlowChList[i_slow][i_ch].type=='I')) {
              ibuffer[i_buf] =
                (unsigned)
                ((((unsigned short *)slow_fields[i_slow][i_ch].b)[i_out]) << 16)
                | (unsigned)
                (((unsigned short *)slow_fields[i_slow + 1][i_ch].b)[i_out]);
            } else {
              buffer[i_buf] =
                ((unsigned short *)slow_fields[i_slow][i_ch].b)[i_out];
            }
            i_out++;
            i_buf++;
            if (i_out>=MAXBUF) i_out = 0;
          }
          if ((SlowChList[i_slow][i_ch].type=='U') ||
              (SlowChList[i_slow][i_ch].type=='I')) {
            if (i_buf > 0) write(slow_fields[i_slow][i_ch].fp,
                ibuffer, i_buf*sizeof(unsigned));
          } else {
            if (i_buf > 0) write(slow_fields[i_slow][i_ch].fp,
                buffer, i_buf*sizeof(unsigned short));
          }
          slow_fields[i_slow][i_ch].i_out = i_out;
        }
      }
    }

    /** normal fast data */
    for(i_ch = 0; i_ch<n_fast; i_ch++) {
      i_in = normal_fast[i_ch].i_in;
      i_out = normal_fast[i_ch].i_out;
      i_buf = 0;
      if (normal_fast[i_ch].size == 2) {
        while (i_in!=i_out) {
          ibuffer[i_buf] = ((unsigned int *)normal_fast[i_ch].b)[i_out];
          i_out++;
          i_buf++;
          if (i_out>=MAXBUF) i_out = 0;
        }
        if (i_buf > 0) write(normal_fast[i_ch].fp,
            ibuffer, i_buf*sizeof(unsigned int));
      } else {
        while (i_in!=i_out) {
          buffer[i_buf] = ((unsigned short *)normal_fast[i_ch].b)[i_out];
          i_out++;
          i_buf++;
          if (i_out>=MAXBUF) i_out = 0;
        }
        if (i_buf > 0) write(normal_fast[i_ch].fp,
            buffer, i_buf*sizeof(unsigned short));
      }
      normal_fast[i_ch].i_out = i_out;
    }

    /** Bolometer data */
    for (i_card = 0; i_card<DAS_CARDS; i_card++) {
      for (i_ch = 0; i_ch<DAS_CHS; i_ch++) {

        i_in = bolo_fields[i_card][i_ch].i_in;
        i_out = bolo_fields[i_card][i_ch].i_out;

        i_buf = 0;
        k = 0;
        while (i_in != i_out) {
          ibuffer[i_buf] =
            ((unsigned int *)bolo_fields[i_card][i_ch].b)[i_out];
          i_out++;
          i_buf++;
          if (i_out>=MAXBUF) i_out = 0;
        }
        if (i_buf > 0)
          write(bolo_fields[i_card][i_ch].fp,
            ibuffer, i_buf*sizeof(unsigned int));
        if (i_buf>d_field) d_field = i_buf;	
        if (max_d_field < d_field) max_d_field = d_field;
        if (min_d_field > d_field) min_d_field = d_field;
        bolo_fields[i_card][i_ch].i_out = i_out;
      }
    }
    usleep(400000);
  }
}
