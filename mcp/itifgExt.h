/*		 ITI-FG  Kernel module free public release
 *
 *		     Copyright (C) 1996-2001 GOM mbH.
 *	         Mittelweg 7-8, 38106 Braunschweig, GERMANY
 *		            All rights reserved
 *
 *
 * $Id: itifgExt.h,v 1.1.1.1 2003-06-16 21:07:13 root Exp $
 *
 */

#ifndef _ITIFGEXT_H
#define _ITIFGEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Includes:
 */

/* #undef linux */
#undef EXT_OS_DEFINED

#ifdef linux
#  define EXT_OS_DEFINED
#  ifdef __KERNEL__
#    include <linux/time.h>	/* struct timeval/timespec */
#    include <linux/ioctl.h>	/* _IOR/W ... */
#  else
#    include <sys/types.h>	/* basic data types */
#    include <sys/time.h>	/* struct timeval/timespec */
#    include <sys/ioctl.h>	/* _IOR/W ... */
#  endif
#endif

#ifdef __NetBSD__
#  define EXT_OS_DEFINED
#  include <sys/time.h>
#  include <sys/ioccom.h>
#  define ENODATA	ETIMEDOUT
#endif

/*
 * Defines:
 */

/* function return values */
#define OK	0
#define ERROR	-1

/* hardware bit values */
#define LOW	0
#define HIGH	1

/* boolean logic values */

/* boolean logic values */
#ifndef FALSE
#  define FALSE	0
#endif
#ifndef TRUE
#  define TRUE	1
#endif

/* pointer init/error value */
#ifndef NULL
#  define NULL	(void*)0
#endif

/* if nothing is defined, fallback */
#ifndef u_char
#  define u_char	unsigned char
#endif
#ifndef u_short
#  define u_short	unsigned short
#endif
#ifndef u_long
#  define u_long	unsigned long
#endif
#ifndef u_int
#  define u_int		unsigned int
#endif

/* error handling (kernel and user space) */
#ifdef DEBUG_ITIFG
#  if defined(linux) && defined(__KERNEL__) || \
      defined(__NetBSD__) && defined(_KERNEL) || \
      !defined(EXT_OS_DEFINED)
#    if (DEBUG_ITIFG & 1)
#      define ITI_PRINT0(fmt) iti_printd (fmt)
#      define ITI_PRINT0A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 2)
#      define ITI_PRINT1(fmt) iti_printd (fmt)
#      define ITI_PRINT1A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 4)
#      define ITI_PRINT2(fmt) iti_printd (fmt)
#      define ITI_PRINT2A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 8)
#      define ITI_PRINT3(fmt) iti_printd (fmt)
#      define ITI_PRINT3A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 16)
#      define ITI_PRINT4(fmt) iti_printd (fmt)
#      define ITI_PRINT4A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 32)
#      define ITI_PRINT5(fmt) iti_printd (fmt)
#      define ITI_PRINT5A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 64)
#      define ITI_PRINT6(fmt) iti_printd (fmt)
#      define ITI_PRINT6A(args...) iti_printda (args)
#    endif
#    if (DEBUG_ITIFG & 128)
#      define ITI_PRINT7(fmt) iti_printd (fmt)
#      define ITI_PRINT7A(args...) iti_printda (args)
#    endif
#  else
#    define iti_printf(args...) fprintf (stderr, args); \
				    fflush (stderr);
#    define ITI_PRINTF(args...) fprintf (stderr, args); \
				    fflush (stderr);
#  endif
#else
#  if !defined(linux) || !defined(__KERNEL__) || !defined(EXT_OS_DEFINED)
#    define iti_printf(args...) fprintf (stderr, args); \
				    fflush (stderr);
#  endif
#  define ITI_PRINTF(fmt,args...)
#  define ITI_PRINTD(fmt)
#  define ITI_PRINTDA(fmt,args...)
#endif

#ifndef ITI_PRINT0
#define ITI_PRINT0(fmt)
#endif
#ifndef ITI_PRINT0A
#define ITI_PRINT0A(fmt,args...)
#endif
#ifndef ITI_PRINT1
#define ITI_PRINT1(fmt)
#endif
#ifndef ITI_PRINT1A
#define ITI_PRINT1A(fmt,args...)
#endif
#ifndef ITI_PRINT2
#define ITI_PRINT2(fmt)
#endif
#ifndef ITI_PRINT2A
#define ITI_PRINT2A(fmt,args...)
#endif
#ifndef ITI_PRINT3
#define ITI_PRINT3(fmt)
#endif
#ifndef ITI_PRINT3A
#define ITI_PRINT3A(fmt,args...)
#endif
#ifndef ITI_PRINT4
#define ITI_PRINT4(fmt)
#endif
#ifndef ITI_PRINT4A
#define ITI_PRINT4A(fmt,args...)
#endif
#ifndef ITI_PRINT5
#define ITI_PRINT5(fmt)
#endif
#ifndef ITI_PRINT5A
#define ITI_PRINT5A(fmt,args...)
#endif
#ifndef ITI_PRINT6
#define ITI_PRINT6(fmt)
#endif
#ifndef ITI_PRINT6A
#define ITI_PRINT6A(fmt,args...)
#endif
#ifndef ITI_PRINT7
#define ITI_PRINT7(fmt)
#endif
#ifndef ITI_PRINT7A
#define ITI_PRINT7A(fmt,args...)
#endif

#define TV_TO_MS(tv, ms)	/* struct timeval -> milliseconds */ \
	((ms) = (((tv).tv_sec * 1000) + (tv).tv_usec / 1000))
#define MS_TO_TV(ms, tv)	/* milliseconds -> struct timeval */ \
	((tv).tv_sec = (ms) / 1000, (tv).tv_usec = \
	 ((ms) * 1000) - (tv).tv_sec * 1000000)

#define TV_MINUS(tv, tv1, tv2)	/* tv = tv1 - tv2 ( tv1 > tv2! ) */ \
	if ((tv1).tv_usec < (tv2).tv_usec) \
	  { \
	    (tv1).tv_usec += 1000000; \
	    (tv1).tv_sec--; \
	  } \
	(tv).tv_usec = (tv1).tv_usec - (tv2).tv_usec; \
	(tv).tv_sec = (tv1).tv_sec - (tv2).tv_sec;

#define TV_PLUS(tv, tv1, tv2)	/* tv = tv1 + tv2 */ \
	(tv).tv_sec = (tv1).tv_sec + (tv2).tv_sec; \
	(tv).tv_usec = (tv1).tv_usec + (tv2).tv_usec; \
	if ((tv).tv_usec > 1000000) \
	  { \
	    (tv).tv_usec -= 1000000; \
	    (tv).tv_sec++; \
	  }

#define ROUND_DOWNTO_PAGE(addr) \
	((unsigned long)(addr) & ~(PAGE_SIZE - 1))
     
#define ROUND_UPTO_PAGE(addr) \
	(((unsigned long)(addr) + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))

/* global structure limits */
#define ITI_BOARDS_MAX	8	/* max. number of boards to plug in */
#define ITI_CAMERAS_MAX	4	/* maximal number of cameras per module */
#define ITI_MODULE_MAX	64	/* module number space */

/* device name defines */
#define ITI_NAME_PREFIX		"/dev/ic"
#define ITI_NAME_DATA_POSTFIX	"idI"
#define ITI_NAME_PORT_POSTFIX	"ioP"

/* shortkeys for acquisition modules */
#define ICP_AMNO	0	/* No Acquisition Module Installed */ 
#define ICP_AMFA	1	/* Fast Acquisition Module */
#define ICP_AMC1	2	/* Color Acquisition Module */
#define ICP_AMVS	3	/* Variable Scan Module */
#define ICP_AMDG	4	/* Digital Module */
#define ICP_AMMTD	6	/* Multiple Tapped Module */
#define ICP_AMCMP	7	/* Standard Composite Sync Module */
#define ICP_AMRGB	8	/* Standard RGB Module */
#define ICP_AMPV	16	/* PCVision */ 
#define ITI_VMPCD	32	/* PCDig (use first invalid IC-PCI module) */

/* maximum number of cameras per acq. module */
#define VS_CAMERAS_MAX	4
#define DG_CAMERAS_MAX	1
#define PV_CAMERAS_MAX	4
#define CMP_CAMERAS_MAX	4

#define PCD_CAMERAS_MAX	1

#define	ITI_FPS_MIN	0.01	/* we support frames per second from */
#define	ITI_FPS_MAX	100.0	/* up to ... */

#define ITI_INIT_FPS	25.0	/* 50 Hz CCIR (interlaced!) */

#define ITI_WAIT_IOP	2000	/* inport data waiting time (20s) */

#define ITI_WAIT_TRG	3000	/* trigger image waiting time (30s * 2.5) */

#define ITI_DLY_BASE	70	/* time base in nanoseconds (1/14.31818 MHz) */

/* limits for the external sync delay time (AM-DIG) */
#define DG_DLY_MIN	((ITI_DLY_BASE << 1) * 3)
			/* min prescaler and duration counter */
#define DG_DLY_MAX	((ITI_DLY_BASE << 11) * 4098)
			/* max prescaler and duration counter */

/* limits for the external sync delay time (PCDig) (64bit!) */
#define PCD_DLY_MIN	((unsigned long long)(ITI_DLY_BASE) * 2)
			/* min prescaler and duration counter */
#define PCD_DLY_MAX	((unsigned long long)(ITI_DLY_BASE << 12) * 65537)
			/* max prescaler and duration counter */
/* this are the init defaults for the exposure/readout times */
#define PCD_EXTIME_INIT		 40000.000	/* us - default is 40ms */
#define PCD_ROTIME_INIT		 40000.000	/* us - default is 40ms */

/* size of converted fpga config file data */
#define PCD_CONFIG_BYTES	65824

/* maximal number of frames to read(2)/mmap(2) at once */
#define ITI_SNAPS_MAX	8

/* this is the maximal frame size to read */ 
#define ITI_SIZE_MAX	(8 * 1024 * 1024)

/* after every 64k bytes a transfer from image to main memory is initiated */
#define ITI_BLOCK_SIZE	0x10000	/* 1 << ITI_INTADR_SHIFT */

/*
 * Typedefs:
 */

/* AM specific data types */
/* lookup tables */
struct vs_lut_t
{
  u_char lutdata[256];
};

#define pv_lut_t vs_lut_t

union iti_lut16_t
{
  u_short lut8data[256];
  u_short lut10data[1024];
  u_short lut12data[4096];
  u_short lut14data[16384];
  u_short lut16data[65536];
};

#define dg_lut16_t iti_lut16_t
#define pcd_lut_t iti_lut16_t

struct dg_lut8_t
{
  u_char lutdata[256];
};

/* camera structures */
struct vs_cam_t
{
  u_char tsel;
  u_char clksrc;
  u_char lpfsel;
  u_char iscale;
  u_char smode;
  u_char tsrc;
  u_char lenpol;
  u_char fldpol;
  u_char fenpol;
  u_char pclkpol;
  u_char eclken;
  u_char fldsten;
  u_char vsten;
  u_char trgen;
  u_char trgsrc;
  u_char fmode;
  u_char tmode;
  u_char strbdly;
  u_short hoff;
  u_short hsz;
  u_short voff;
  u_short vsz;
  u_short froff;
  u_char cppos;
  u_char cpen;
  u_char cpsrc;
  u_char strbsel;
  u_char strbpol;
  u_char cpst;
  u_short nref;
  u_short pref;
};

struct dg_cam_t {
  u_char fsel;
  u_char clkdiv;
  u_char xillen;
  u_char mc;
  u_char pclkpol;
  u_char lenpol;
  u_char fenpol;
  u_char fldpol;
  u_char smode;
  u_char ilmode;
  u_char lensm;
  u_char nolmiss;
  u_char lvar;
  u_char exclk;
  u_char extmd;
  u_char exsen;
  u_char expol;
  u_char prien;
  u_char prip;
  u_short extime;
  u_char psize;
  u_char ilpixel;
  u_char muxa;
  u_char muxb;
  u_char muxc;
  u_short hoff;
  u_short hact;
  u_short voff;
  u_short vact;
  u_char trigen;
  u_char trigsrc;
};

struct pv_cam_t
{
  u_short htotal;
  u_short hsyncpw;
  u_short hequipw;
  u_short hserrpw;
  u_short vtotal;
  /* pv_vsyncpw_t */
  u_char vsyncpw;
  u_char hsyncpol;
  u_char vsyncpol;

  u_short vgstart;
  u_short vgwidth;
  u_short hoffset;
  u_short hactive;
  u_short voffset;
  u_short vactive;
  /* pv_strbdly_t */
  u_char strbdly;
  u_char lenpol;	/* REV-B Only! */
  u_char fenpol;	/* REV-B Only! */
  /* pv_frstdly_t */
  u_char froff;
  u_char frstonv;
  u_char frstmd;
  u_char frstpol;
  u_char frstsz;
  /* pv_con1_t */
  u_char syncsel;
  u_char chsyncen;
  u_char vsyncen;
  u_char clksrc;
  u_char lpfsel;
  u_char vscan;		/* REV-B Only! */
  u_char iscale;
  u_char vclkpol;	/* REV-B Only! */
  u_char smode;
  u_char camsel;
  u_char fldshft;
  /* pv_trigcon_t */
  u_char trigen;
  u_char trigsrc;
  u_char trigpol;
  u_char strbmd;
  u_char strben;	/* REV-B Only! */
  u_char strbpol;
  u_char skpfldmd;
  /* pv_p_clamp_t */
  u_char ccnt;
  u_char pclmpol;
  u_char clmpsrc;
  /* pll registers */
  u_short fdiv;
  u_short rdiv;
  u_short vcogain;
  u_short pda;
  u_short lcnt;
  u_short nref;
  u_short pref;
};

struct cmp_cam_t {
  u_char        status;
  u_char        iform;
  u_char        tdec;
  u_short       vdelay;
  u_short       vactive;
  u_short       hdelay;
  u_short       hactive;
  u_short       hscale;
  u_char        bright;
  u_char        control;
  u_short       contrast;
  u_short       sat_u;
  u_short       sat_v;
  u_char        hue;
  u_char        scloop;
  u_char        oform;
  u_char        vscale_hi;
  u_short       vscale;
  u_char        adly;
  u_char        bdly;
  u_char        adc;
  u_char        vtc;
};

struct pcd_cam_t
{
  /* pcd_ca_ctcon_t */
  u_char lenenb;
  u_char lensync;
  u_char lenpol;
  u_char fenenb;
  u_char fenpol;
  u_char multenb;
  u_char multpol;
  u_char multmd;
  /* pcd_ca_outcon_t */
  u_char restime;
  u_char respol;
  u_char cam3enb;
  u_char exsenb;
  u_char exspol;
  u_char exsmd;
  u_char prienb;
  u_char pripol;
  u_char cammd;
  u_char cclkenb;
  u_char cclkpol;
  /* pcd_ca_exscon_t */
  u_char exssel;
  u_char prisel;
  u_char prihold;

  double extime; /* from this we calculate the exsclk /exstime regs */
  double rotime; /* from this we calculate the emidpnt/pmidpnt regs */
  u_long hoff;
  u_long hact;
  u_long voff;
  u_long vact;
  float cclkfreq; /* from this we calculate the bitword for VCO 'A' */
  /* pcd_ca_lutcon0_t */
  /* pcd_ca_lutcon1_t */
  /* pcd_ca_trigcon_t */
  u_char trig0pol;
  u_char trig0src;
  u_char trig0gd;
  u_char trig1pol;
  u_char trig1src;
  u_char trig1gd;
  /* pcd_ca_trigdiv_t */
  u_char trigdiv;
  u_char divcon;

  int pixelsize; /* this value we need to load the lookup tables right */
  int geometry; /* this we need to load the source scatter gather right */
};

/* to handle different modules in the same way */
union iti_dev_t
{
  struct vs_cam_t vs_cam;
  struct dg_cam_t dg_cam;
  struct pv_cam_t pv_cam;
  struct cmp_cam_t cmp_cam;
  struct pcd_cam_t pcd_cam;
};

/* some useful accounting */
struct iti_acc_t
{
  /* grab statistics */
  u_long frames_captured;	/* frames acquired to on board memory */
  u_long frames_transfered;	/* frames transfered to main memopry */
  u_long frames_timedout;	/* frames not acquired into given period */
  u_long frames_get_by_blocked;	/* frame is acquired by a blocking read */
  u_long frames_get_by_signal;	/* frame is acquired by receiving a signal */
  u_long frames_get_by_select;	/* frame is acquired waiting with select */
  /* ioport statistics */
  u_long events_captured;	/* overall number of captured interrupts */
  u_long bytes_valid;		/* inport data are stored (read(2) assigned) */
  u_long bytes_lost;		/* ioport data are lost (no process in read) */
};

/* snap statistics */
#ifndef _ITIFGOS_H	/* already defined in itifgOS.h */
#  ifndef EXT_OS_DEFINED	/* if nothing is defined, fallback */
struct timeval
{
  long	tv_sec;		/* seconds */
  long	tv_usec;	/* microseconds */
};
#  endif
#endif

struct iti_info_t
{
  struct timeval timestamp;
  struct iti_acc_t framenums;
};

/* delay time */
#ifndef EXT_OS_DEFINED	/* if nothing is defined, fallback */
struct timespec
{
  long	tv_sec;		/* seconds */
  long	tv_nsec;	/* nanoseconds */
};
#endif

/* defines for procfs entries */
#  define PFS_PREFIX	"/proc/"
#  define PFS_STRING	"itifg"
 
#  define PFS_VERSION_IDENT	"Version:"
#  define PFS_DATE_IDENT	"Date:"
#  define PFS_BOARDS_IDENT	"Boards:"
#  define PFS_MODULES_IDENT	"Modules:"

/* ... or get the setup via ioctl */
struct iti_setup_t
{
  long version;
  char date[128];
  int boards;
  int modules[ITI_BOARDS_MAX];
};

struct mod_names_t
{
  int id;
  char name[16];
};

enum exmode_t
{
  EX_CONF,	/* take mode from the camera config */
  EX_FREE,	/* set to free running */
  EX_SOFT0,	/* set to software trigger 0/1 */
  EX_SOFT1,
  EX_HARD0,	/* set to hardware trigger 0/1 */
  EX_HARD1
};

#if defined(linux) || defined(__NetBSD__)

/* IC-PCI ioctl defines use 'G' for coding and range from 0 to 31 */

#define GIOC_CR_SET_STOP	_IO  ('G', 0)
#define GIOC_CR_SET_START	_IO  ('G', 1)
#define GIOC_CR_SET_TRIG	_IO  ('G', 2)
#define GIOC_CR_SET_HDEC	_IOW ('G', 3, int)
#define GIOC_CR_SET_VDEC	_IOW ('G', 4, int)

/* AM-VS ioctl defines use 'G' for coding and range from 96 to 127 */

#define GIOC_VS_GET_LUT		_IOR ('G',  96, struct vs_lut_t)
#define GIOC_VS_SET_LUT		_IOW ('G',  97, struct vs_lut_t)
#define GIOC_VS_SET_LUT_LIN	_IO  ('G',  98)
#define GIOC_VS_GET_NREF	_IOR ('G',  99, int)
#define GIOC_VS_SET_NREF	_IOW ('G', 100, int)
#define GIOC_VS_GET_PREF	_IOR ('G', 101, int)
#define GIOC_VS_SET_PREF	_IOW ('G', 102, int)

/* AM-DIG ioctl defines use 'G' for coding and range from 128 to 159 */

#define GIOC_DG_GET_LUTPG	_IOR ('G', 128, int)
#define GIOC_DG_SET_LUTPG	_IOW ('G', 129, int)
#define GIOC_DG_GET_LUT		_IOR ('G', 130, void*) /* union dg_lut16_t */
#define GIOC_DG_SET_LUT		_IOW ('G', 131, void*) /* union dg_lut16_t */
#define GIOC_DG_GET_OLUTPG	_IOR ('G', 132, int)
#define GIOC_DG_SET_OLUTPG	_IOW ('G', 133, int)
#define GIOC_DG_GET_OLUT	_IOR ('G', 134, struct dg_lut8_t)
#define GIOC_DG_SET_OLUT	_IOW ('G', 135, struct dg_lut8_t)
#define GIOC_DG_SET_LUT_LIN	_IO  ('G', 136)
#define GIOC_DG_SET_NORM_TO8	_IO  ('G', 137)
#define GIOC_DG_SET_NORM_TO16	_IO  ('G', 138)
#define GIOC_DG_GET_EXTIME	_IOR ('G', 139, struct timespec)
#define GIOC_DG_SET_EXTIME	_IOW ('G', 140, struct timespec)
#define GIOC_DG_GET_CAMMD	_IOR ('G', 141, int)
#define GIOC_DG_SET_CAMMD	_IOW ('G', 142, int)

/* GENERIC ioctl defines use 'G' for coding and range from 224 to 255 */

#define GIOC_GE_GET_SETUP	_IOR ('G', 224, struct iti_setup_t)

#define GIOC_GE_GET_WIDTH	_IOR ('G', 225, short)
#define GIOC_GE_GET_HEIGHT	_IOR ('G', 226, short)
#define GIOC_GE_GET_DEPTH	_IOR ('G', 227, int)
#define GIOC_GE_GET_RAWSIZE	_IOR ('G', 228, size_t)
#define GIOC_GE_GET_PAGEDSIZE	_IOR ('G', 229, size_t)

#define GIOC_GE_GET_CAMERA	_IOR ('G', 230, int)
#define GIOC_GE_SET_CAMERA	_IOW ('G', 231, int)
#define GIOC_GE_SET_DEFCNF	_IO  ('G', 232)
#define GIOC_GE_GET_CAMCNF	_IOR ('G', 233, union iti_dev_t)
#define GIOC_GE_SET_CAMCNF	_IOW ('G', 234, union iti_dev_t)

#define GIOC_GE_GET_SYNCWAIT	_IOR ('G', 235, struct timeval)
#define GIOC_GE_SET_TIMEOUT	_IOW ('G', 236, struct timeval)

#define GIOC_GE_GET_STATS	_IOR ('G', 237, struct iti_acc_t)
#define GIOC_GE_SET_STATS	_IO  ('G', 238)

/* AM-STD/COMP ioctl defines use 'H' for coding and range from 64 to 95 */

#define HIOC_CMP_SET_TO_BW	_IO  ('H',  64)
#define HIOC_CMP_SET_TO_COL	_IO  ('H',  65)

#define GIOC_GE_GET_AQSIZE      _IOR ('G', 238, int) 

#define GIOC_GE_GET_AQSIZE      _IOR ('G', 238, int) 

/* PCVision ioctl defines use 'H' for coding and range from 96 to 127 */

#define HIOC_PV_GET_LUT		_IOR ('H',  96, struct vs_lut_t)
#define HIOC_PV_SET_LUT		_IOW ('H',  97, struct vs_lut_t)
#define HIOC_PV_SET_LUT_LIN	_IO  ('H',  98)
#define HIOC_PV_GET_NREF	_IOR ('H',  99, int)
#define HIOC_PV_SET_NREF	_IOW ('H', 100, int)
#define HIOC_PV_GET_PREF	_IOR ('H', 101, int)
#define HIOC_PV_SET_PREF	_IOW ('H', 102, int)

/* PCDig ioctl defines use 'H' for coding and range from 128 to 159 */

#define HIOC_PCD_GET_LUT0PG	_IOR ('H', 128, int)
#define HIOC_PCD_SET_LUT0PG	_IOW ('H', 129, int)
#define HIOC_PCD_GET_LUT0	_IOR ('H', 130, void*) /* union pcd_lut_t */
#define HIOC_PCD_SET_LUT0	_IOW ('H', 131, void*) /* union pcd_lut_t */
#define HIOC_PCD_GET_LUT1PG	_IOR ('H', 132, int)
#define HIOC_PCD_SET_LUT1PG	_IOW ('H', 133, int)
#define HIOC_PCD_GET_LUT1	_IOR ('H', 134, void*) /* union pcd_lut_t */
#define HIOC_PCD_SET_LUT1	_IOW ('H', 135, void*) /* union pcd_lut_t */
#define HIOC_PCD_SET_LUT_LIN	_IO  ('H', 136)
#define HIOC_PCD_SET_NORM_TO8	_IO  ('H', 137)
#define HIOC_PCD_SET_NORM_TO16	_IO  ('H', 138)
#define HIOC_PCD_SET_EXMODE	_IOW ('H', 139, enum exmode_t)
#define HIOC_PCD_GET_EXTIME	_IOR ('H', 140, struct timespec)
#define HIOC_PCD_SET_EXTIME	_IOW ('H', 141, struct timespec)
#define HIOC_PCD_GET_CAMMD	_IOR ('H', 142, int)
#define HIOC_PCD_SET_CAMMD	_IOW ('H', 143, int)
/* Set/get trigger modes (i.e. how trigger controls start/end of acquire, 
   not sync signal). PCD_TMODE_... macros in pcdigReg.h define value range 
   and interpretation. */
#define HIOC_PCD_GET_STMODE     _IOR ('H', 144, int)
#define HIOC_PCD_SET_STMODE     _IOW ('H', 145, int)
#define HIOC_PCD_GET_ETMODE     _IOR ('H', 146, int)
#define HIOC_PCD_SET_ETMODE     _IOW ('H', 147, int)
/* ... and number of frames to capture, values [PCD_FCNT_MIN, PCD_FCNT_MAX] 
   or PCD_FCNT_INFINITE */
#define HIOC_PCD_GET_FCNT     	_IOR ('H', 148, int)
#define HIOC_PCD_SET_FCNT     	_IOW ('H', 149, int)

#endif /* linux */

#ifndef EXT_OS_DEFINED	/* if nothing is defined, fallback */
#define GIOC_GE_GET_AQSIZE      128
#define GIOC_CR_SET_STOP	1
#define GIOC_CR_SET_START	2
#define GIOC_CR_SET_TRIG	3
#define GIOC_CR_SET_HDEC	4 
#define GIOC_CR_SET_VDEC	5 
#define GIOC_VS_GET_LUT		6 
#define GIOC_VS_SET_LUT		7 
#define GIOC_VS_SET_LUT_LIN	8 
#define GIOC_VS_GET_NREF	9 
#define GIOC_VS_SET_NREF	10
#define GIOC_VS_GET_PREF	11
#define GIOC_VS_SET_PREF	12
#define GIOC_DG_GET_LUTPG	13
#define GIOC_DG_SET_LUTPG	14
#define GIOC_DG_GET_LUT		15
#define GIOC_DG_SET_LUT		16
#define GIOC_DG_GET_OLUTPG	17
#define GIOC_DG_SET_OLUTPG	18
#define GIOC_DG_GET_OLUT	19
#define GIOC_DG_SET_OLUT	20
#define GIOC_DG_SET_LUT_LIN	21
#define GIOC_DG_SET_NORM_TO8	22
#define GIOC_DG_SET_NORM_TO16	23
#define GIOC_DG_GET_EXTIME	24
#define GIOC_DG_SET_EXTIME	25
#define GIOC_DG_GET_CAMMD	26
#define GIOC_DG_SET_CAMMD	27
#define GIOC_GE_GET_SETUP	28
#define GIOC_GE_GET_WIDTH	29
#define GIOC_GE_GET_HEIGHT	30
#define GIOC_GE_GET_DEPTH	31
#define GIOC_GE_GET_RAWSIZE	32
#define GIOC_GE_GET_PAGEDSIZE	33
#define GIOC_GE_GET_CAMERA	34
#define GIOC_GE_SET_CAMERA	35
#define GIOC_GE_SET_DEFCNF	36
#define GIOC_GE_GET_CAMCNF	37
#define GIOC_GE_SET_CAMCNF	38
#define GIOC_GE_GET_SYNCWAIT	39
#define GIOC_GE_SET_TIMEOUT	40
#define GIOC_GE_GET_STATS	41
#define GIOC_GE_SET_STATS	42
#define HIOC_CMP_SET_TO_BW	43
#define HIOC_CMP_SET_TO_COL	44
#define HIOC_PV_GET_LUT		45
#define HIOC_PV_SET_LUT		46
#define HIOC_PV_SET_LUT_LIN	47
#define HIOC_PV_GET_NREF	48
#define HIOC_PV_SET_NREF	49
#define HIOC_PV_GET_PREF	50
#define HIOC_PV_SET_PREF	51
#define HIOC_PCD_GET_LUT0PG	52
#define HIOC_PCD_SET_LUT0PG	53
#define HIOC_PCD_GET_LUT0	54
#define HIOC_PCD_SET_LUT0	55
#define HIOC_PCD_GET_LUT1PG	56
#define HIOC_PCD_SET_LUT1PG	57
#define HIOC_PCD_GET_LUT1	58
#define HIOC_PCD_SET_LUT1	59
#define HIOC_PCD_SET_LUT_LIN	60
#define HIOC_PCD_SET_NORM_TO8	61
#define HIOC_PCD_SET_NORM_TO16	62
#define HIOC_PCD_SET_EXMODE	63
#define HIOC_PCD_GET_EXTIME	66
#define HIOC_PCD_SET_EXTIME	67
#define HIOC_PCD_GET_CAMMD	68
#define HIOC_PCD_SET_CAMMD	69
#define HIOC_PCD_GET_STMODE     70
#define HIOC_PCD_SET_STMODE     71
#define HIOC_PCD_GET_ETMODE     72
#define HIOC_PCD_SET_ETMODE     73
#define HIOC_PCD_GET_FCNT       74
#define HIOC_PCD_SET_FCNT       75

#endif /* nothing defined */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ITIFGEXT_H */

