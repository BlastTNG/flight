/*	@(#)tables.h	1.6	04/20/94

	tables.h

	tables.h contains the variables and the structures
needed by tablec. to read and interpret the star tables from
the Guide Star Catalog.

	4/8/93 -- Added conditional compilation allowing relocation
	of lookup tables.

*/

/*	Location of the files that provide indexing into the GSC */
#ifndef LG_TBL_NAME
#define LG_TBL_NAME	"/data/pointing/gsc_index/lg_reg_x.tbl"
#endif
#ifndef SM_TBL_NAME
#define SM_TBL_NAME	"/data/pointing/gsc_index/sm_reg_x.tbl"
#endif
#ifndef REG_TBL_NAME
#define REG_TBL_NAME	"/data/pointing/gsc_index/regions.tbl"
#endif

/*	FITS files are broken up into blocks of a given size.
	Read a block at a time	*/
#define	FITS_READ_SIZE	2880
#define	FITS_DATA_SIZE	sizeof(char)	/* Unit of data	*/

/*  Each of the files that have information that we need has
a header that is some integer number of FITS read sizes.	*/
#define	LG_HEADER_SIZE	3
#define SM_HEADER_SIZE	3
#define REG_HEADER_SIZE	5
#define GSC_HEADER_SIZE	3
#define	SAO_HEADER_SIZE	10

/*	Number of entries in each table	*/
#define LG_NUM_ENTRIES	24
#define SM_NUM_ENTRIES	732
#define REG_NUM_ENTRIES	9537
#define NUM_SAO_ENTRIES	258997

/* 	Keyword before number of entries in header	*/
/*	(This should make the above references obsolete.	) */

#define	NUM_ENTRIES_KEYWORD	"NAXIS2  = "


/*  Location of the two CD's	*/
#define	NORTHERN_CD	"/cdrom"
#define	SOUTHERN_CD	"/who_knows_where"

#define SOUTH_BEGIN	367	/* First large field in the 
				southern hemisphere.	*/



/*  There are two sets of structures that are used.  The first
frames the FITS files into strings, the second set is the result
of reading the first set.  The format of the first is determined
by the format of the FITS file.  The second derives from the
first, and thus depends only on how you wish to internally 
represent the table data.	*/

typedef struct {
	char	dec_ctr[6];	/*  Center of Declination Band	*/
	char	blank[2];	/*  Spaces	*/
	char	n_lg_reg[2];	/*  Number of Large regions in dec band */
	char	blank2[2];	/*  Spaces	*/
	char	f_lg_reg[3];	/*  ID of first large region in band	*/
}	LG_RECORD;

typedef	struct {
	char	lg_reg_n[3];	/*  Number (ID) of the large region.	*/
	char	blank[1];
	char	f_sm_reg[5];	/*  Number (ID) of first small region in
				the large region.	*/
	char	blank2[2];	/*  Spaces	*/
	char	depth[1];	/*  Depth of subdivision of large region */
}	SM_RECORD;

typedef	struct {
	char	reg_no[5];	/*  Region number	*/
	char	blank[2];	/* Spaces	*/
	char	ra_h_low[2];	/* Alpha Low Limit - Hours */
	char	blank2[1];	
	char	ra_m_low[2];	/* Alpha Low Limit - Minutes */
	char	blank3[1];	
	char	ra_s_low[5];	/* Alpha Low Limit - Seconds */
	char	blank4[1];
	char	ra_h_hi[2];	/* Alpha High Limit - Hours */
	char	blank5[1];	
	char	ra_m_hi[2];	/* Alpha High Limit - Minutes */
	char	blank6[1];	
	char	ra_s_hi[5];	/* Alpha High Limit - Seconds */
	char	blank7[1];
	char	decsi_lo[1];	/* Delta Low Limit - Sign */
	char	dec_d_lo[2];	/* Delta Low Limit - Degrees */
	char	blank9[1];	
	char	dec_m_lo[4];	/* Delta Low Limit - Minutes */
	char	blank10[1];
	char	decsi_hi[1];	/* Delta High Limit - Sign */
	char	dec_d_hi[2];	/* Delta High Limit - Degrees */
	char	blank12[1];	
	char	dec_m_hi[4];	/* Delta High Limit - Minutes */
}	REG_RECORD;

typedef struct {
	char	gsc_id[5];	/*  Note there are no blanks.	*/
	char	ra_deg[9];	
	char	dec_deg[9];	
	char	pos_err[5];
	char	mag[5];
	char	mag_err[4];
	char	mag_band[2];
	char	GSCclass[1];
	char	plate_id[4];
	char	multiple[1];
}	GSC_RECORD;

typedef struct {
	char	sao_num[6];	/* SAO Number.	*/
	char	blank1[70];	/* Ignored stuff	*/
	char	photo_mag[4];	/*  Photographic magnitude	*/
	char	V_mag[4];	/*  Visual magnitude	*/
	char	type[3];	/*  Spectral type	*/
	char	blank2[63];	/*  Other useless junk.	*/
	char	ra_hrs[2];	/*  Ra hours (J2000)	*/
	char	ra_min[2];	/*  Ra minutes	*/
	char	ra_sec[6];	/*  Ra seconds	*/
	char	blank3[7];	/*  Annual proper motion in RA	*/
	char	dec_sgn[1];	/*  Dec sign	*/
	char	dec_deg[2];	/*  Dec degrees (J2000)	*/
	char	dec_min[2];	/*  Arcminutes DEC	*/
	char	dec_sec[5];	/*  Arcseconds DEC	*/
	char	blank4[27];	/*  End of the record	*/
}	SAO_RECORD;

/*	The indexing may be dropped in some of the tables below.
It can be implied by its position in an array of entries	*/
typedef	struct {
	float	dec_low;	/*  Lower bound of declination band	*/
	float	dec_high;	/*  Upper bound of declination band	*/
	int	num_regions;	/*  Number of large regions in band	*/
	int	lg_reg_id;	/*  ID of first large region in field	*/
}	LARGE_TABLE;

typedef	struct	{
	int	sm_reg_id;	/*  ID of first small region in field	*/
	int	subdivision;	/*  Degree of subdivision of large field */
}	SMALL_TABLE;

typedef	struct {
	float	ra_high;	/*  Upper RA in decimal degrees	*/
	float	ra_low;		/*  Lower RA in decimal degrees	*/
	float	dec_high;	/*  Upper DEC in decimal degrees	*/
	float	dec_low;	/*  Lower DEC in decimal degrees	*/
}	REGION_TABLE;

#include <stdio.h>	/*  Pickup the defintion of a FILE	*/
typedef FILE SAO_ENTRY;

/*  I found a file that had 8658 star table entries in it.	*/
#define MAX_STARS_PER_FILE	15000

	/*  Maximum number of stars that will occur in any given file	*/
typedef struct {
	float	ra;
	float	dec;
	float	magnitude;
}	STAR;

typedef union {
	LARGE_TABLE	*lt;
	SMALL_TABLE	*st;
	REGION_TABLE	*rt;
	SAO_ENTRY	*saot;
}	ALL_TABLE_PTR;

/*  Mode flags for use in the generic calls.	*/
#define	LARGE_MODE	0
#define	SMALL_MODE	1
#define REGION_MODE	2
#define	GSC_MODE	3
#define	SAO_MODE	4


/*  HALF_BAND is used to find the upper and lower limits of 
the large bands, given the center declination value of the band.	*/
#define	HALF_BAND	3.750


/*	Can't believe I remembered this from K+R	*/
#define	MIN(a,b)	a>b?b:a
#define MAX(a,b)	a>b?a:b

/*  Conversion macros	*/
#define HMS_TO_DD(h,m,s) (15.0*h + 0.25*m + s/240.0)
#define DM_TO_DD(d,m)	(d + m/60.0)
#define SIGN(s)	(s=='-'?-1:1)
#define ABS(x) (x>0?x:(-1 * x))


int fits_to_table(char *fits, void *tbl_ptr, int mode, int want_to_read);
int read_table(const char *filename, void *table, int mode);
