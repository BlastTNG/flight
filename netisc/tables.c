/*	@(#)tables.c	2.1	09/04/92

	tabels.c	

	This file contains the code that should be used to read 
the tables off of the guide star catalog. 

	Created 3/12/92	(RM)

	Modified 3/16/92  Added filename as an argument to read_table.
	This will allow GSC star files to be read in in the same way.
	
	4/20/92	Added the SAO option.

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "tables.h"


/*	Many functions need space to read FITS data into.  */
char	fits[FITS_READ_SIZE];


int fits_to_table(char *fits, void *tbl_ptr, int mode, int want_to_read);


/*  read_lg_table reads the large region FITS table and packs it
into an array of LARGE_TABLE entries.  read_lg_table knows lots of
stuff from tables.h.  The caller has better know that there are 24
entries in the large table and provide space accoridingly.  'table'
is a pointer to the first entry.  Return values are positive good
negative bad.  read_table returns the number of read records.	*/


/*  Read table for the three cases is quite similar.  So I have expanded
the read_lg_table to read_table as a generic function.  read_tbl
is called with an additional argument that flags what type of
table is being read.	*/

int	
read_table(filename, table, mode)
const char	*filename;	/*  File to read from	*/
void	*table;	    /*  This will be cast into the appropriate 
			type of pointer according to mode.	*/
int	mode;		
{	
	int i;	/*  Ubiquitous local definition.	*/
	int status;	/*  Return value holder for some functions.	*/

	FILE	*fp;
	
	int	num_records_read;	/*  Number of table records read. */
	int	total_records;		/*  Mode dependant	*/
	int	num_header_blocks;		/*  Also mode dependant	*/
	int	table_entry_size;	/*  Size of fits entry for table  */

	char	*keyword_ptr;		/*  Pointer that wll point to 
					our keyword.	*/


	/*  Set up the variables to be used in the table reader.	*/
	switch(mode) {
	   case LARGE_MODE:
		num_header_blocks = LG_HEADER_SIZE;
		total_records = LG_NUM_ENTRIES;	/*  Number of entries	*/
		table_entry_size = sizeof(LG_RECORD);
		break;
	   case SMALL_MODE:
		num_header_blocks = SM_HEADER_SIZE;
		total_records = SM_NUM_ENTRIES;	/*  Number of entries	*/
		table_entry_size = sizeof(SM_RECORD);
		break;
	   case REGION_MODE:
		num_header_blocks = REG_HEADER_SIZE;
		total_records = REG_NUM_ENTRIES;  /*  Number of entries	*/
		table_entry_size = sizeof(REG_RECORD);
		break;
	   case	GSC_MODE:
		num_header_blocks = GSC_HEADER_SIZE;
		total_records = -1;	/*  Value will be read below.	*/
		table_entry_size = sizeof(GSC_RECORD);
		break;
	   case SAO_MODE:
		num_header_blocks = SAO_HEADER_SIZE;
		total_records = NUM_SAO_ENTRIES;
		table_entry_size = sizeof(SAO_RECORD);
		break;
	   default:
		printf("read_tbl called with unknown mode.\n");
		return(-4);
	}

		
	/*  Open the FITS file to be read	*/
	fp = fopen(filename, "r");	/*  Open file for reading.	*/
	if (fp == NULL) {
		/*  Error opening file	*/
		printf("Error opening %s for reading in read_table.\n",
			filename);
		return(-1);	/*  Return trouble.	*/
	}

	/*  Read in the first number of garbage/header blocks	*/
	//printf("Reading in %d header blocks and chucking them.\n", 
	//	num_header_blocks);
	for(i=0; i<num_header_blocks; i++) {
		status = (int)fread(fits, FITS_DATA_SIZE, FITS_READ_SIZE, fp);
		if (status != FITS_READ_SIZE) {
			printf("Error reading header in read_table\n");
			return(-2);
		}

		/*  Search the header block for the keyword that
		will give us the number of entries in the table.	*/
		keyword_ptr = strstr(fits, NUM_ENTRIES_KEYWORD);
		if (keyword_ptr != NULL) {
			/*  Keyword has been found, read out integer  */
			keyword_ptr += strlen(NUM_ENTRIES_KEYWORD);
			status = atoi(keyword_ptr);
		//	printf(
		//"Found Keyword:  Changing total_records from %d to %d.\n",
	//		total_records, status);
			total_records = status;
		}
	}

	
	/*	Read in a block of data and transform it into a table.
	Continue to do this untill the expected number of table records
	have been read.	*/
	
	num_records_read = 0;	/*  Starting read	*/
	while(num_records_read < total_records) {
	/*  Continue until all records have been read.	*/

		/*  Read in a block of data.	*/
		status = (int)fread(fits, FITS_DATA_SIZE, FITS_READ_SIZE, fp);
		if (status != FITS_READ_SIZE) {
			printf("Error reading table in read_table.\n");
			printf("%d out of %d read.\n", status, FITS_READ_SIZE);
			return(-3);
		}


		/* Convert the FITS block into table entries.
		fits_to_table returns the number of records converted	
		*/
		
		status = fits_to_table(fits, table, 
				mode, total_records - num_records_read);

		/*  Update number of records read */
		/*  Move table pointer along	*/
		num_records_read += status;
		/* You would think that the table pointer would have to 
		be moved along, but for our SPECIAL cases it doesn't.
		The short and region tables read the index first
		and then offset into the table based on the index.
		The large table only require one read, so no update
		required.	*/
		/*table += (status * table_entry_size); */
		/* ( I would have liked to do somthing slick
		with the ALL_TABLE_PTR union.  But this is 
		the way that it works out).	*/
	}

	/*  Should be done reading in the table information.	*/
	/*  Close up the file.	*/
	if (fclose(fp) != 0) printf("ERROR closing file in read_table\n");

	return(num_records_read);
}





/*  fits_to_table takes a block of FITS data and converts it into local
table format.  This basically involves scanning string data and turning
it into floating point or integer data.	fits_to_table returns
the number of records sucessfully converted into C table entries. */

/*  want_to_read is a hack.  The last valid records in a FITS file may 
only partially fill a FITS read block.  Now the caller has to prevent
blank data being interpreted as records by providing a limit as to
how many records will be read.  Note that the mode implies a number 
of records per full block.  So, the number read will be the minimum
of want_to_read and records_per_block.
*/
int
fits_to_table(fits, tbl_ptr, mode, want_to_read)
char	*fits;
void	*tbl_ptr;	/*  Insertion pointer for  local table	*/
int	mode;	/*  Method of conversion	*/
int	want_to_read;	/*  Number of records you would like to read.	*/
{
	int	i;	/*  Loop variable	*/
	int status;	
	float	hold;	/*  Holding float variable	*/
	int	temp;	/*  Temporary integer value	*/
	int	index;	/*  Temporary integer value	*/
	
	/*	Conversion holding variables */
	int	hours;
	int	minutes;
	double	seconds;
	int	degrees;
	double	dminutes;
	
	char	string[19];	/*  Holding string for the nasty
				end of line wrap arround.	*/

	/*  The FITS format pointers	*/
	LG_RECORD	*lr_ptr;	/*  Pointer to cast over fits read.  */
	SM_RECORD	*sr_ptr;	/*  Small record pointer.	*/
	REG_RECORD	*reg_ptr;	/*  Region record pointer.	*/
	GSC_RECORD	*gsc_ptr;	/*  GSC record pointer	*/
	SAO_RECORD	*sao_ptr;	/*  SAO record pointer	*/

	/*  The local table pointers	*/
	LARGE_TABLE	*lt_ptr;
	SMALL_TABLE	*st_ptr;
	REGION_TABLE	*rt_ptr;
	STAR		*star_ptr;
	SAO_ENTRY	*sao_file;	/*  This will actaully be a file pointer	*/

	int	converted_records=0;	/*  This will become the return value */
	int	records_per_block=1;   /*  Setting to one allows the
					default clause in the switch
					statement to alert the user.	*/
					

	/*  Figure the number of records to convert
	and cast required pointers.	*/
	if (mode == LARGE_MODE) {	
		records_per_block = FITS_READ_SIZE / sizeof(LG_RECORD);
		lr_ptr = (LG_RECORD *) fits;
		lt_ptr = (LARGE_TABLE *) tbl_ptr;
	}
		
	if (mode == SMALL_MODE)	{
		records_per_block = FITS_READ_SIZE / sizeof(SM_RECORD);
		sr_ptr = (SM_RECORD *) fits;
		st_ptr = (SMALL_TABLE *) tbl_ptr;
	}
	if (mode == REGION_MODE) {	
		records_per_block = FITS_READ_SIZE / sizeof(REG_RECORD);
		reg_ptr = (REG_RECORD *) fits;
		rt_ptr = (REGION_TABLE *) tbl_ptr;
	}
	if (mode == GSC_MODE) {
		records_per_block = FITS_READ_SIZE / sizeof(GSC_RECORD);
		gsc_ptr = (GSC_RECORD *) fits;
		star_ptr = (STAR *) tbl_ptr;
	}
	if (mode == SAO_MODE) {
		records_per_block = FITS_READ_SIZE / sizeof(SAO_RECORD);
		sao_ptr = (SAO_RECORD *) fits;
		sao_file = (SAO_ENTRY *) tbl_ptr;
	}


	/*  Loop and convert records	*/
	records_per_block = MIN(records_per_block, want_to_read);
	for(i=0; i< records_per_block; i++) {
	
	   /*  Do conversion on the ith record based on the mode	*/
	   switch(mode) {

	      case LARGE_MODE:
		/*  High and Low decs.	*/
		/*  The convention of the GSC is to make 
		DEC-High the larger magnitude Declination rather
		than the DEC that is greater than the other.
		This only matters when DEC is negative.	*/
		status = sscanf((lr_ptr+i)->dec_ctr, "%f", &hold);
		if (status != 1) return(converted_records);
		/*  Sucessfull conversion, now move into table	*/
		/*  Kepping in mind the above comment, we need
		to multiply by the sign of the declination.	*/
		(lt_ptr+i)->dec_low = (float) (hold - (hold/ABS(hold)) * HALF_BAND);
		(lt_ptr+i)->dec_high = (float) (hold + (hold/ABS(hold)) * HALF_BAND);

		/* Number of regions in field	*/
		status = sscanf((lr_ptr+i)->n_lg_reg, "%d", &temp);
		if (status != 1) return(converted_records);
		/*  Sucessfull conversion, now move into table	*/
		(lt_ptr+i)->num_regions = temp;
		
		/*  ID of first region	*/
		status = sscanf((lr_ptr+i)->f_lg_reg, "%d", &temp);
		if (status != 1) return(converted_records);
		/*  Sucessfull conversion, now move into table	*/
		(lt_ptr+i)->lg_reg_id = temp;

		/*  All finished with conversion, add one to the pile
		and come arround again.	*/
		converted_records++;
		break;	


	      case SMALL_MODE:
		/*  Number ID of large region
		(These should be sequential)	*/
		status = sscanf((sr_ptr+i)->lg_reg_n, "%d", &index);
		if (status != 1) return(converted_records);
		
		/*  First small region	*/
		status = sscanf((sr_ptr+i)->f_sm_reg, "%d", &temp);
		if (status != 1) return(converted_records);
		/*  Sucessfull conversion, now move into table	*/
		(st_ptr+index)->sm_reg_id = temp;

		/*  Depth of subdivision	*/
			/*  Depth is a one character field.	*/
		strncpy(string, (sr_ptr+i)->depth, 1);
		string[1] = 0;	/*  Null terminate	*/
	
		status = sscanf(string, "%d", &temp);
		if (status != 1) return(converted_records);
		/*  Sucessfull conversion, now move into table	*/
		(st_ptr+index)->subdivision = temp;

		/*  All finished with conversion, add one to the pile
		and come arround again.	*/
		converted_records++;
		break;	
	
	      case REGION_MODE:
		/*  Fill in the region mode.  Note that there will need 
		to be a conversions to dedimal degrees.	*/
		
		/*  My confidence is up and my memory is churning.
		From now on it is ato? rather than scanf.	*/

		/*  Region number	*/
		index = atoi((reg_ptr+i)->reg_no);
	
		/*  RA low limit	*/
		hours = atoi((reg_ptr+i)->ra_h_low);
		minutes = atoi((reg_ptr+i)->ra_m_low);
		seconds = atof((reg_ptr+i)->ra_s_low);
		/*  Convert to decimal degrees and load into table	*/
		(rt_ptr+index)->ra_low = (float) (HMS_TO_DD(hours, minutes, seconds));
		
		/*  RA High limit	*/
		hours = atoi((reg_ptr+i)->ra_h_hi);
		minutes = atoi((reg_ptr+i)->ra_m_hi);
		seconds = atof((reg_ptr+i)->ra_s_hi);
		/*  Convert to decimal degrees and load into table	*/
		(rt_ptr+index)->ra_high = (float) (HMS_TO_DD(hours, minutes, seconds));
	
		/*  DEC  low limit	*/
		degrees = atoi((reg_ptr+i)->dec_d_lo);
		if (degrees < 0) printf("Read in neg deg. On rec. %d\n",
			index);
		dminutes = atof((reg_ptr+i)->dec_m_lo);
		/*  Convert and load. Watch the sign !	*/
		(rt_ptr+index)->dec_low = (float) (DM_TO_DD(degrees, dminutes) *
				SIGN((reg_ptr+i)->decsi_lo[0]));
			
		/*  DEC  High limit	*/
		degrees = atoi((reg_ptr+i)->dec_d_hi);
		dminutes = atof((reg_ptr+i)->dec_m_hi);
		/*  Convert and load. Watch the sign !	*/
		(rt_ptr+index)->dec_high = (float) (DM_TO_DD(degrees, dminutes) *
				SIGN((reg_ptr+i)->decsi_hi[0]));
			
		
		/*  Finished the record, come arround for another	*/
		converted_records++;
		break;

	      case GSC_MODE:
		/*  There are no blanks in the star files, so each
		string must be copied onto a smaller holding string.  */

		/*  Star index	*/
		strncpy(string, (gsc_ptr+i)->gsc_id, 5);
		string[5] = 0;
                index = atoi(string);
                if (index < 0 || index >= MAX_STARS_PER_FILE) break;  // Happens in some of the AST discs.
		
		/*  RA	*/
		strncpy(string, (gsc_ptr+i)->ra_deg, 9);
		string[9] = 0;
		(star_ptr+index)->ra = (float) atof(string);
	
		/*  DEC	*/
		strncpy(string, (gsc_ptr+i)->dec_deg, 9);
		string[9] = 0;
		(star_ptr+index)->dec = (float) atof(string);

		/*  Magnitude */
		strncpy(string, (gsc_ptr+i)->mag, 5);
		string[5] = 0;
		(star_ptr+index)->magnitude = (float) atof(string);

		/*  Finished the record, come arround for another	*/
		converted_records++;
		break;

	      case SAO_MODE:
		/*  Write each of the fields of interest to a file	*/
		if ((sao_ptr+i)->type[1] == ' ') {
			/*  Be certain that there is alway something there */
			(sao_ptr+i)->type[0]='X';
			(sao_ptr+i)->type[1]='X';
			(sao_ptr+i)->type[2]='X';
		}
		fprintf(sao_file,
	 "%.6s %.4s %.4s %.2s %.2s %.6s  %.1s %.2s %.2s %.5s %.3s\n",
			(sao_ptr+i)->sao_num,
			(sao_ptr+i)->photo_mag,
			(sao_ptr+i)->V_mag,
			(sao_ptr+i)->ra_hrs,
			(sao_ptr+i)->ra_min,
			(sao_ptr+i)->ra_sec,
			(sao_ptr+i)->dec_sgn,
			(sao_ptr+i)->dec_deg,
			(sao_ptr+i)->dec_min,
			(sao_ptr+i)->dec_sec,
			(sao_ptr+i)->type);
		/*  That's it! No conversion is necessary	*/
		
		/*  Finished this record, come arround for another.	*/
		converted_records++;
		break;
		
	      default:
		printf("fits_to_table called with an unknown mode.\n");
		return(-5);
	   } /*  switch */
	 } /*  for	*/

	return(converted_records);	/*  Number of sucessful reads	*/

 } /*  End of function	*/
