#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

#include <termios.h>
#include <ctype.h>

#include <math.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>  
#include <sys/poll.h>  
#include <sys/wait.h> 
#include <sys/errno.h>
#include <sys/signal.h>

#ifdef linux
#  include <linux/limits.h>	/* PATH_MAX */
#  include <asm/page.h>		/* PAGE_SIZE */
#endif

//local headers
#include "itifgExt.h"
#include "pointing_struct.h"
#include "sort.h"
#include "ephem_read.h"
#include "starpos.h"
#ifndef TYPES_DEFINED
#include "ephem_types.h"
#endif
#define MAX_LENGTH 64
#define EPHEM_FILE "./ephemeris/ephem.2000"
#define OFILE "starlist.dat"  //list of up to mag 6 stars
#define MAX_STAR 5060         //total number of stars in file

#define ARCSEC_PER_ROW 3.789 //boomerang starcam numbers
#define ARCSEC_PER_COL 4.002 //must be recalibrated for blastcam

#define BOX 6 //size of box average filter used to make tiny;larger number = faster algorithm 
#define NO_SIGMA 5.0 //threshold used in locate_blob; smaller number = risk of
                     //finding noise, larger number = risk of not finding star;
                     //depends on camera gain set in ~cmactavi/programs/cam_command/sc_com
#define STARBOX 10 //used in locate_star(); sum all pixels (mean subtracted)  in STARBOX by STARBOX;
                   //stars usually about 5 by 5; 
#define ERRORBOX 0.0005 //error in radians (~1000 arcsec) used in GetStar(); find
                        //ra, dec within ERRORBOX

struct PointingDataStruct localPointingData;

//frame grabber variables
int read_config (const char*,       /* camera config file name */
             union iti_dev_t*,  /* camera structure to fill in */
             int,               /* board  \               */
             int,               /* module - to search for */
             int,               /* camera /               */
             char*,             /* camera name */
             char*);            /* fpga-file name */
static struct image_t{
  caddr_t srcptr;
  off_t srcoff;
  size_t paged_size;
  short width, height;
  double mean, std_dev;
} image ={
  NULL, 0, 0, 0, 0, 0, 0 
};
int WIDTH, HEIGHT;
char config_file[MAX_LENGTH] = "cameras/jai-cv_m10-ni-pcv.cam";
static volatile int timeout = FALSE;
static int devdesc = -1;
static caddr_t dataptr = NULL;
static size_t mapsize = 0UL;
unsigned char* global_image;
int board_no = 0;
int module_no = 16; //PCVision module number
int camera_no = 0;
float set_rate = 1.0;

//global structure containing star position data
struct VSCDataStruct VSCData[3];
int vsc_index = 0;

//image processing variables
unsigned char* tiny;
unsigned char* median;
unsigned char* binary;
int tiny_height, tiny_width;
double std_dev = 0;
double mean = 0;
int star_found = 0;

//variable that contains star position data in tiny image
int row, column;

//variable that contains star position data in original image
double row_i, column_j;


/********************************************************************************/
/*initializes frame grabber device                                              */
/********************************************************************************/
void Init_Devs (){

  int flags, error;
  union iti_dev_t camconf;
  int board, camera;
  char camera_name[MAX_LENGTH], exo_filename[MAX_LENGTH]; 
  struct timeval timeout;
  time_t timeout_ms;
  char device_name[PATH_MAX];
  
  snprintf (device_name, PATH_MAX, "%s%d%s", ITI_NAME_PREFIX,
            board_no, ITI_NAME_DATA_POSTFIX);
  
  flags = O_RDONLY;
  
  if ((devdesc = open (device_name, flags)) < 0){
    perror ("Open module device");
    exit(1);
  }
 
  timeout_ms = 1000/set_rate;
  MS_TO_TV (timeout_ms, timeout); 
  if (ioctl (devdesc, GIOC_GE_SET_TIMEOUT, &timeout) < 0){
    perror ("GIOC_GE_SET_TIMEOUT");
    exit(1);
  }
      
  if (ioctl (devdesc, GIOC_GE_SET_CAMERA, &camera_no) < 0){
    perror ("GIOC_GE_SET_CAMERA");
    exit(1);
  }
  
  if (ioctl (devdesc, GIOC_GE_GET_CAMCNF, &camconf) < 0){
    perror ("GIOC_GE_GET_CAMCNF");
    exit(1);
  }
  
  board = 0;
  camera = 0;
  if ((error = read_config ( config_file, &camconf,
                             board, 16,
                             camera, camera_name, exo_filename)) < 0){
    fprintf (stderr, "read_config:  Error reading config file (%s).\n",
             config_file); 
    exit (EXIT_FAILURE);
  }
  
  fprintf (stderr, "CameraName : %s.\n", camera_name);
  
  if (ioctl (devdesc, GIOC_GE_SET_CAMCNF, &camconf) < 0){
    perror ("GIOC_GE_SET_CAMCNF");
    exit(1);
  } 

}

/********************************************************************************/
/*setup image buffer                                                            */
/********************************************************************************/
void Setup_Bufs (){

  if (ioctl (devdesc, GIOC_GE_GET_WIDTH, &image.width) < 0){
    perror ("GIOC_GE_GET_WIDTH");
    exit(1);
  }

  WIDTH = image.width;
  
  if (ioctl (devdesc, GIOC_GE_GET_HEIGHT, &image.height) < 0){
    perror ("GIOC_GE_GET_HEIGHT");
    exit(1);
  }

  HEIGHT = image.height;
  
  if (ioctl (devdesc, GIOC_GE_GET_PAGEDSIZE, &image.paged_size) < 0){
    perror ("GIOC_GE_GET_PAGEDSIZE");
    exit(1);
  }
  
  fprintf (stderr, "\n");
  fprintf (stderr, "Image width: %d.\n", image.width);
  fprintf (stderr, "Image height: %d.\n", image.height);

  mapsize = image.paged_size; 
  if ((dataptr = (void*)mmap (0, mapsize, PROT_READ, 
                              MAP_SHARED, devdesc, 0)) == (void*)-1){
    perror ("Can't map camera device");
    exit(1);
  }
  
  image.srcptr = dataptr;
  image.srcoff = 0;
  fprintf (stderr, "Source pointer: %p.\n", image.srcptr); 
  fprintf (stderr, "Source offset: %ld.\n", (long)image.srcoff);

}

/********************************************************************************/
/*get mean value of part of image                                               */
/********************************************************************************/
void get_mean(){

  int i, j;
  double n = 0.0009765625;     /* look at only 1024 pixels */
  double sum_x = 0;	       
  double sum_x2 = 0;
  
  /* look at a small box of pixels and keep running tab of mean, std_dev */

  for (i = 150; i < 182; i++) {
    for (j = 150; j < 182; j++) {
      sum_x2 += (global_image[i*WIDTH+j]*global_image[i*WIDTH+j]);
      sum_x += global_image[i*WIDTH+j];      
    }
  }
  mean = sum_x*n;
}

/********************************************************************************/
/*takes block average of image with BOX X BOX filter; creates tiny              */
/********************************************************************************/
void make_tiny(){ 
  
  int i, j, p;
  double value = 0, box_sqrd;
  unsigned char *tgi;
  int i_plus_box, j_plus_box, i_div_box_times_width;
  
  tiny_width = (int)((float)WIDTH/(float)BOX);
  tiny_height = (int)((float)HEIGHT/(float)BOX);
  box_sqrd = (double)BOX*(double)BOX;
  tiny = (unsigned char*)malloc(mapsize/(BOX*BOX));

  //loop over all pixels in image; calculate the average pixel value in BOX*BOX;
  //replace value in upper, left pixel with average value
  for (i = 0; i <= (HEIGHT-BOX); i = i + BOX) {
    i_plus_box = i+BOX;
    i_div_box_times_width = i/BOX*tiny_width;
    for (j = 0; j <= (WIDTH-BOX); j = j + BOX) {
      j_plus_box = j+BOX;
      tgi = global_image + i*WIDTH; 
      for (p = i; p < i_plus_box; p++) {
        value+=(tgi[j]+tgi[j+1]+tgi[j+2]+tgi[j+3]+tgi[j+4]+tgi[j+5]);
        //for (q = j; q < j_plus_box; q++) {
        //  value += tgi[q];                 
        // }
        tgi += WIDTH;
      }
      value = value/(box_sqrd); 
      tiny[i_div_box_times_width + j/BOX] = (long)(value+0.5);
      value = 0;
    }
  }
   
}

/********************************************************************************/
/*insertion sort used to sort arrays in median()                                */
/********************************************************************************/
void  insort (array, len)
register KEY_T  array[];
register int    len;
{
	register int	i, j;
	register KEY_T	temp;

	for (i = 1; i < len; i++) {
		/* invariant:  array[0..i-1] is sorted */
		j = i;
		/* customization bug: SWAP is not used here */
		temp = array[j];
		while (j > 0 && GT(array[j-1], temp)) {
			array[j] = array[j-1];
			j--;
		}
		array[j] = temp;
	}
}

/********************************************************************************/
/*calls insort which sorts array; returns median value                          */
/********************************************************************************/
float qsort_median(int * array, int n){
  float mod = 0;
 
  insort(array, n);
  
  mod = n%2;
  if (mod == 0){
    return (((array[n/2] + array[n/2-1])/2)+0.5);
  } else {
    return array[n/2];
  }
}

/********************************************************************************/
/*creates median filtered tiny;                                                  */
/********************************************************************************/
void make_median(){

  int i, j, p, q, k;
  int middle[9];
  int corner[4];
  int edge[6];
  int value, p_times_tiny_width;

  median = (unsigned char*)malloc(mapsize/(BOX*BOX));

//loop over all pixels in tiny image; replace each pixel with the median value of
//all bordering pixels (center pixel included)  
  
  k=0;
  for (i = 1; i < tiny_height - 1; i++) {
    for (j = 1; j < tiny_width - 1; j++) {
      for (p = i - 1; p <= i + 1; p++) {
        p_times_tiny_width = p*tiny_width;
        for (q = j - 1; q <= j + 1; q++) {
          middle[k] = tiny[p_times_tiny_width +q];
          k++;
        }
      }
      k = 0;
      value = qsort_median(middle, 9);
      median[i*tiny_width + j] = value;
    }
  }
 
//find median values for corner pixels
  //top-left
  i = 0;
  j = 0;
  k = 0;
  for (p = i; p <= i + 1; p++) {
    p_times_tiny_width = p*tiny_width;
    for (q = j; q <= j + 1; q++) {
      corner[k] = tiny[p_times_tiny_width +q];
      k++;
    }
  }

  k = 0;
  value = qsort_median(corner, 4);
  median[i*tiny_width + j] = value;
  //top-right
  i = 0;
  j = tiny_width - 1;
  for (p = i; p <= i + 1; p++) {
    p_times_tiny_width = p*tiny_width;
    for (q = j-1; q <= j; q++) {
      corner[k] = tiny[p_times_tiny_width +q];
      k++;
    }
  }
  k = 0;
  value = qsort_median(corner, 4);
  median[i*tiny_width + j] = value;
  //bottom-left
  i = tiny_height-1;
  j = 0;
  for (p = i-1; p <= i; p++) {
    p_times_tiny_width = p*tiny_width;
    for (q = j; q <= j + 1; q++) {
      corner[k] = tiny[p_times_tiny_width +q];
      k++;
    }
  }
  k = 0;
  value = qsort_median(corner, 4);
  median[i*tiny_width + j] = value;
  //bottom-right
  i = tiny_height-1;
  j = tiny_width-1;
  for (p = i-1; p <= i; p++) {
    for (q = j-1; q <= j; q++) {
      corner[k] = tiny[p_times_tiny_width +q];
      k++;
    }
  }
  k = 0;
  value = qsort_median(corner, 4);
  median[i*tiny_width + j] = value;
  
//find median values for edge pixels
  //top edge
  i = 0;
  for (j = 1; j < tiny_width - 1; j++) {
    for (p = i; p <= i + 1; p++) {
      p_times_tiny_width = p*tiny_width;
      for (q = j - 1; q <= j + 1; q++) {
        edge[k] = tiny[p_times_tiny_width +q];
        k++;
      }
    }
    k = 0;
    value = qsort_median(edge, 6);
    median[i*tiny_width + j] = value;
  }

  //bottom edge
  i = tiny_height-1;
  for (j = 1; j < tiny_width - 1; j++) {
    for (p = i - 1; p <= i; p++) {
      p_times_tiny_width = p*tiny_width;
      for (q = j - 1; q <= j + 1; q++) {
        edge[k] = tiny[p_times_tiny_width +q];
        k++;
      }
    }
    k = 0;
    value = qsort_median(edge, 6);
    median[i*tiny_width + j] = value;
  }
  //left edge
  j = 0;
  for (i = 1; i < tiny_height - 1; i++){
    for (p = i - 1; p <= i + 1; p++) {
      p_times_tiny_width = p*tiny_width;
      for (q = j; q <= j + 1; q++) {
        edge[k] = tiny[p_times_tiny_width +q];
        k++;
      }
    }
    k = 0;
    value = qsort_median(edge, 6);
    median[i*tiny_width + j] = value;
  }
  //right edge
  j = tiny_width-1;
  for (i = 1; i < tiny_height - 1; i++){
    for (p = i - 1; p <= i + 1; p++) {
      p_times_tiny_width = p*tiny_width;
      for (q = j - 1; q <= j; q++) {
        edge[k] = tiny[p_times_tiny_width +q];
        k++;
      }
    }
    k = 0;
    value = qsort_median(edge, 6);
    median[i*tiny_width + j] = value;
  }
  
}

/********************************************************************************/
/*finds the standard deviation of median                                        */
/********************************************************************************/
void get_std_dev(){

  int i, j;
  double sum_x = 0;	     
  double sum_x2 = 0;
  double n = 1.0/((float)tiny_width*(float)tiny_height);
  
  for (i = 0; i < tiny_height; i++) {
    for (j = 0; j < tiny_width; j++) {
      sum_x2 += (median[i*tiny_width+j]*median[i*tiny_width+j]);
      sum_x += median[i*tiny_width+j];   
    }
  }

  std_dev = sqrt((sum_x2- sum_x*sum_x*n)*n);
   
}

/********************************************************************************/
/*finds brightest pixel in tiny                                                 */
/********************************************************************************/
void locate_blob(){ 
  
  int i, j;
  double diff = 0, last_diff = 0, this_diff;
  
  binary = (unsigned char*)malloc(mapsize/(BOX*BOX));
  
  for (i = 0; i < tiny_height; i++) {
    for (j = 0; j < tiny_width; j++) {

      //if pixel is greater that num*std_dev then it qualifies as a star
      if (tiny[i*tiny_width+j] >= (median[i*tiny_width+j] + NO_SIGMA*std_dev) ){  
        row = i*BOX;
        column = j*BOX;
        this_diff =  (tiny[i*tiny_width+j] - median[i*tiny_width+j]);  
        if ( this_diff > last_diff){
          diff = this_diff;
          row = i*BOX;
          column = j*BOX;
          star_found = 1;
        }
        
        last_diff = this_diff;        
      }
      
    }
  }    
}
      
/********************************************************************************/
/*subtracts mean from image; locate star column and row                         */
/********************************************************************************/

void locate_star(){
  
  long data;
  int i, j;
  double sum_pix_times_i = 0, sum_pix_times_j = 0;
  double sum_pix = 0;
  double mag;

  for (i = row - STARBOX; i < row + STARBOX; i++) {
      for (j = column - STARBOX; j < column + STARBOX; j++) {

        //subtract mean from each pixel within STARBOX
        data = (int)(global_image[i*WIDTH+j] - mean);
        if  ( data < 0){
          data = 0;          
        }

        //sum all pixels,pix*x, pix*y
        sum_pix_times_i += (data*i);
        sum_pix_times_j += (data*j);
        sum_pix += data;
      }
    }

  //find centroid and set magnitude (sum of all pixels in STARBOX)
    row_i = sum_pix_times_i/sum_pix;
    column_j = sum_pix_times_j/sum_pix;
    mag = sum_pix;
    //printf("%f %f %f\n", row_i, column_j, mag);
    
    sum_pix = 0;
    sum_pix_times_j = 0;
    sum_pix_times_i = 0;

    //copy info into vsc buffer
    VSCData[vsc_index].col = column_j;
    VSCData[vsc_index].row = row_i;
    VSCData[vsc_index].mag = mag;
    VSCData[vsc_index].sf_frame = localPointingData.mcp_frame;
  }

/*****************************************************************/
/* main program                                                  */
/*****************************************************************/
void starfind(){

  int error, frame = 0;
  struct pollfd wait_fd;
  int wait_timeout;
  int sys_check = -1;
  
  //initialise camera settings
  while (sys_check < 0){
    sys_check = system("/home/cmactavi/programs/cam_command/sc_com");
  }

  fprintf(stderr, "Starfind startup.\n");

  //initialise frame grabber device
  Init_Devs ();

  //setup frame grabber buffers
  Setup_Bufs();

  //allocate memory for image
  global_image = (unsigned char*)malloc(mapsize);
  frame = 0;    

  //the infinite loop
  while(1){
     
    fflush(stdout);

    //timeing stuff required by poll
    timeout = FALSE;    
    wait_fd.fd = devdesc;
    wait_fd.events = POLLIN | POLLPRI;
    wait_timeout = (1 /set_rate) * 1000;

    //get current frame num and pointing info
    //just before grabbing frame
    //copy all pointing data into "local global"
    localPointingData = PointingData[GETREADINDEX(point_index)];

    //grab frame
    if (ioctl (devdesc, GIOC_CR_SET_START, NULL) < 0){
      perror ("GIOC_CR_SET_START");
      exit(1);
    }
    error = poll (&wait_fd, 1, wait_timeout);
    if (ioctl (devdesc, GIOC_CR_SET_STOP, NULL) < 0){
      perror ("GIOC_CR_SET_STOP");
      exit(1);
    }
    
    //check value error returned by poll
    switch (error){
    case -1:
      perror ("Some error during poll occured");
      exit(1);
      break;
    case 0:
      timeout = TRUE;
      break;
    case 1:
      if (wait_fd.revents & POLLPRI)
        timeout = TRUE;
      break;
    default:
      perror ("unexpected poll return value");
      exit(1);
    }
    
    // process image
    if ((image.srcoff = lseek (devdesc, 0UL, SEEK_CUR)) < 0){
      if (errno != EINTR){
        perror ("lseek 0/SEEK_CUR");
        exit(1);
      }
    }
    memcpy(global_image, image.srcptr, mapsize);

    //get mean from portion of image
    get_mean();

    //make block averaged image
    make_tiny();

    //make median of tiny
    make_median();

    //get std devtaion of median
    get_std_dev();

    //find brightest blob in tiny image 
    locate_blob();

    if (star_found){

      //locate corresponding blob (star) pixel x&y in original image
      //find centroid of star; copy info back to mcp
      locate_star();
            
      vsc_index = INC_INDEX(vsc_index);
    }
        
    image.srcoff = 0;
    frame++;
  }  
}
