#include "loadframe.h"

/*  Load the dimensions + data part of a QCam_Frame from a binary file
    Returns # elements written from final fread of buffer data
    
    unsigned short bits    (# of bits in image)
    unsigned long width    width in pixels
    unsigned long height   height in pixels
    char *buffer           image buffer
    
    If allocate = 1, loadframe allocates correct ammount of memory
    for the image buffer. Otherwise assumes it has already been
    done externally
*/

int loadframe( char *filename, QCam_Frame *frame, int allocate ) {
  FILE *infile;
  int nread;
  unsigned pixelsize;        
  
  infile = fopen( filename, "rb" );
  
  if( infile == NULL ) return NULL;
  
  nread = fread(&(frame->bits), sizeof(unsigned short), 1, infile);
  nread = fread(&(frame->width), sizeof(unsigned long), 1, infile);
  nread = fread(&(frame->height), sizeof(unsigned long), 1, infile);
  
  // Decide if we are using 1 or 2 bytes per pixel
  
  if( frame->bits > 8 ) {
    frame->format = qfmtMono16;
    pixelsize = 2;
  } else {
    frame->format = qfmtMono8;
    pixelsize = 1;
  }
        
  frame->bufferSize = frame->width * frame->height * pixelsize;
  
  if( allocate == 1 ) {
    printf("Here\n");
    frame->pBuffer = new unsigned char[frame->bufferSize];
  }

  nread = fread(frame->pBuffer,1,frame->bufferSize,infile);
  
  fclose(infile);
  
  
  return nread;
}
