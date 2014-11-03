/*
	bloblist.cpp

	Implementation of bloblist class

*/

#include "bloblist.h"

bloblist::bloblist( int flux_in, double x_in, double y_in ) {
  flux = flux_in;
  x = x_in;
  y = y_in;
  
  type = 0;
  
  nextblob = NULL;
  prevblob = NULL;
  
  snr = 1;
  mean = 0;
}

bloblist::~bloblist() {
}

// --- Set blob properties ----------------------------------------------

void bloblist::setflux( int flux_in ) {
	flux = flux_in;
}

void bloblist::setx( double x_in ) {
	x = x_in;
}

void bloblist::sety( double y_in ) {
	y = y_in;
}

void bloblist::settype( int type_in ) {
	if( (type_in >= 0) && (type_in <= 2) ) type = type_in;
}

void bloblist::setsnr( double snr_in ) {
	snr = snr_in;
}

void bloblist::setmean( double mean_in ) {
	mean = mean_in;
}

// --- Get blob properties ----------------------------------------------

int bloblist::getflux() {
	return flux;
}

double bloblist::getx() {
	return x;
}

double bloblist::gety() {
	return y;
}

int bloblist::gettype() {
	return type;
}

double bloblist::getsnr() {
	return snr;
}

double bloblist::getmean() {
	return mean;
}

// --- Linked list head/tail pointers -----------------------------------

void bloblist::setnextblob( bloblist *nextblob_in ) {
	nextblob = nextblob_in;
}

void bloblist::setprevblob( bloblist *prevblob_in ) {
	prevblob = prevblob_in;
}

bloblist *bloblist::getnextblob() {
	return nextblob;
}

bloblist *bloblist::getprevblob() {
	return prevblob;
}
