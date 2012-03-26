//
// Frame.cpp
// Implementation of Frames

#include "math.h"

#include "Frame.h"

using namespace std;

Frame::Frame() :
  primaryBlob(0), secondaryBlob(0), primaryPoint(0), secondaryPoint(0),
  scale(1) {
}

Frame::Frame(const Frame& f) :
  primaryBlob(0), secondaryBlob(0), primaryPoint(0), secondaryPoint(0),
  scale(f.scale)  {

  //printf( "Copying frame" );
  if (f.primaryBlob) primaryBlob = new Blob(*(f.primaryBlob));
  if (f.secondaryBlob) secondaryBlob = new Blob(*(f.secondaryBlob));
  if (f.primaryPoint) primaryPoint = new Point(*(f.primaryPoint));
  if (f.secondaryPoint) secondaryPoint = new Point(*(f.secondaryPoint));
  blob = f.blob;  // This should be a shallow copy.
}

Frame::~Frame() {
  //printf("Deleating frame");
  delete primaryBlob;
  delete secondaryBlob;
  delete primaryPoint;
  delete secondaryPoint;
}

Frame& Frame::operator=(const Frame& rhs) {
  printf("Assigning frame");
  if (rhs.primaryBlob) primaryBlob = new Blob(*(rhs.primaryBlob));
  if (rhs.secondaryBlob) secondaryBlob = new Blob(*(rhs.secondaryBlob));
  if (rhs.primaryPoint) primaryPoint = new Point(*(rhs.primaryPoint));
  if (rhs.secondaryPoint) secondaryPoint = new Point(*(rhs.secondaryPoint));
  
  blob = rhs.blob; // This should be a shallow copy.
  return *this;
}

// Convert flux to magnitude to avoid later conversion inefficiency
void Frame::addBlob( const Degree& x, const Degree& y, double flux) {
  if (flux < 0) flux = 1; // Keep log from crashing.
  double mag = 17 - 2.5*log10( flux );
  
  printf( "Adding Blob( %lg, %lg)", x, y);
  blob.push_back( Blob(x, y, mag) );
}

void Frame::addPrimaryBlob( const Degree& x, const Degree& y, double flux) {
  if (flux < 0) flux = 1; // Keep log from crashing.
  double mag = 17 - 2.5*log10( flux );
  primaryBlob = new Blob(x, y, mag);
}

void Frame::addSecondaryBlob( const Degree& x, const Degree& y, double flux) {
  if (flux < 0) flux = 1; // Keep log from crashing.
  double mag = 17 - 2.5*log10( flux );
  secondaryBlob = new Blob(x, y, mag);
}

void Frame::clear() {
  blob.clear();
}


Radian Frame::getAxisLength() {
  // This is the length from the primaryBlob to the secondaryBlob
  //Q_ASSERT( primaryBlob );
  //Q_ASSERT( secondaryBlob );
  
  // This is not valid for a distorted field or an extended field.
  return Radian( scale*acos( primaryBlob->dot(*secondaryBlob) ) );
}

Point Frame::getPrimary() const {
  return *primaryPoint;
}

Point Frame::setPrimary(const Point& p1) {
  delete primaryPoint;
  primaryPoint = new Point(p1);
  return *primaryPoint;
}

Point Frame::getSecondary() const {
  return *secondaryPoint;
}

Point Frame::setSecondary(const Point& s1) {
  delete secondaryPoint;
  secondaryPoint = new Point(s1);
  return *secondaryPoint;
}

std::list<Star> Frame::getStars() {
  //Q_ASSERT( primaryBlob );
  //Q_ASSERT( secondaryBlob );
  //Q_ASSERT( primaryPoint );
  //Q_ASSERT( secondaryPoint );

	std::list<Star> ret;

  // Setup the axes.
  Point a( *primaryBlob );
  Point c( a.cross(*secondaryBlob) );
  Point b( c.cross(a) );

  Point x( *primaryPoint );
  Point z( x.cross(*secondaryPoint) );
  Point y( z.cross(x) );
  
  // Loop over all of the Blobs
  std::list<Blob>::iterator it;
  for (it = blob.begin(); it != blob.end(); it++) {
    Point& u = (*it);
    Point p = x*a.dot(u) + y*b.dot(u) + z*c.dot(u); 
    ret.push_back( Star(p, (*it).mag) );
  }
  
  return ret;
}

std::ostream &operator<<(std::ostream &output, Frame& f) {
  output << "Frame with " << f.count() << " blobs\n";
  std::list<Star> s = f.getStars();
  std::list<Star>::iterator it;
  for (it = s.begin(); it != s.end(); it++) {
  //  output << (*it) << endl;
	  cout << (*it) << endl;
  }
  return output;
}
