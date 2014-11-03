//
//  Star.cpp
//
 // Implementation for Star

#include <math.h>
#define M_PI 3.14159265358979323846

#include "Star.h"

Star::Star() : Point(0, 0, 1) {}

Star::Star(const Radian& r, const Radian &d, double m) : 
  Point(0, 0, 1), ra(r), dec(d), mag(m) {
  init();
}

Star::Star(const Degree& r, const Degree &d, double m) : 
  Point(0, 0, 1), ra(r), dec(d), mag(m) {
  init();
}

Star::Star(const Hour& r, const Degree &d, double m) : 
  Point(0, 0, 1), ra(r), dec(d), mag(m) {
  init();
}

Star::Star(const Point& p, double m) : 
  Point( p ), mag(m) {
  xyzToRaDec();
}

void Star::init() {
  RaDecToxyz();
  xyzToRaDec();  // Returns ra/dec to correct range
}

Star::~Star() {}

Hour Star::getRa(const Hour &) const {
  return Hour( ra );
}

Degree Star::getRa(const Degree &) const {
  return Degree( ra );
}

Radian Star::getRa(const Radian &) const {
  return Radian( ra );
}

Radian Star::getDec(const Radian &) const {
  return Radian( dec );
}

Degree Star::getDec(const Degree &) const {
  return Degree( dec );
}

DMS Star::getDec(const DMS &) const {
  return DMS( dec );
}

double Star::getMagnitude() const {
  return mag;
}

double Star::setX(double a) {
  Point::setX(a);
  xyzToRaDec();
  return x;
}

double Star::setY(double a) {
  Point::setY(a);
  xyzToRaDec();
  return y;
}

double Star::setZ(double a) {
  Point::setZ(a);
  xyzToRaDec();
  return z;
}

double Star::setRa(double r) {
  ra = r;
  RaDecToxyz();
  xyzToRaDec();  // Returns ra/dec to correct range
  return ra;
}

double Star::setDec(double d) {
  dec = d;
  RaDecToxyz();
  xyzToRaDec();  // Returns ra/dec to correct range
  return dec;
}

double Star::setMagnitude(double m) {
  mag = m;
  return mag;
}

void Star::xyzToRaDec() {
  ra = atan2(y, x);
  ra = fmod(ra + Radian(2*M_PI), 2*M_PI);
  
  dec = atan2( z, sqrt(x*x + y*y) );
}

void Star::RaDecToxyz() {
  x = cos(ra) * cos(dec);
  y = sin(ra) * cos(dec);
  z = sin(dec);
}

QString Star::asQString() const {
  return getRa(Hour()).asQString() + ", " + 
    getDec(DMS()).asQString() + ", " + 
    QString::number( getMagnitude() );
}

// Make this comparison more physically based.  Two stars are
// considered identical if they are within 0.5" of each other and they
// are within one magnitude of each other.
bool Star::operator==(const Star& s) const {

  if (fabs(getMagnitude() - s.getMagnitude()) > 1) return false;
  double sepDeg = 180./M_PI* acos( dot(s) );
  if (sepDeg > 0.5/3600) return false;
  return true;
  
  //return ra==s.ra && dec==s.dec && mag==s.mag;
}


std::ostream &operator<<(std::ostream& output, const Star& s) {
  return output << "{" << s.getRa(Hour()) << ", " << s.getDec(DMS()) << 
    ", " << s.getMagnitude() << "}";
}

