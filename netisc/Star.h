//
// Star.h
//
// Definitiions for the star class
//
// A Star is a point, with the addition of a magnitude and the 
// Ra and Dec properties that are available via set/get mechanism.
//

#ifndef STAR_H
#define STAR_H

#include <iostream>
#include <math.h>

#include <qstring.h>

#include "Point.h"
#include "Units.h"


class Star : public Point {
public:
  Star(); // For arrays
  Star(const Radian &ra, const Radian &dec, double mag=0);
  Star(const Degree &ra, const Degree &dec, double mag=0);
  Star(const Hour &ra, const Degree &dec, double mag=0);
  Star(const Point& p, double mag=0);
  ~Star();
  
  // The variable passed is a dummary variable only used to give type info.
  // It is better to use a NULL pointer cast?  
  Radian getRa(const Radian&) const;
  Degree getRa(const Degree&) const;
  Hour getRa(const Hour&) const;
  
  Radian getDec(const Radian&) const;
  Degree getDec(const Degree&) const;
  DMS getDec(const DMS&) const;
  
  double getMagnitude() const;
  
  // These are inherited from Point
  //double getX() const;
  //double getY() const;
  //double getZ() const;
  
  double setX(double x);
  double setY(double y);
  double setZ(double z);
  double setRa(double ra);
  double setDec(double dec);
  double setMagnitude(double mag);
  
  QString asQString() const;
  bool operator==(const Star&) const;
  
  
 private:
  Radian ra, dec;
  double mag;
  void xyzToRaDec();
  void RaDecToxyz();
  void init();
};

std::ostream &operator<<(std::ostream &output, const Star& s);

#endif
