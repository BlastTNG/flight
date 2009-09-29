//
// Units.h
//
// Definitiions for the star class
//
// A Star is a point, with the addition of a magnitude and the 
// Ra and Dec properties that are available via set/get mechanism.
//

#ifndef UNITS_H
#define UNITS_H

#include <iostream>  // Learn how double is printed.

#include <qstring.h>

// Helper classes to ensure that coordinates are understood
class Radian;
class Degree;
class Hour;

class Radian {
 public:
  Radian(double d=0) : val(d) {}
    Radian(const Degree &d);
    Radian(const Hour &d);
    operator double() const {return val;}
    Radian operator+(const Radian& r2) { return Radian(val+double(r2));} 
    Radian operator-(const Radian& r2) { return Radian(val-double(r2));} 
    double asDouble() const {return val;}
    QString asQString() const;
 private:
    double val;
};


class Degree {
 public:
  Degree(int d, int m, double s=0) : val(d + m/60.+ s/3600.) {}
    Degree(double d=0) : val(d) {}
      Degree(const Radian &d);
      Degree(const Hour &d);
      operator double() const {return val;}
      double asDouble() const {return val;}
      QString asQString() const;
 private:
      double val;
};

class Hour {
 public:
  Hour(int h, int m, double s=0) : val(h + m/60. + s/3600.) {}
    Hour(double d=0) : val(d) {}
      Hour(const Radian &d);
      Hour(const Degree &d);
      operator double() const {return val;}
      double asDouble() const {return val;}
      QString asQString() const;
 private:
      double val;
};
class DMS {
 public:
  DMS(double d=0) : val(d) {}
    DMS(const Radian &d);
    DMS(const Degree &d);
    operator double() const {return val;}
    double asDouble() const {return val;}
    QString asQString() const;
 private:
    double val;
};

// Nice ways to print

std::ostream& operator<<(std::ostream& os, const Radian& r);
std::ostream& operator<<(std::ostream& os, const Degree& r);
std::ostream& operator<<(std::ostream& os, const Hour& r);
std::ostream& operator<<(std::ostream& os, const DMS& r);

#endif // UNITS_H
