//  Units.cpp
//
// Implementation for Units


#include <math.h>
#define M_PI 3.14159265358979323846

#include "Units.h"



// Conversion routines

Radian::Radian(const Degree &d) {
  val = (M_PI/180.) * double(d);
}

Radian::Radian(const Hour &d) {
  val = (M_PI/12.) * double(d);
}

Degree::Degree(const Radian &d) {
  val = (180./M_PI)*d;
}

Degree::Degree(const Hour &d) {
  val = 15*d;
}

Hour::Hour(const Radian &d) {
  val = (12./M_PI)*double(d);
}

Hour::Hour(const Degree &d) {
  val = d/15;
}

DMS::DMS(const Radian &d) {
  val = (180./M_PI)*d;
}

DMS::DMS(const Degree &d) {
  val = d;
}

QString Radian::asQString() const {
  return QString::number( asDouble() );
}

QString Degree::asQString() const {
  return QString::number( asDouble() );
}

QString Hour::asQString() const {
  QString s;
  
  int hour( val );
  if (hour < 10) s = "0"; // Show leading zero
  s += QString::number( hour ) +  ":";
  
  int minute = int( 60*(val - hour) );
  if (minute < 10) s+= "0";
  s += QString::number( minute ) +  ":";
  
  double second = 3600*(val - hour - minute/60.);
  if (second < 10) s+= "0";
  s += QString::number( second );
  
  return s;
}

QString DMS::asQString() const {
  QString s;
  
  int hour(val);
  if (hour < 10 && hour > 0) s = "0"; // Show leading zero
  s += QString::number( hour ) +  ":";
  
  int minute = int( 60*(val - hour) );
  if (minute < 10) s+= "0";
  s += QString::number( minute ) +  ":";
  
  double second = 3600*(val - hour - minute/60.);
  if (second < 10) s+= "0";
  s += QString::number( second );
  
  return s;
}


// Nice ways to print

std::ostream& operator<<(std::ostream& os, const Radian& r) {
  os << ( r.asDouble() ) << " rad";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Degree& r) {
  os << r.asDouble() << " deg";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Hour& r) {
  int hour = int(r);
  if (hour < 10) os << 0; // Show leading zero
  os << hour << ":";
  
  int minute = int( 60*(r - hour) );
  if (minute < 10) os << 0;
  os << minute << ":";
  
  double second = 3600*(r - hour - minute/60.);
  if (second < 10) os << 0;
  os << second;
  
  return os;
}

std::ostream& operator<<(std::ostream& os, const DMS& r) {
  int hour = int(r);
  if (hour < 10) os << 0; // Show leading zero
  os << hour << ":";
  
  int minute = int( 60*(r - hour) );
  if (minute < 10) os << 0;
  os << minute << ":";
  
  double second = 3600*(r - hour - minute/60.);
  if (second < 10) os << 0;
  os << second;
  
  return os;
}


