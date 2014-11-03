
//
// AnnularRegion
//
// Region used to find stars at a given distance away
//
// 

#ifndef ANNULAR_REGION
#define ANNULAR_REGION

#include "Region.h"
#include "Units.h"

class AnnularRegion : public MinorCircleRegion {
 public:
  AnnularRegion(const MinorCircleRegion& mcr, Radian width);
  virtual ~AnnularRegion();
  virtual RegionGS::Location location(const Point& p) const; 
  
 private:
  Radian width;
  MinorCircleRegion *inner;
};

#endif // ANNULAR_REGION
