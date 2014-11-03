
// 
// AnnularRegion.cpp
//
// Implementation of AnnularRegion

#include "AnnularRegion.h"

AnnularRegion::AnnularRegion(const MinorCircleRegion &mcr, Radian w) :
  MinorCircleRegion(mcr), width(w) {
  
  // Make a second MCR with a smaller radius
  inner = new MinorCircleRegion( getCenter(), getRadius() - w );
}

AnnularRegion::~AnnularRegion() {
  delete inner;
}

RegionGS::Location AnnularRegion::location(const Point& p) const {
  Location loc = MinorCircleRegion::location(p);
  Location inloc;
  switch (loc) {
  case Outside:
    return Outside;
    break;

  case Inside:
    inloc = inner->location(p);
  
    switch(inloc) {
    
    case Outside:
      return Inside;
      break;
    
    case Inside:
      return Outside;
      break;
   
    case Border:
      return Border;
      break;
    }
    break;
  
  case Border: 
    return Border;
    break;
  }
  
  throw Error; // Control should not reach here.
}

