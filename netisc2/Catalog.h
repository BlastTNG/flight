//
//  Catalog.h
//
//  Interface class for access to the star catalogs.


#ifndef CATALOG_H
#define CATALOG_H

#include <list>
#include "Star.h"
#include "Region.h"

class Catalog {
public:  
  virtual void loadFromFile(const char *filename) = 0;
  virtual void sync() = 0;
  virtual bool add(const Star& s) = 0;        // Add a single Star
  //virtual void add(const & std::list<Star> ls);  // Add a list of stars
  virtual std::list<Star> find(const RegionGS& region) = 0; // Returns a list of Stars in the given Region, satisfying the given magLimit
  
protected:
};

#endif
