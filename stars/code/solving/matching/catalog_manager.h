/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__MATCHING__CATALOG_MANAGER_H
#define SOLVING__MATCHING__CATALOG_MANAGER_H

#include <string>
#include "../../cataloging/abstract_catalog_manager.h"

namespace Parameters
{
    class Manager;
}

namespace Solving
{
    namespace Matching
    {

class CatalogManager: public Cataloging::AbstractCatalogManager
{
  public:
    CatalogManager(Parameters::Manager& params, bool enabled, std::string catalog_path_);
    bool abort();
};

    }
}

#endif
