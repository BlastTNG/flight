/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "catalog_manager.h"
#include "../update.h"
#include "../logger.h"

using namespace Solving::Matching;

CatalogManager::CatalogManager(Parameters::Manager& params, bool enabled, std::string catalog_path_):
    Cataloging::AbstractCatalogManager(params, enabled, catalog_path_, Solving::logger)
{
}

bool CatalogManager::abort()
{
    return Solving::done();
}

