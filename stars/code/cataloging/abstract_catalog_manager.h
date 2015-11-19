/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef CATALOGING__ABSTRACT_CATALOG_MANAGER_H
#define CATALOGING__ABSTRACT_CATALOG_MANAGER_H

#include <vector>
#include <array>
#include <boost/filesystem/fstream.hpp>
#include "../solving/solution.h"
#include "../solving/star.h"
#include "../solving/blob.h"
#include "../shared/solving/filters.h"
#include "../shared/solving/settings.h"

namespace Parameters
{
    class Manager;
}

namespace Cataloging
{
    class Region;
    class AbstractCatalogManager;
    namespace fs = boost::filesystem;
}

namespace Logging
{
    class Logger;
}

class Cataloging::Region
{
  public:
    Region();
    std::vector<Solving::Star> stars;
    double ra, dec;
};

class Cataloging::AbstractCatalogManager
{
  public:
    AbstractCatalogManager(Parameters::Manager& params, bool enabled, std::string catalog_path_,
        Logging::Logger& logger_);
    virtual ~AbstractCatalogManager();
    void init();
    Region load_region(fs::path filename);
    void load_regions();
    bool in_range(double value, double min, double max);
    std::vector<Solving::Solution> load_triplets_from_file(std::vector<Solving::Blob>& base_blobs,
            int leg0, int leg1,
            double leg0min, double leg0max, double leg1min,
            double leg1max, double leg2min, double leg2max, Shared::Solving::Filters& filters);
    std::vector<Solving::Solution> load_solutions_from_triplet(std::vector<Solving::Blob>& blobs,
            Shared::Solving::Filters& filters);
    std::vector<Solving::Solution> load_pairs_from_file(std::vector<Solving::Blob>& base_blobs,
            int leg, double leg_min, double leg_max, Shared::Solving::Filters& filters);
    std::vector<Solving::Solution> load_solutions_from_pair(std::vector<Solving::Blob>& blobs,
            Shared::Solving::Filters& filters);
    std::vector<Solving::Star> get_stars_near(double ra, double dec);
    double get_epoch();
    void precess_if_desired(double epoch, float& ra, float& dec);
    void precess_if_desired(double epoch, double& ra, double& dec);
    void fill_stars_in_fov(::Solving::Solution& solution, Shared::Image::Raw& image,
        Shared::Solving::Refraction& refraction_settings);
    void get_star_name(int id, std::string& name, double& mag);

  protected:
    virtual bool abort();
    Logging::Logger *logger;

  private:
    int image_width;
    int image_height;
    fs::path catalog_path;
	std::vector<std::pair<double, std::string>> star_names;
    std::vector<Region> regions;
    double mag_limit;

};

#endif

