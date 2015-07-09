/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__MATCHING__FLUXER_H
#define SOLVING__MATCHING__FLUXER_H

#include <vector>

namespace Solving
{
    class Solution;
    class Blob;
    class Star;

    namespace Matching
    {
        class Fluxer;
    }
}

namespace levmar_fluxer
{

struct additional_data
{
    double* star_fluxes;
    double* errors;
};

}

class Solving::Matching::Fluxer
{
  public:
    Fluxer();
    ~Fluxer();
    void fit(Solution& solution, std::vector<Blob>& blobs, std::vector<Star>& stars, bool print=true);

  private:
    unsigned int max_num_blobs;
    double* workspace;
    double* covar;
    levmar_fluxer::additional_data adata;
    double* blob_fluxes;
    double* params;
    double* info;
};

#endif
