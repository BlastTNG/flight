#! /usr/bin/env python

import os
import pylab as pl
import stars_log_parser as parser

def plot_fluxes(solution):
    print "plotting flux for", solution.filename
    print "solution had fit %d blobs and ignored %d blobs" \
        % (solution.fluxer.num_blobs_fit, solution.fluxer.num_blobs_ignored)
    print "solution had best fit exposure", solution.fluxer.best_fit_exposure, "s"
    star_fluxes = map(lambda match: match.star.flux, solution.matches)
    blob_fluxes = map(lambda match: match.blob.flux, solution.matches)
    pl.plot(star_fluxes, blob_fluxes, 'o')
    xlim = list(pl.xlim())
    xlim[0] = 0
    max_x = max(star_fluxes) * 1.1
    xlim[1] = max_x
    ylim = list(pl.ylim())
    ylim[0] = 0
    pl.plot([0, max_x], [0, max_x*solution.fluxer.best_fit_line], '-')
    pl.xlim((xlim))
    pl.ylim((ylim))
    pl.xlabel("star flux (electrons / m^s / s)")
    pl.ylabel("blob flux (ADU)")

last_solving_logname = parser.get_last_logname("../../output_dir/logs", "solving")
print "using", last_solving_logname
print "using last solution"
solutions = parser.get_solutions(last_solving_logname)
plot_fluxes(solutions[-1])
pl.show()

