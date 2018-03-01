#! /usr/bin/env python

import stars_log_parser
import pylab as pl

def plot_solution_fit(solution):
    exaggeration = 100.0
    star_xs = pl.array(map(lambda match: match.star.fitted_x, solution.matches))
    star_ys = pl.array(map(lambda match: match.star.fitted_y, solution.matches))
    blob_xs = pl.array(map(lambda match: match.blob.x, solution.matches))
    blob_ys = pl.array(map(lambda match: match.blob.y, solution.matches))
    error_xs = (blob_xs - star_xs)*exaggeration
    error_ys = (blob_ys - star_ys)*exaggeration
    pl.plot(star_xs, star_ys, 'ro', label="star_positions")
    for i in range(len(star_xs)):
        label = None
        if i == 0:
            label = "error toward blob position\n(exaggerated %.1f)"%exaggeration
        pl.plot([star_xs[i], star_xs[i]+error_xs[i]], [star_ys[i], star_ys[i]+error_ys[i]],\
            'b-', label=label)
    pl.legend()
    pl.xlabel("pixel x-coordinate")
    pl.ylabel("pixel y-coordinate")

last_logname = stars_log_parser.get_last_logname("../../output_dir/logs", "solving")
last_solution = stars_log_parser.get_solutions(last_logname)[-1]
plot_solution_fit(last_solution)
pl.show()

