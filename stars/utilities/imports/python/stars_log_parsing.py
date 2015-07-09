import os
from angles import *

def get_last_logname(logs_dirname, logtype):
    for date_dirname in reversed(sorted(os.listdir(logs_dirname))):
        if os.path.isdir(os.path.join(logs_dirname, date_dirname)):
            for filename in reversed(sorted(os.listdir(os.path.join(logs_dirname, date_dirname)))):
                if filename.endswith(logtype+".log"):
                    return os.path.join(logs_dirname, date_dirname, filename)
    return None

class Coordinates():
    def __init__(self):
        self.ra = None
        self.dec = None
        self.az = None
        self.el = None
        self.roll = None
        self.filled = False
    def set_equatorial(self, ra, dec, roll):
        self.ra = ra
        self.dec = dec
        self.roll = roll
        self.filled = True
    def set_horizontal(self, az, el, roll):
        self.az = az
        self.el = el
        self.roll = roll
        self.filled = True
    def __repr__(self):
        if self.filled:
            s = "Coordinates\n"
        if self.ra:
            s += "    equatorial: %s" % str(map(to_degrees, [self.ra, self.dec, self.roll]))
        if self.az:
            s += "    horizontal: %s" % str(map(to_degrees, [self.az, self.el, self.roll]))
        return s

class Attitude():
    def __init__(self):
        self.equatorial =        Coordinates()
        self.equatorial_errors = Coordinates()
        self.horizontal =        Coordinates()
        self.horizontal_errors = Coordinates()

class ImageStats():
    pass

class HorizontalInfo():
    pass

class Solution:
    def __init__(self):
        self.fluxer = Fluxer()
        self.blobs = []
        self.stars = []
        self.matches = []
        self.image_stats = ImageStats()
        self.attitude = Attitude()
        self.horizontal_info = HorizontalInfo()
    def add_match(self, line):
        blob_id = int(line.split()[7])
        star_id = int(line.split()[12])
        star_fitted_x = float(line.split()[16])
        star_fitted_y = float(line.split()[17])
        blob = None
        for solution_blob in self.blobs:
            if solution_blob.id == blob_id:
                blob = solution_blob
        star = None
        for solution_star in self.stars:
            if solution_star.id == star_id:
                star = solution_star
        star.fitted_x = star_fitted_x
        star.fitted_y = star_fitted_y
        self.matches.append(Match(blob, star))
    def __str__(self):
        s = "solution for image " + str(self.filename)
        s += " matching " + str(len(self.matches))
        return s
    def __repr__(self):
        return str(self)

class Match:
    def __init__(self, blob, star):
        self.blob = blob
        self.star = star

class Fluxer:
    def __init__(self):
        self.best_fit_line = None
        self.best_fit_exposure = None
        self.num_blobs_fit = None
        self.num_blobs_ignored = None

class Blob:
    def __init__(self, from_line=None):
        if from_line != None:
            self.unload_line(from_line)
    def unload_line(self, line):
        self.id = int(line.split()[8])
        self.x = float(line.split()[10])
        self.y = float(line.split()[11])
        self.flux = float(line.split()[13])
        saturated = line.split()[15]
        self.saturated = False
        if saturated in ["1"]:
            self.saturated = True
    def __str__(self):
        s = "blob with"
        for attribute in ["id", "x", "y", "flux", "saturated"]:
            s += " %s %d" % (attribute, getattr(self, attribute))
        return s

class Star:
    def __init__(self, from_line=None):
        if from_line != None:
            self.unload_line(from_line)
    def unload_line(self, line):
        self.id = int(line.split()[6])
        self.ra = from_degrees(float(line.split()[9]))
        self.dec = from_degrees(float(line.split()[12]))
        self.flux = float(line.split()[14])
    def __str__(self):
        s = "star with"
        for attribute in ["id", "ra", "dec", "flux"]:
            s += " %s %d" % (attribute, getattr(self, attribute))
        return s

def add_line_to_fluxer(fluxer, line):
    if "fluxer: found best fit line" in line:
        fluxer.best_fit_line = float(line.split()[8])
    if "fluxer: found best fit exposure" in line:
        fluxer.best_fit_exposure = float(line.split()[8])/1000.0
    if "fluxer: ignoring" in line:
        fluxer.num_blobs_ignored = int(line.split()[5])
    if "fluxer: fitting" in line:
        fluxer.num_blobs_fit = int(line.split()[5])

def add_line_to_solution(line, solution):
    if "starting to solve image" in line:
        solution.filename = line.split()[7]
    if "fluxer:" in line:
        add_line_to_fluxer(solution.fluxer, line)
    if "finder: found blob with id" in line:
        solution.blobs.append(Blob(from_line=line))
    if "solution: star id" in line:
        solution.stars.append(Star(from_line=line))
    if "solution: blob with" in line and "matches star" in line:
        solution.add_match(line)
    if "solution: equatorial attitude (deg)" in line:
        solution.attitude.equatorial.set_equatorial(\
            *map(from_degrees, map(float, line.split()[7:10])))
    if "solution: equatorial errors (arcsec)" in line:
        solution.attitude.equatorial_errors.set_equatorial(\
            *map(from_arcsec, map(float, line.split()[7:10])))
    if "solution: equatorial pointing error (arcsec)" in line:
        solution.attitude.equatorial_pointing_error = from_arcsec(float(line.split()[8]))
    if "solution: horizontal attitude (deg)" in line:
        solution.attitude.horizontal.set_horizontal(\
            *map(from_degrees, map(float, line.split()[7:10])))
    if "solution: horizontal errors (arcsec)" in line:
        solution.attitude.horizontal_errors.set_horizontal(\
            *map(from_arcsec, map(float, line.split()[7:10])))
    if "solution: equatorial iplatescale (arcsec/px)" in line:
        solution.iplatescale = from_arcsec(float(line.split()[7]))
    if "stats: mean" in line:
        solution.image_stats.mean = float(line.split()[5])
    if "stats: noise" in line:
        solution.image_stats.noise = float(line.split()[5])
    if "solution fitter: final pass used lat" in line:
        solution.horizontal_info.lat = from_degrees(float(line.split()[9]))
        solution.horizontal_info.lst = from_hours(float(line.split()[12]))

def get_solutions_from_output(output):
    solutions = []
    in_solution = False
    for line in output:
        if "starting to solve image" in line:
            solutions.append(Solution())
            in_solution = True
        if "finished solving image" in line:
            in_solution = False
        if in_solution:
            add_line_to_solution(line, solutions[-1])
    return solutions

def get_solutions(log):
    if log.endswith(".log") and "STARS is running" not in log:
        output = open(log).readlines()
    else:
        output = log.split("\n")
    return get_solutions_from_output(output)

def get_last_solution(log, imagename=None):
    solutions = get_solutions(log)
    if len(solutions) > 0 and solutions[-1].attitude.equatorial.filled:
        if imagename == None or solutions[-1].filename == imagename:
            return solutions[-1]
    return None

def get_solutions_with_matches_from_dir(dirname):
    solutions = []
    for filename in sorted(os.listdir(dirname)):
        possible_solutions = get_solutions(os.path.join(dirname, filename))
        for possible_solution in possible_solutions:
            if len(possible_solution.matches) > 0:
                solutions.append(possible_solution)
    return solutions

