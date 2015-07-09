#! /usr/bin/env python

import os
import sys
import pyfits
import pylab

def load_raw_data(filename):
    infile = open(filename, "r")
    data = pylab.fromstring(infile.read(), "uint16")
    infile.close()
    return data

def load_fits_data(filename):
    infile = pyfits.open(filename)
    data = infile[0].data.flatten()
    infile.close()
    return data

def diff_image(raw_filename, fits_filename, verbose=False):
    filestem_raw = os.path.splitext(os.path.split(raw_filename)[-1])[0]
    filestem_fits = os.path.splitext(os.path.split(fits_filename)[-1])[0]
    if filestem_fits != filestem_raw:
        print "warning: files %s and %s don't have the same stem" % (filestem_raw, filestem_fits)
    if verbose:
        print "comparison:"
    raw_data = load_raw_data(raw_filename)
    if verbose:
        print "%64s --" % raw_filename,
        print raw_data
    fits_data = load_fits_data(fits_filename)
    if verbose:
        print "%64s --" % fits_filename,
        print fits_data
    differ = False
    if len(raw_data) != len(fits_data):
        differ = True
    elif (raw_data-fits_data).any():
        differ = True
    if differ:
        print "file %s contents differ" % filestem_raw
    else:
        print "file %s contents are identical" % filestem_raw

def diff_dir(raw_dirname, fits_dirname):
        for raw_filename in os.listdir(raw_dirname):
            match_found = False
            for fits_filename in os.listdir(fits_dirname):
                full_raw_filename = os.path.join(raw_dirname, raw_filename)
                full_fits_filename = os.path.join(fits_dirname, fits_filename)
                raw_stem = os.path.splitext(raw_filename)[0]
                fits_stem = os.path.splitext(fits_filename)[0]
                if raw_stem == fits_stem:
                    match_found = True
                    diff_image(full_raw_filename, full_fits_filename, verbose=False)
            if not match_found:
                print "could not find match for raw image", raw_filename
                

def main():
    try:
        raw_filename = sys.argv[1]
        fits_filename = sys.argv[2]
    except IndexError:
        print "Usage: ./diff_raw_with_fits.py raw_filename fits_filename"
        return
    if os.path.isdir(raw_filename) and os.path.isdir(fits_filename):
        diff_dir(raw_filename, fits_filename)
    else:
        diff_image(raw_filename, fits_filename, verbose=True)

if __name__ == "__main__":
    sys.exit(main())

