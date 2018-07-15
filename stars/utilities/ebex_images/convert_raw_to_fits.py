#! /usr/bin/env python

import os
import pylab
import pyfits

image_width = 1536
image_height = 1024

def convert(infilename):
    infile = open(infilename, "r")
    contents = infile.read()
    infile.close()
    flat_data = pylab.fromstring(contents, "uint16")
    if len(flat_data) != image_width*image_height:
        print "data has length", len(flat_data), "which does not match", image_width, "*", image_height
        exit()

    data = []
    for j in range(image_height):
        data.append(flat_data[j*image_width: (j+1)*image_width])
    data = pylab.array(data)

    header = pyfits.core.Header()
    header.update("SIMPLE", "T")
    header.update("BITPIX", 16)
    header.update("EXTEND", "T")
    pyfits.writeto(infilename.replace(".raw", ".fits"), data, header)

for filename in os.listdir("."):
    if filename.endswith(".raw"):
        print "converting", filename
        convert(filename)

