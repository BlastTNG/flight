#! /usr/bin/env python

import pylab
import Image

class Header:
    pass

def parse_header(header_str):
    header = Header()
    header.tex_width = pylab.fromstring(header_str[4:8], "int32")[0]
    header.tex_height = pylab.fromstring(header_str[8:12], "int32")[0]
    header.start_char = pylab.fromstring(header_str[12:16], "int32")[0]
    header.end_char = pylab.fromstring(header_str[16:20], "int32")[0]
    return header

def create_glf(original_filename, image):
    infile = open(original_filename, "r")
    contents = infile.read()
    infile.close()
    header = parse_header(contents[0:24])    
    num_chars = header.end_char - header.start_char + 1
    return contents[0: 24+num_chars*24] + image.tostring()

def save_file(outdata, outfilename):
    outfile = open(outfilename, "wb")
    outfile.write(outdata)
    outfile.close()

original_filename = "arial24.glf"
infilename = "arial24_with_degrees.png"
image = Image.open(infilename)
outdata = create_glf(original_filename, image)
save_file(outdata, infilename.replace(".png", ".glf"))


