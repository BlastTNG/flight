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

def load_glf(filename):
    infile = open(filename, "r")
    contents = infile.read()
    infile.close()
    header = parse_header(contents[0:24])
    num_chars = header.end_char - header.start_char + 1
    characters_str = contents[24: 24+num_chars*24]
    num_tex_bytes = header.tex_width * header.tex_height * 2;
    tex_bytes = contents[24+num_chars*24: 24+num_chars*24 + num_tex_bytes]
    image = Image.fromstring("LA", [header.tex_width, header.tex_height], tex_bytes)
    return image

filename = "arial24.glf"
image = load_glf(filename)
#image.show()
image.save(filename.replace(".glf", ".png"))

# To preserve transparency, edit with gimp, and when saving
# deselect "Save color values from transparent pixels".

