#! /usr/bin/env python

import os

for filename in os.listdir("."):
    if filename.endswith(".ez"):
        outfilename = filename.strip(".ez")
        command = "ezip -d %s %s" % (filename, outfilename)
        os.system(command)

