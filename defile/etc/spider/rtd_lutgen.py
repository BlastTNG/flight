#!/usr/bin/env /usr/bin/python

""" generates a lookup from x = Vt / Vb to Rt
      
     +Vb/2  o----- R1 ----+---- Rb ----+----o  +Vt/2
                          |            |
		          |            |
		          R2           Rt
		          |            |
		          |            |
     -Vb/2  o----- R1 ----+---- Rb ----+----o  -Vt/2

"""

import numpy as np

#define size of bias bridges. tmax should be highest value of Rt in the table
R = { "ntd": { "1": 4700, "2": 220, "b": 30e6, "tmax": 10e6},
      "cnx": { "1": 12e3, "2": 100, "b": 33e4, "tmax": 15e4}
    }

def get_Rt(r, x):
  """from a set of resistor values and an x measurement, get Rt"""
  return (2*x*(2*r["1"]*r["b"] + r["2"]*r["b"] + r["1"]*r["2"])) / \
      (r["2"] - x*(2*r["1"] + r["2"]))

def get_xmax(r):
  """from a set of resistor values including tmax, get xmax"""
  return (r["2"]*r["tmax"]) / \
      ((2*r["1"]+r["2"])*(2*r["b"]+r["tmax"]) + 2*r["1"]*r["2"])

def writefile(name, key, n):
  x = np.linspace(0, get_xmax(R[key]), n)
  np.savetxt(name, np.rec.array((x, get_Rt(R[key], x))))

writefile("r_cernox.lut", "cnx", 100)
writefile("r_ntd.lut", "ntd", 100)
