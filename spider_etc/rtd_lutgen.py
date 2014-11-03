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

import sys
import numpy as np

#define size of bias bridges. tmax should be highest value of Rt in the table
R = { 
      #"ntd" test board coniguration
      "ntd-test": { "1": 4700., "2": 220., "b": 3300., "tmax": 1100.},
      #"ntd"  configuration with warm load resistors
      # "ntd": { "1": 4700., "2": 220., "b": 50.e6, "tmax": 10.e6},
      #"ntd" proper coniguration
      "ntd": { "1": 4700., "2": 220., "b": 30.e6, "tmax": 10.e6},
      "cnx": { "1": 12.e3, "2": 100., "b": 33.e4, "tmax": 15.e4}
    }

def get_Rt(r, x):
  """from a set of resistor values and an x measurement, get Rt"""
  return (2*x*(2*r["1"]*r["b"] + r["2"]*r["b"] + r["1"]*r["2"])) / \
      (r["2"] - x*(2*r["1"] + r["2"]))

def get_x(r, Rt):
  """from a set of resistor values and an Rt measurement, get x"""
  return (r["2"]*Rt) / \
      ((2*r["1"]+r["2"])*(2*r["b"]+Rt) + 2*r["1"]*r["2"])

def writefile(name, key, n):
  print get_x(R[key], R[key]["tmax"])
  x = np.linspace(0, get_x(R[key], R[key]["tmax"]), n)
  np.savetxt(name, np.rec.array((x, get_Rt(R[key], x))))

def usage():
  print 'rtd_lutgen: utilize the nonlinear expression for thermistor resistance'
  print 'Usage: ./rtd_lutgen.py action'
  print
  print 'Where "action" (with potential extra args) can be'
  print
  print './rtdlutgen.py lut [path]'
  print 'to generate ntd and cnx lookup files, optional path (default ".")'
  print
  print './rtdlutgen.py cal key Rmeas Rpcm'
  print 'produces voltage calibration gain'
  print 'key is ntd, ntd-test (for a fake-insert ntd), or cnx (for cernox)'
  print 'Rmeas is a calibrated measurement of the resistance'
  print 'Rpcm is pcm-reported resistance'
  print
  print './rtdlutgen.py R key x'
  print 'evaluate the function R for set "key" at value x'
  print
  print './rtdlutgen.py Rinv key Rt'
  print 'evaluate the function R inverse for set "key" at value Rt'
  print


#handle simple commands chosen on the command line
if len(sys.argv) < 2:
  usage()
  sys.exit(1)

if sys.argv[1] == "lut":
  try:
    path = sys.argv[2]
  except IndexError:
    path = '.'
  print 'Generating luts (in path "%s")' % path
  writefile(path + "/r_cernox.lut", "cnx", 100)
  writefile(path + "/r_ntd.lut", "ntd", 100)
  writefile(path + "/r_ntd-test.lut", "ntd-test", 100)

elif sys.argv[1] == "cal" and len(sys.argv) == 5:
  key = sys.argv[2]
  Rmeas = float(sys.argv[3])
  Rpcm = float(sys.argv[4])

  if key not in R:
    print 'unknown calibration key: %s' % key
    sys.exit(1)

  print 'Calibrating %s: Rpcm %e, Rmeas %e' % (key,Rpcm,Rmeas)
  print 'Gain: %s' % (get_x(R[key],Rmeas) / get_x(R[key],Rpcm))

elif sys.argv[1] == "R" and len(sys.argv) == 4:
  key = sys.argv[2]
  x = float(sys.argv[3])
  print 'R(%s,%e) = %e' % (key, x, get_Rt(R[key], x))

elif sys.argv[1] == "Rinv" and len(sys.argv) == 4:
  key = sys.argv[2]
  Rt = float(sys.argv[3])
  print 'R^(-1)(%s,%e) = %e' % (key, Rt, get_x(R[key], Rt))

else:
  usage()
  sys.exit(1)
