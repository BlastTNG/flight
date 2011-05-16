#!/usr/bin/env /usr/bin/python

import re
from sedobjects import *

#LABEL -> MATE pins "description" [alpha] [bcd] [M-Mmate] [F-Fmate]
connRE = re.compile(r'^'
    r'([A-Z0-9_]{1,16})\s+->\s+'  #LABEL
    r'([A-Z0-9_]{1,16})\s+'	  #MATE
    r'(\d{1,3})\s+'		  #pins
    r'"([^"]{1,255})"'		  #description
    r'(?:\s*(alpha))?'		  #alpha
    r'(?:\s*(bcd))?'		  #bcd
    r'(?:\s+M-([mMfF]))?'	  #mmate
    r'(?:\s+F-([mMfF]))?'	  #fmate
    r'$')

#CMPNAME "Description of the COMPONENT" [< PARTOF]
compRE = re.compile(r'^'
    r'([A-Z_][A-Z0-9_]{0,7})\s+'    #CMPNAME
    r'"([^"]{1,65})"'		    #description
    r'(?:\s+<\s+([A-Z0-9_]{1,8}))?' #PARTOF
    r'$')

#JACK [IN] [&]ref "label" CONN/G -> DEST[/jack] [CABLE [&C##] "[desc]" [len]]
jackRE = re.compile(r'^JACK'
    '(?:\s*(IN))?\s+'			#IN
    r'((?:&J?\d{1,5})|[a-z0-9]+)\s+'	#ref
    r'"([^"]{1,32})"\s+'		#label
    r'([A-Z0-9_]{1,16}/[mMfF])\s+->\s+'	#CONN/G
    r'(&?[A-Z0-9_]{1,8})'		#DEST
    r'(?:/((?:&J?\d{1,5})|[a-z0-9]+))?'	#/jack
    r'(?:\s+CABLE'			#CABLE
      r'(?:\s+(&C?[0-9]{1,8}))?\s+'	#   &C##
      r'"([^"]{0,64})"'			#   desc
      r'(?:\s+(\d+))?'			#   len
    r')?$')

#LINE "description" (jack;pin[,pin2,...])[,(jack;pin[,pin2,...]), ...]
lineRE = re.compile(r'^LINE\s+'
    r'"([^"]{1,64})"\s+'		    #description
    r'((?:'
    r'\((?:(?:&J?\d{1,5})|[a-z0-9]+)[;,]'   #jack
    r'(?:\s*[A-Za-z0-9]{1,3},?)*\),?\s*'    #pin
    r')+)$')

#CABLE [&]C## "description" [len]
cableRE = re.compile(r'^CABLE\s+'
    r'(&?C?[0-9]{1,8})\s+'  #C##
    r'"([^"]{1,64})"'	    #description
    r'(?:\s+([0-9]+))?'	    #len
    r'$')

#lookup table of regular expressions corresponding to object classes
RElookup = {Connector: connRE, Component: compRE, Jack: jackRE, 
    Line: lineRE, Cable: cableRE}

def parse(type, line):
  """parses the string: line into an object of class: type and returns it"""
  match = RElookup[type].match(line)
  if match:
    return type(*match.groups())
  else:
    return None


if __name__ == "__main__":
  print "\nCONNECTOR tests"
  conn1 = ('TYPE -> MATE 10 "TYPE has 10 male pins that go with female MATE" '
      'alpha bcd M-F')
  conn2 = 'CONN -> CONN 0 "A connector with variable num of pins" alpha F-F'
  conn3 = 'COMM -> COMM 0 "A connector with non-alphanumeric pins" F-F'
  conn4 = 'TEST -> TEST 10 "A connector to test jack mating" M-F F-M'
  print (conn1), '\n', parse(Connector,conn1)
  print (conn2), '\n', parse(Connector, conn2)
  print (conn3), '\n', parse(Connector, conn3)
  test_connector = parse(Connector, conn4)
  test_connector.mate = test_connector
  print (conn4), '\n', test_connector

  print "\nCOMPONENT tests"
  comp1 = 'COMPA "COMPA is part of COMP1" < COMP1'
  comp2 = 'COMP1 "COMP1 is not part of anything"'
  comp3 = 'some random string that does not match'    #bad declaration
  print (comp1), '\n', parse(Component,comp1)
  print (comp2), '\n', parse(Component,comp2)
  print (comp3), '\n', parse(Component,comp3)

  print "\nCABLE tests"
  cab1 = 'CABLE 12345 "This cable has no length"'
  cab2 = 'CABLE &C12346 "This cable is 350mm long" 350'
  print (cab1), '\n', parse(Cable, cab1)
  print (cab2), '\n', parse(Cable, cab2)

  print "\n JACK tests"
  jack1 = 'JACK 1 "connects to COMPA with no cable" CONN/M -> COMPA'
  jack2 = 'JACK &J2 "now has cable" CONN2/F -> COMPB/2 CABLE &C4 "cable, len=?"'
  jack3 = 'JACK j3 "now has cable" CONN2/F -> COMPB CABLE "cable, len=300" 300'
  jack4 = 'JACK 4 "cable desc is blank" TEST/M -> COMP CABLE ""'
  jack5 = 'JACK 5 "a possible mate for jack 4" TEST/M -> COMP CABLE "soemthing"'
  print (jack1), '\n', parse(Jack, jack1)
  print (jack2), '\n', parse(Jack, jack2)
  print (jack3), '\n', parse(Jack, jack3)
  j4 = parse(Jack, jack4)
  j4.conn = test_connector
  print (jack4), '\n', j4
  j5 = parse(Jack, jack5)
  j5.conn = test_connector
  print (jack5), '\n', j5
  if j5.canMate(j4): print "It CAN mate with jack 4"
  else: print "It CAN NOT mate with jack 4"

  print "\nLINE tests"
  line1 = 'LINE "this line has one pin" (&J22,1)'
  line2 = 'LINE "this one has 4 pins" (0,1),(0,2),(1,A),(1,C)'
  line3 = 'LINE "shortform for above" (0;1,2),(1;A,C)'
  print (line1), '\n', parse(Line, line1)
  print (line2), '\n', parse(Line, line2)
  print (line3), '\n', parse(Line, line3)
