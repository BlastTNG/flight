#!/usr/bin/env /usr/bin/python

import re
from sedobjects import *

#LABEL -> MATE pins "description" alpha bcd M-Mmate F-Fmate
connRE = re.compile(r'([A-Z0-9_]{1,16})\s+->\s+([A-Z0-9_]{1,16})\s+(\d{1,3})'\
    r'\s+"([^"]{1,255})"\s+([yYnN])\s+([yYnN])\s+M-([mMfFxX])\s+F-([mMfFxX])')

#NAME_1 "Long form of name" [< PARENT]
compRE = re.compile(r'([A-Z0-9_]{1,8})\s+"([^"]{1,65})"' \
    r'(?:\s+<\s+([A-Z0-9_]{1,8}))?')

#JACK [IN] ref "label" conn/g -> dest_part[/jackref] [CABLE "desc" [len]]
jackRE = re.compile(r'JACK(?:\s*(IN))?\s*([a-z0-9]+)\s+"([^"]{1,32})"\s+'\
    r'([A-Z0-9_]{1,16}/[mMfF])\s+->\s+([A-Z0-9_]{1,8})(?:/([a-z0-9]+))?'\
    r'(?:\s+CABLE\s+"([^"]{0,64})(?:\s+(\d+))?)?')

#LINE "description" (jack#,pin#[,pin#,...])[,(jack#,pin#s),...]
lineRE = re.compile(r'LINE\s+"([^"]{1,64})"\s+'\
    r'((?:\([a-z0-9]+[;,](?:\s*[A-Za-z0-9]{1,3},?)*\),?\s*)+)')

#CABLE key "description" [len]
cableRE = re.compile(r'CABLE\s+(C?[0-9]{1,8})\s+"([^"]{1,64})"(?:\s+([0-9]+))?')

#lookup table of regular expressions corresponding to object classes
RElookup = {Connector: connRE, Component: compRE, Jack: jackRE, \
    Line: lineRE, Cable: cableRE}

def parse(type, line):
  """parses the string: line into an object of class: type and returns it"""
  match = RElookup[type].match(line)
  if match:
    return type(*match.groups())
  else:
    return None


if __name__ == "__main__":
  print "COMPONENT tests"
  comp1 = 'COMPA "COMPA is part of COMP1" < COMP1'
  comp2 = 'COMP1 "COMP1 is not part of anything"'
  comp3 = 'some random string that does not match'
  print repr(comp1), '\ngives:', parse(Component,comp1)
  print repr(comp2), '\ngives:', parse(Component,comp2)
  print repr(comp3), '\ngives:', parse(Component,comp3)

  print "\nCONNECTOR tests"
  conn1 = 'TYPE -> MATE 10 "TYPE has 10 male pins that go with female MATE"\
      y y M-F F-X'
  conn2 = 'CONN -> CONN 0 "A connector with variable num of pins" y y M-M F-X'
  print repr(conn1), '\ngives', parse(Connector,conn1)
  connector = parse(Connector, conn2)
  connector.mate = connector
  print repr(conn2), '\ngives', connector

  print "\nCABLE tests"
  cab1 = 'CABLE 12345 "This cable has no length"'
  cab2 = 'CABLE 12346 "This cable is 350mm long" 350'
  print repr(cab1), '\ngives', parse(Cable, cab1)
  print repr(cab2), '\ngives', parse(Cable, cab2)

  print "\n JACK tests"
  jack1 = 'JACK1 "connects to COMPA with no cable" CONN/M -> COMPA'
  jack2 = 'JACK2 "now has cable" CONN2/f -> COMPB/j12 CABLE "cable, len=?"'
  jack3 = 'JACK j3 "now has cable" CONN2/f -> COMPB CABLE "cable, len=300" 300'
  jack4 = 'JACK 4 "cable desc is blank" CONN/m -> COMP CABLE ""'
  jack5 = 'JACK 5 "a possible mate for jack 4" CONN/M -> COMP CABLE "soemthing"'
  print repr(jack1), '\ngives', parse(Jack, jack1)
  print repr(jack2), '\ngives', parse(Jack, jack2)
  print repr(jack3), '\ngives', parse(Jack, jack3)
  j4 = parse(Jack, jack4)
  j4.conn = connector
  print repr(jack4), '\ngives', j4
  j5 = parse(Jack, jack5)
  j5.conn = connector
  print repr(jack5), '\ngives', j5
  if j5.canMate(j4): print "It can mate with the previous jack"
  else: print "It can't mate with the previous jack"

  print "\nLINE tests"
  line1 = 'LINE "this line has one pin" (0,1)'
  line2 = 'LINE "this one has 4 pins" (0,1),(0,2),(1,A),(1,C)'
  line3 = 'LINE "shortform for above" (0;1,2),(1;A,C)'
  print repr(line1), '\ngives', parse(Line, line1)
  print repr(line2), '\ngives', parse(Line, line2)
  print repr(line3), '\ngives', parse(Line, line3)
