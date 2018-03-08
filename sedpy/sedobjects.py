#!/usr/bin/env /usr/bin/python

#TODO: can keep UID fixed, but should update the time
#to get time: datetime.utcnow().replace(microsecond=0).isoformat(' ')
UID_UTIME = "1\t2008-08-08 08:08:08"    #for spider "sbenton" has UID 1
#UID_UTIME = "23\t2008-08-08 08:08:08"	#for BLASTpol "sbenton" has UID 23

class Failure(Exception):
  """exception to use for failures in parsing and matching"""
  def __init__(self, msg, linenum=0):
    self.errstr = msg
    self.msg="Error: %s" % msg
    if linenum > 0: self.msg += " (%d)"%linenum

  def __str__(self):
    return self.msg

class Connector:
  """contains information about objects in the connector and congender tables"""
  #file objects to write output to
  connout = open('out/connector', 'w')
  gconnout = open('out/congender', 'w')

  def __init__(self, type, matetype, count, desc, alpha, bcd, mMate, fMate):
    #need to find reference to mate
    self.type = type
    self.matetype = matetype
    self.desc = desc
    self.count = int(count)
    #alpha and bcd will wither be None or a non-empty string
    self.flags = {'alpha': 'N', 'bcd': 'N'}
    if (alpha): self.flags['alpha'] = 'Y';
    if (bcd): self.flags['bcd'] = 'Y';
    #gender contains 'M' or 'F' for mating pair, 'X' used to simplify comparison
    self.genders = {'M': 'X', 'F': 'X', 'X':'u'}
    if mMate: self.genders['M'] = mMate.upper()
    if fMate: self.genders['F'] = fMate.upper()
    self.mate = None   #object reference                                 #needed

  def gendersMatch(self, other):
    """ensure genders are unused or properly matched between two connectors"""
    return (other.genders[self.genders['M']] in 'uM' \
	and other.genders[self.genders['F']] in 'uF' \
	and self.genders[other.genders['M']] in 'uM' \
	and self.genders[other.genders['F']] in 'uF')

  def __str__(self):
    """return a string representing the connector"""
    if self.flags['alpha'] == "Y": alpha = " alpha"
    else: alpha = ""
    if self.flags['bcd'] == "Y": bcd = " bcd"
    else: bcd = ""
    if self.genders['M'] == 'X': male = ""
    else: male = " M-%s" % self.genders['M']
    if self.genders['F'] == 'X': female = ""
    else: female = " F-%s" % self.genders['F']
    return '%s -> %s %d "%s"%s%s%s%s' % (self.type, self.matetype,
	self.count, self.desc, alpha, bcd, male, female)

  def __eq__(self, other):
    """equality comparison"""
    if hasattr(other, 'type'): return self.type == other.type
    else: return self.type == other

  def toInfile(self):
    self.__class__.connout.write('%s\t%s\t%s\t%s\t%s\t%s\t%s\n'%(self.type, \
	self.desc, self.count, self.flags['alpha'], self.flags['bcd'], \
	self.mate.type, UID_UTIME))
    if self.genders['M'] != 'X':
      self.__class__.gconnout.write("%s\tM\t%s\t\\N\t\\N\t\\N\t\\N\t%s\n"%\
	  (self.type, self.genders['M'], UID_UTIME))
    if self.genders['F'] != 'X':
      self.__class__.gconnout.write("%s\tF\t%s\t\\N\t\\N\t\\N\t\\N\t%s\n"%\
	  (self.type, self.genders['F'], UID_UTIME))

class Component:
  """contains information about objects in the component table"""
  compout = open('out/component', 'w')

  def __init__(self, ref, name, partOf):
    #need to populate jack and line lists
    self.ref = ref
    self.name = name
    self.partOf = partOf
    self.jacks = []                                                      #needed
    self.lines = []                                                      #needed

  def __str__(self):
    if self.partOf: partof = " < %s" % self.partOf
    else: partof = ""
    return '%s "%s"%s' % (self.ref, self.name, partof)

  def __eq__(self, other):
    if hasattr(other, 'ref'): return self.ref == other.ref
    else: return self.ref == other

  def toInfile(self):
    if self.partOf is None: partOf = "\N"
    else: partOf = self.partOf
    self.__class__.compout.write("%s\t%s\t%s\t%s\n" % (self.ref,\
	self.name, partOf, UID_UTIME))

class Cable:
  """entries for the cable table. most are implicit p2p cables"""
  cabout = open('out/cable', 'w')

  def __init__(self, ref, label, length):
    #need to change p2p for implicit cables, populate jacks and lines lists
    self.number = 0                                                      #needed
    if ref and ref[0] == "&":	#try to infer number from ref
      if ref[1] == "C": number = int(ref[2:])
      else: number = int(ref[1:])
      if number > 0:
	self.number = number
    self.ref  = ref #internal use only
    self.label = label
    self.length = length and int(length) #short circuit avoids casting None
    self.p2p = 'N'  #is this a sensible default?                         #needed
    self.jacks = []                                                      #needed
    self.lines = []                                                      #needed

  def __str__(self):
    if self.number > 0: ref = "&C%d" % self.number
    else: ref = self.ref
    if self.length > 0: length = " %d" % self.length
    else: length = ""
    return 'CABLE %s "%s"%s' % (ref, self.label, length)

  def __eq__(self, other):
    if hasattr(other, 'ref'): return self.ref == other.ref
    else: return self.ref == other

  def toInfile(self):
    if self.length is None or self.length < 0: lenstr = "\\N"
    else: lenstr = self.length.str()
    self.__class__.cabout.write("%s\t%s\t%s\t%s\t%s\n" % (self.number,\
	self.label, self.p2p, lenstr, UID_UTIME))

class Jack:
  """contains information for jack table"""
  jackout = open('out/jack', 'w')

  def __init__(self, internal, ref, label, conn_str, dest_part, \
      dest_jack, c_num, c_desc, c_len):
    #need to populate pins list, find references to location, destination, conn
    #need to set placeholder and cablemaster where appropriate
    self.number = 0                                                      #needed
    #try to infer number from ref
    if ref and ref[0] == "&":
      if ref[1] == "J": number = int(ref[2:])
      else: number = int(ref[1:])
      if number > 0:
	self.number = number
    self.internal = internal  #either None or 'IN'
    self.ref = ref #internal use only (matched in LINEs)
    self.label = label
    self.conn_str = conn_str  #contains: "conn_type/gender"
    self.gender = self.conn_str[-1].upper()
    self.conn = None #object reference                                   #needed
    self.dest_str = dest_part
    self.dest_jack = dest_jack   #only needed for indistinguishable jacks
    self.location = None #object reference                               #needed
    self.dest = None #object reference	                                 #needed
    if c_desc is not None: #allows ""
      self.cable = Cable(c_num, c_desc, c_len)
      self.cable.p2p = 'Y'
    else: self.cable = None
    self.pins = []                                                       #needed
    self.mate = None #object reference                                   #needed
    self.placeholder = False #in direct connection, one of pair is placeholder
    self.cablemaster = False #in cable pair, cable shared but only 1 is master
    
  def canMate(self, other):
    #check cable use and length
    if (self.cable is not None and other.cable is not None):
      if (self.cable.length and other.cable.length and \
	  self.cable.length != other.cable.length): return False
    elif self.cable is not None or other.cable is not None: return False
    #check that connectors match properly
    if self.cable is not None: #for cables, ends must be same type or mates
      if (self.conn.mate == other.conn and other.conn.mate == self.conn) or \
	  self.conn == other.conn: return True
    else: #no cable means must be mates, and must have genders match
      if self.conn.mate == other.conn and other.conn.mate == self.conn and \
	  self.conn.genders[self.gender] == other.gender and \
	  other.conn.genders[other.gender] == self.gender: return True
    return False

  def __str__(self):
    if self.internal: internal = " IN"
    else: internal = ""
    if self.number > 0: ref = "&J%d" % self.number
    else: ref = self.ref
    if self.conn: connector = "%s/%s" % (self.conn.type, self.gender)
    else: connector = self.conn_str
    if self.dest:
      if hasattr(self.dest, 'number') and self.dest.number > 0:
	dest = "&C%d" % self.dest.number
      else: dest = self.dest.ref #component or unnumbered cable
    else: dest = self.dest_str
    if self.mate:
      if self.mate.number > 0: destjack = "&J%d" % self.mate.number
      else: destjack = self.mate.ref
    elif self.dest_jack: destjack = self.dest_jack
    else: destjack = ""
    if destjack: destination = "%s/%s" % (dest, destjack)
    else: destination = dest
    if self.cable:
      if self.cable.number > 0: cref = " &C%d" % self.cable.number
      #elif self.cable.ref: cref = " %s" % self.cable.ref #shouldn't happen
      else: cref = ""
      if self.cable.length: length = " %d" % self.cable.length
      else: length = ""
      cable = ' CABLE%s "%s"%s' % (cref, self.cable.label, length)
    else: cable = ""
    return 'JACK%s %s "%s" %s -> %s%s' % (internal, ref, self.label,
	connector, destination, cable)

  def __eq__(self, other):
    if hasattr(other, 'ref'): return self.ref == other.ref
    else: return self.ref == other

  def toInfile(self):
    if self.placeholder: return   #don't write placeholders
    if self.cable is not None: dest = self.cable.number #implicit cable
    elif hasattr(self.dest, 'p2p'): dest = self.dest.number #explicit cable
    else: dest = self.dest.ref  #mates directly to component
    if hasattr(self.location, 'p2p'): location = self.location.number
    else: location = self.location.ref
    if self.internal: internal = 'Y'
    else: internal = 'N'
    self.__class__.jackout.write("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n"%(\
	self.number, self.gender, internal, location, self.conn.type, dest,\
	self.label, UID_UTIME))
    
import re
class Line:
  """contains information on the line table"""
  pinRE = re.compile(r'\(((?:&J?\d{1,5})|[a-z0-9]+)[;,]((?:\s*[A-Za-z0-9]{1,3},?)*)\)')
  count = 1
  lineout = open('out/line', 'w')

  def __init__(self, desc, pin_str):
    #pin_str takes further processing, need to fill in owner (comp or cable)
    self.number = self.__class__.count
    self.__class__.count += 1
    self.desc = desc
    self.pinnums = [];  #internal reference
    self.jacknums = []; #internal reference
    for match in self.__class__.pinRE.finditer(pin_str):
      for pin in match.group(2).replace(',',' ').split():
	self.jacknums.append(match.group(1))
	self.pinnums.append(pin)
    self.owner = None #object reference                                  #needed
    self.autogen = False #set to true for automatically generated lines  #needed
                         #autogen lines will only connect to a single pin

  def __str__(self):
    old_jacknum = self.jacknums[0]
    pintok = ["(%s;"%old_jacknum]
    for jacknum, pinnum in zip(self.jacknums, self.pinnums):
      if jacknum != old_jacknum:
	pintok.append("),(%s;" % jacknum)
	old_jacknum = jacknum
      if pintok[-1][-1] != ';': pintok.append(',')
      pintok.append(pinnum)
    pintok.append(")")
    return 'LINE "%s" %s' % (self.desc, "".join(pintok))
    #return str((self.number, self.desc, zip(self.jacknums, self.pinnums)))

  def __eq__(self, other):
    """doesn't quite test equality in this case: more like overlap"""
    if self.autogen or other.autogen: return False #autogens don't count
    for jackpin in zip(self.jacknums, self.pinnums):
      if jackpin in zip(other.jacknums, other.pinnums): return True
    return False

  def toInfile(self):
    if hasattr(self.owner,'p2p'):  #owner is a cable, so use its number
      owner = self.owner.number
    else: owner = self.owner.ref  #it is a component
    self.__class__.lineout.write("%s\t%s\t%s\t%s\n"%(self.number,\
	self.desc, owner, UID_UTIME))

class Pin:
  """info for the pin table, is inferred from lines and jacks"""
  pinout = open('out/pin', 'w')

  def __init__(self, number, desc, jack, bline, cline):
    self.number = number #not necessarily a number; 3-digit string
    self.desc = desc
    #jack and lines are all object references instead of numbers
    self.jack = jack  #object reference
    self.lines = {'box': bline, 'cable': cline}

  def __eq__(self, other):
    #should only be used from within a given jack
    if hasattr(other, 'number'): return self.number == other.number
    else: return self.number == other

  def toInfile(self):
    self.__class__.pinout.write("%s\t%s\t%s\t%s\t%s\tN\tN\t%s\n"%(self.number,\
	self.jack.number, self.desc, self.lines['box'].number,\
	self.lines['cable'].number, UID_UTIME))
    
