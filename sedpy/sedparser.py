#!/usr/bin/env /usr/bin/python

import sys
from sedobjects import *
import lineparser    #provides lineparser.parse(Type, string)

defaultfile = "description.txt"

connectors = []  #list of connectors
containers = []  #list of components and cables
expected = []    #list of parts that have ben referenced but not declared

def writeInfiles():
  """from connectors, containers lists creates database infiles"""
  for conn in connectors:
    conn.toInfile()
  for cont in containers:
    #print "writing", cont.ref
    for line in cont.lines: line.toInfile()
    for jack in cont.jacks:
      #print "writing", jack.ref, "with dest", jack.dest.ref
      if not jack.placeholder:
	for pin in jack.pins: pin.toInfile()
	jack.toInfile()
	if jack.cable and jack.cablemaster:
	  jack.cable.toInfile()
	  for cline in jack.cable.lines: cline.toInfile()
    cont.toInfile()

def mateJack(jack):
  """tries to find mate for jack, then does some pairing operations"""
  try: jack.dest = containers[containers.index(jack.dest_str)]
  except ValueError: #couldn't find target
    if jack.dest_str not in expected:
      expected.append(Component(jack.dest_str, "expected", None))
    return
  if jack.dest_jack is not None: #explicitly specified mating jack
    try: jack.mate = jack.dest.jacks[jack.dest.jacks.index(jack.dest_jack)]
    except ValueError: return #no match for explicit matching jack
    else: #explicit mate found
      if jack.mate.dest_str != containers[-1].ref or\
	  not jack.canMate(jack.mate): raise Failure("Mate doesn't fit right")
  else:    #non-explicit mate, search for one amongst jacks on destination
    for tempjack in jack.dest.jacks: 
      if jack.canMate(tempjack) and tempjack.dest_str == containers[-1].ref:
	if tempjack.mate is not None: 
	  raise Failure("Matching jack (%s/%s) already mated"\
	      %(tempjack.location.ref,tempjack.ref))
	jack.mate = tempjack
	break
  if jack.mate is None: return #no matching jack at all on part
  jack.mate.mate = jack
  jack.mate.dest = containers[-1]

  #unify the cables, mate takes precedence (declared first)
  if jack.cable is not None:
    if jack.mate.cable.length is None or jack.mate.cable.length < 0: 
      jack.mate.cable.length = jack.cable.length
    if jack.mate.cable.label == "" or jack.mate.cable.label is None:
      if jack.cable.label == "" or jack.cable.label is None:
	#make up a name since none is given
	jack.mate.cable.label == "%s-%s"%(jack.mate.location, jack.location)
      else: jack.mate.cable.label = jack.cable.label
    jack.cable = jack.mate.cable #unify cable references
    jack.mate.cablemaster = True
  else: jack.placeholder = True  #this jack is redundant

  #for pins on mate (from previous lines) add autogen lines, etc. as needed
  for pin in jack.mate.pins:
    line = pin.lines['box']
    newline = Line(line.desc, "(%s,%s)"%(jack.ref, pin.number))
    newline.autogen = True
    newline.owner = containers[-1]
    containers[-1].lines.append(newline)
    if jack.placeholder: #no cable is used
      pin.lines['cable'] = newline
    else: #cable
      cableline = Line(line.desc, "")
      cableline.autogen = True
      cableline.owner = jack.cable
      pin.lines['cable'] = cableline
      newpin = Pin(pin.number, line.desc, jack, newline, cableline)
      jack.pins.append(newpin)
      jack.cable.lines.append(cableline)

def addPins(line):
  """create pins (and auto-lines) for a given line"""
  for jacknum, pinnum in zip(line.jacknums, line.pinnums):
    #find jack and perform some consistency checks
    try: jack = containers[-1].jacks[containers[-1].jacks.index(jacknum)]
    except ValueError: 
      raise Failure("unrecognized jack: %s"%jacknum)
    lineend = 'box'
    if jack.placeholder: #jack is placeholder in direct-connected pair
      jack = jack.mate  #use the real jack...its mate
      lineend = 'cable'
    try:  #test pin number
      if jack.conn.flags['alpha'] == 'N' and ((int(pinnum) > jack.conn.count \
	  and jack.conn.count > 0) or int(pinnum) <= 0):
	raise Failure("invalid pin number: %s"%pinnum)
    except ValueError: 
      raise Failure("unexpected non-numeric pin: %s"%pinnum)

    if jack.mate is None: #unmated, add a dangling pin (no cline yet)
      newpin = Pin(pinnum, line.desc, jack, line, None)
      if newpin not in jack.pins: jack.pins.append(newpin)
    else: #already mated, need to add extra stuff
      try:  #see if pin has already been generated
	pin = jack.pins[jack.pins.index(pinnum)]
      except ValueError:  #pin not yet generated
	newline = Line(line.desc, "(%s,%s)"%(jack.mate.ref,pinnum))
	newline.autogen = True
	if jack.cable is None: #no cable
	  if lineend == 'box': 
	    newpin = Pin(pinnum, line.desc, jack, line, newline)
	    newline.owner = jack.mate.location
	    jack.mate.location.lines.append(newline)
	  else: 
	    newpin = Pin(pinnum, line.desc, jack, newline, line) 
	    newline.owner = jack.location
	    jack.location.lines.append(newline)
	else:  #using a cable, make another new line and pin
	  if lineend == 'cable': raise Failure('inconsistent state')
	  cableline = Line(line.desc, "")
	  cableline.autogen = True
	  cableline.owner = jack.cable
	  newpin = Pin(pinnum, line.desc, jack, line, cableline)
	  newerpin = Pin(pinnum, line.desc, jack.mate, newline, cableline)
	  jack.cable.lines.append(cableline)
	  if newerpin not in jack.mate.pins: jack.mate.pins.append(newerpin)
	jack.pins.append(newpin)

      else:  #pin exists
	if not pin.lines[lineend].autogen:
	  raise Failure("line conflicts for pin (%s,%s)"%(jacknum,pinnum))
	jack.parent.lines.remove(pin.lines[lineend])
	pin.desc = line.desc
	pin.lines[lineend] = line
	#other lines exist already (?), so this case is done

  if line in containers[-1].lines:
    raise Failure("line overlaps existing one")
  containers[-1].lines.append(line)

def sedparser(argv=None):
  """main function allows calls from interactive prompt, or extra cleverness"""
  #parse command line arguments, open description file
  startWithConnectors = True #connector list at top? make overridable by args
  if argv is None: argv = sys.argv
  if len(argv) == 2: 
    f = open(argv[1], 'r')
  else: 
    f = open(defaultfile, 'r')
  
  #main loop, red each line of the description file in turn
  linecount = 0
  for str in f:
    linecount += 1
    str = str.strip()

    #CONNECTORS
    if startWithConnectors: #set to false when done
      conn = lineparser.parse(Connector, str)
      if conn is not None: #successful
	if conn in connectors: raise Failure("connector exists", linecount)
	connectors.append(conn)
	try:
	  mate = connectors[connectors.index(conn.matetype)]
	  #check that each gender mate is reciprocated, or unused
	  if conn.gendersMatch(mate):
	    conn.mate = mate
	    mate.mate = conn
	  else: raise Failure("Incompatible gender mating", linecount)
	except ValueError: pass #mate not found
      elif str[0] == '*' and str == '*ENDCONNECTORLIST':
	startWithConnectors = False
	for c in connectors:
	  if c.mate is None: print "Warning: unmated connector", c.type
	print "Info: done parsing connectors, found", len(connectors)
      elif str != "":
	raise Failure("Unrecognized connector", linecount)
  
    elif str[0:4] == "JACK":
      jack = lineparser.parse(Jack, str)
      if jack is None: raise Failure("Bad jack line", linecount)
      if jack in containers[-1].jacks:
	raise Failure("non-unique jack identifier", linecount)
      try: 
	jack.conn = connectors[connectors.index(jack.conn_str[:-2])]
	if jack.conn.genders[jack.gender] == 'X':
	  raise Failure("Invalid connector gender", linecount)
      except ValueError: raise Failure("Non-existant connector type", linecount)
      jack.location = containers[-1]
      containers[-1].jacks.append(jack)
      try: mateJack(jack)
      except Failure, err: raise Failure(err.errstr, linecount)

    elif str[0:4] == "LINE":
      line = lineparser.parse(Line, str)
      if line is None: raise Failure("Bad line line", linecount)
      line.owner = containers[-1]
      try: addPins(line) #catch failures and reraise with line number
      except Failure, err: raise Failure(err.errstr, linecount)

    elif str[0:5] == "CABLE":
      cable = lineparser.parse(Cable, str)
      if cable is None: raise Failure("Bad cable line", linecount)
      if cable in containers: 
	raise Failure("cable exists: rename or use re-edit line", linecount)
      containers.append(cable)
      try: expected.remove(cable)
      except ValueError: pass

    elif str != "" and str[0] != '#': #COMPONENT
      comp = lineparser.parse(Component, str)
      if comp is None:  #check if line is a re-edit command
	try:
	  containers.append(containers.pop(containers.index(str)))
	  print "Info: re-editing", str
	  continue
	except ValueError: raise Failure("nothing found to re-edit", linecount)
      if comp in containers: 
	raise Failure("component exists: use re-edit line?", linecount)
      containers.append(comp)
      try: expected.remove(comp)
      except ValueError: pass
      if comp.partOf is not None and comp.partOf not in containers \
	  and comp.partOf not in expected:
	expected.append(Component(comp.partOf, "expected", None))


  for part in expected: print part.ref
  if len(expected) > 0: raise Failure("above parts used but not declared", -1)
  print "Info: done parsing, found", len(containers), "components and cables:"
  print "%10s%10s%10s"%("Part", "Jacks", "Lines")
  allMated = True
  cableCount = 1
  for cont in containers:
    print "%10s%10s%10s"%(cont.ref, len(cont.jacks), len(cont.lines))
    if hasattr(cont, 'number'): #it's a cable
      cont.number = cableCount
      cableCount += 1
    for ijack in cont.jacks:
      if ijack.conn.count > 0 and len(ijack.pins) > ijack.conn.count:
	raise Failure("Jack %s jas too many pins"%ijack.ref)
      if ijack.mate is None: 
	print ("\tJack %s unmated"%ijack.ref)
	allMated = False
      if ijack.cable and ijack.cablemaster:
	ijack.cable.number = cableCount
	cableCount += 1
  if not allMated: 
    raise Failure("Not all jacks mated, check for indistinguishable mates")


if __name__ == "__main__":
  try:
    sedparser()
    writeInfiles()
    print "\nAll Done!"
    sys.exit()
  except Failure, err:
    print >>sys.stderr, err

