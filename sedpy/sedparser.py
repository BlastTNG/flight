#!/usr/bin/env /usr/bin/python

import sys
import os
import os.path

#create output directory, needed before sedobjects imported
if not os.path.isdir("out"):
  os.mkdir("out", 0755)

from sedobjects import *
import lineparser    #provides lineparser.parse(Type, string)

defaultfile = "description.txt"

connectors = []  #list of connectors
containers = []  #list of components and cables
expected = []    #list of parts that have ben referenced but not declared

################################################################################
# things to do after the list is fully parsed, mated, and checked

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

def countConnectors():
  """count how many of each connector type and gender are used as jacks"""
  f = open("out/connector_count.txt", 'w')
  countconn = {}   #counts how many of each connector
  for conn in connectors: countconn[conn.type] = {'M': 0, 'F': 0}
  for cont in containers:
    for jack in cont.jacks:
      if not jack.placeholder:
	name = jack.conn.type
	gender = jack.gender
	matename = jack.conn.mate.type
	mategender = jack.conn.genders[jack.gender]
	countconn[name][gender] += 1
	countconn[matename][mategender] += 1
  f.write("%10s%10s%10s\n" % ("Jack Type", "# Male", "# Female"))
  for conn in connectors:
    if countconn[conn.type]['M'] > 0 or countconn[conn.type]['F'] > 0:
      f.write("%10s%10s%10s\n" % (conn.type, 
	  countconn[conn.type]['M'], countconn[conn.type]['F']))

def rewriteDescription(filename):
  """read and rewrite the description file, adds extra info from parsing"""
  nd = open("out/newdescription.txt", 'w')  #path to new description
  f = open(filename, 'r')
  startWithConnectors = True
  cont = None
  for line in f:
    sline = line.strip()

    if startWithConnectors: #set to false when done
      conn = lineparser.parse(Connector, sline)
      if conn is not None:
	nd.write("%s\n" % str(connectors[connectors.index(conn)]))
      elif sline[0] == '*' and sline == '*ENDCONNECTORLIST':
	startWithConnectors = False
	nd.write(line)
  
    elif sline[0:4] == "JACK":
      #temporarily replace placeholder numbers with mate's, for writing
      jack = cont.jacks[cont.jacks.index(lineparser.parse(Jack, sline))]
      oldnumber = jack.number
      oldmnumber = jack.mate.number
      if jack.placeholder: jack.number = jack.mate.number
      if jack.mate.placeholder: jack.mate.number = jack.number
      nd.write("    %s\n" % str(jack))
      jack.number = oldnumber
      jack.mate.number = oldmnumber

    elif sline[0:4] == "LINE":
      line = lineparser.parse(Line, sline)
      line = cont.lines[cont.lines.index(line)]
      #temporarily replace jack numbers with global references, for writing
      newjacknums = []
      for jacknum in line.jacknums:
	jack = cont.jacks[cont.jacks.index(jacknum)]
	if jack.placeholder: jack = jack.mate
	newjacknums.append("&J%d" % jack.number)
      oldjacknums = line.jacknums
      line.jacknums = newjacknums
      nd.write("    %s\n" % str(line))
      line.jacknums = oldjacknums

    elif sline[0:5] == "CABLE":
      cont = containers[containers.index(lineparser.parse(Cable, sline))]
      nd.write("%s\n" % str(cont))

    elif sline == "" or sline[0] == '#':	#blank or comment, just reprint
      nd.write(line)

    else: #COMPONENT
      cont = containers[containers.index(lineparser.parse(Component, sline))]
      nd.write("%s\n" % str(cont))

################################################################################
# parser logic

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
	  raise Failure("Matching jack (%s/%s) already mated"
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
	jack.mate.cable.label = "%s-%s"%\
	    (jack.mate.location.ref, jack.location.ref)
	print "Warning invented cable name: %s"% jack.mate.cable.label
      else: jack.mate.cable.label = jack.cable.label
    #compare cable numbers
    if jack.cable.ref and jack.mate.cable.ref:
      if jack.cable.ref != jack.mate.cable.ref:
	raise Failure("p2p cable numbers don't match for mating jacks")
    elif not jack.mate.cable.ref:
      jack.mate.cable.ref = jack.cable.ref
      jack.mate.cable.number = jack.cable.number
    jack.cable = jack.mate.cable #unify cable references
    jack.mate.cablemaster = True
  else: jack.placeholder = True #newer jack is always the placeholder

  #for pins on mate (from previous lines) add autogen lines, etc. as needed
  for pin in jack.mate.pins:
    line = pin.lines['box']
    newline = Line(line.desc, "(%s,%s)"%(jack.ref, pin.number))
    newline.autogen = True
    newline.owner = containers[-1]
    containers[-1].lines.append(newline)
    if jack.cable is None:
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
    realjack = jack
    if jack.placeholder: #jack is placeholder in direct-connected pair
      realjack = jack.mate  #use the real jack...its mate
      lineend = 'cable'
    try:  #test pin number
      if jack.conn.flags['alpha'] == 'N' and ((int(pinnum) > jack.conn.count \
	  and jack.conn.count > 0) or int(pinnum) <= 0):
	raise Failure("invalid pin number: %s"%pinnum)
    except ValueError: 
      raise Failure("unexpected non-numeric pin: %s"%pinnum)

    if jack.mate is None: #unmated, add a dangling pin (no cline yet)
      newpin = Pin(pinnum, line.desc, jack, line, None)
      if newpin not in realjack.pins: realjack.pins.append(newpin)
    else: #already mated, need to add extra stuff
      try:  #see if pin has already been generated
	pin = realjack.pins[realjack.pins.index(pinnum)]
      except ValueError:  #pin not yet generated
	newline = Line(line.desc, "(%s,%s)"%(jack.mate.ref,pinnum))
	newline.autogen = True
	newline.owner = jack.mate.location
	jack.mate.location.lines.append(newline)
	if jack.cable is None: #no cable
	  if lineend == 'box': 
	    newpin = Pin(pinnum, line.desc, realjack, line, newline)
	  else: newpin = Pin(pinnum, line.desc, realjack, newline, line) 
	else:  #using a cable, make another new line and pin
	  if lineend == 'cable': raise Failure('inconsistent state')
	  cableline = Line(line.desc, "")
	  cableline.autogen = True
	  cableline.owner = jack.cable
	  newpin = Pin(pinnum, line.desc, jack, line, cableline)
	  newerpin = Pin(pinnum, line.desc, jack.mate, newline, cableline)
	  jack.cable.lines.append(cableline)
	  if newerpin not in jack.mate.pins: jack.mate.pins.append(newerpin)
	realjack.pins.append(newpin)

      else:  #pin exists
	if not pin.lines[lineend].autogen:
	  raise Failure("line conflicts for pin (%s;%s)"%(jacknum,pinnum))
	jack.location.lines.remove(pin.lines[lineend])
	pin.desc = line.desc
	pin.lines[lineend] = line
	#other lines exist already (?), so this case is done

  if line in containers[-1].lines:
    raise Failure("line overlaps existing one")
  containers[-1].lines.append(line)

def sedparser(filename):
  """main function allows calls from interactive prompt, or extra cleverness"""
  startWithConnectors = True #connector list at top. make overridable by args?
  f = open(filename, 'r')

  #lists of used cable and jack numbers
  used_cable_nums = []
  used_jack_nums = []
  
  #main loop, red each line of the description file in turn
  linecount = 0
  for line in f:
    linecount += 1
    #print "On line", linecount
    line = line.strip()

    #CONNECTORS
    if startWithConnectors: #set to false when done
      conn = lineparser.parse(Connector, line)
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
      elif line[0] == '*' and line == '*ENDCONNECTORLIST':
	startWithConnectors = False
	for c in connectors:
	  if c.mate is None: print "Warning: unmated connector", c.type
	print "Info: done parsing connectors, found", len(connectors)
      elif line != "":
	raise Failure("Unrecognized connector", linecount)
  
    elif line[0:4] == "JACK":
      jack = lineparser.parse(Jack, line)
      if jack is None: raise Failure("Bad jack line", linecount)
      if jack in containers[-1].jacks:
	raise Failure("non-unique jack identifier", linecount)
      try: 
	jack.conn = connectors[connectors.index(jack.conn_str[:-2])]
	if jack.conn.genders[jack.gender] == 'X':
	  raise Failure("Invalid connector gender", linecount)
      except ValueError: raise Failure("Non-existant connector type", linecount)
      if jack.internal and hasattr(containers[-1], 'p2p'):
	raise Failure("Cables can't have internal jacks", linecount)
      jack.location = containers[-1]
      containers[-1].jacks.append(jack)
      try: mateJack(jack)
      except Failure, err: raise Failure(err.errstr, linecount)
      if jack.number > 0:
	if not jack.number in used_jack_nums:
	  used_jack_nums.append(jack.number)
	elif not jack.mate or not jack.placeholder:
	  raise Failure("repeated global jack number %d"%jack.number, linecount)
      if jack.cable and jack.cable.number > 0:
	if not jack.cable.number in used_cable_nums:
	  used_cable_nums.append(jack.cable.number)
	elif not jack.mate:
	  raise Failure("repeated cable number %d"%jack.cable.number, linecount)

    elif line[0:4] == "LINE":
      line = lineparser.parse(Line, line)
      if line is None: raise Failure("Bad line line", linecount)
      line.owner = containers[-1]
      try: addPins(line) #catch failures and reraise with line number
      except Failure, err: raise Failure(err.errstr, linecount)

    elif line[0:5] == "CABLE":
      cable = lineparser.parse(Cable, line)
      if cable is None: raise Failure("Bad cable line", linecount)
      if cable in containers: 
	raise Failure("cable exists: rename or use re-edit line", linecount)
      containers.append(cable)
      try: expected.remove(cable)
      except ValueError: pass
      if cable.number > 0:
	if not cable.number in used_cable_nums:
	  used_cable_nums.append(cable.number)
	else:
	  raise Failure("repeated cable number %d"%cable.number, linecount)

    elif line != "" and line[0] != '#': #COMPONENT
      comp = lineparser.parse(Component, line)
      if comp is None:  #check if line is a re-edit command
	try:
	  containers.append(containers.pop(containers.index(line)))
	  print "Info: re-editing", line
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


  #print stats, assign cable and jack numbers, check pins and mating
  for part in expected: print part.ref
  if len(expected) > 0: raise Failure("above parts used but not declared", -1)
  print "Info: done parsing, found", len(containers), "components and cables:"

  sf = open("out/stats.txt", 'w')   #print stats to statfile
  sf.write("%10s%10s%10s\n" % ("Part", "Jacks", "Lines"))
  all_mated = True
  cable_count = 1
  jack_count = 1
  for cont in containers:
    sf.write("%10s%10s%10s\n" % (cont.ref, len(cont.jacks), len(cont.lines)))
    #assign number to unnumbered cables
    if hasattr(cont, 'number') and cont.number <= 0:
      while cable_count in used_cable_nums: cable_count += 1
      cont.number = cable_count
      cable_count += 1
    for ijack in cont.jacks:
      #check number of pins
      if ijack.conn.count > 0 and len(ijack.pins) > ijack.conn.count:
	raise Failure("Jack %s jas too many pins: %d/%d"\
	    %(ijack.ref,len(ijack.pins),ijack.conn.count))
      #check for mating
      if ijack.mate is None: 
	print ("\tJack %s of %s unmated" % (ijack.ref, cont.ref))
	all_mated = False
      #assign number to unnumbered jacks
      if ijack.number <= 0:
	while jack_count in used_jack_nums: jack_count += 1
	ijack.number = jack_count
	jack_count += 1
      #assign number to unnumbered p2p cables
      if ijack.cable and ijack.cablemaster and ijack.cable.number <= 0:
	while cable_count in used_cable_nums: cable_count += 1
	ijack.cable.number = cable_count
	cable_count += 1
  if not all_mated: 
    raise Failure("Not all jacks mated, check for indistinguishable mates")

################################################################################
# main: parse the file, and then do some extra stuff

if __name__ == "__main__":
  try:
    #run the parser
    if len(sys.argv) == 2: filename = sys.argv[1]
    else: filename = defaultfile
    print "Parsing file:", filename
    sedparser(filename)

    #outptus
    writeInfiles()
    countConnectors()
    rewriteDescription(filename)

    print "\nAll Done!"
    sys.exit()
  except Failure, err:
    print >>sys.stderr, err
    sys.exit(1)

