#!/usr/bin/env /usr/bin/python

import time
import sys
import mpd
import re
import pygetdata as gd
import numpy as np

client = mpd.MPDClient();
client.connect("galadriel.blast", 6606)
client.password("250micron")

df = gd.dirfile('/data/etc/defile.lnk', gd.RDONLY)

songNameLookup = {
    "VYCma":	  ("YMCA", "Village People, The"),
    "CG12":	  ("Don't Stop Believin'", "Journey"),
    "Spearhead":  ('Theme from "Shaft"', "Hayes, Isaac"),
    "8470":	  ("Hey Mickey", "Basil, Toni"),
    "Lupus1a":	  ("Hungry Like the Wolf", "Duran Duran"),
    "Lupus1b":	  ("Walking on Sunshine", "Katrina & the Waves"),
    "NatsRegion": ("The Good, the Bad, and the Ugly", "Morricone, Ennio"),
    "IRAS15100":  ("Home", "Edward Sharpe & The Magnetic Zeros"),
    "Nanten":	  ("Turning Japanese", "Vapors, The"),
    "M83":	  ("Eye of the Tiger", "Survivor")
    }

#lookup the songs in the database
songLookup = dict()
for region in songNameLookup.iterkeys():
  songlook = songNameLookup[region]
  song = client.find('title', songlook[0], 'artist', songlook[1])[0]
  songLookup[region] = song
  print region, song, "\n"

#parse schedule file passed as command line argument
schedparser = re.compile(r"\w+\s+(?P<day>[0-9.]+)\s+(?P<hour>[0-9.]+)[^#]*#\s*(?P<name>\S*).*")
schedule = [];
with open(sys.argv[1]) as f:
  for line in f:
    match = schedparser.match(line)
    if (match):
      lst =  (float(match.group("day"))*24*3600 \
	  + float(match.group("hour"))*3600)
      schedule.append( (lst, match.group("name")) )

print "Schedule parsed with", len(schedule), "command lines"
print
print "************************************************************"
print "* Starting main loop, waiting for new scans                *"
print "************************************************************"
print

try:
  #loop. read LST_SCHED and if it crosses a schedule threshold, play the song
  while True:
    lst = df.getdata("LST_SCHED", gd.FLOAT, \
	first_frame=df.nframes-1, num_frames=1)
    print "lst: ", lst
    sch0 = schedule[0]
    newsong = False
    while lst > sch0[0]:
      try:
	sch0 = schedule.pop(0)
      except IndexError:
	print "Schedule out of commands! Do something!"
	sys.exit(1)
      print "Passed threshold", sch0[0], "<", lst, "for region", sch0[1]
      newsong = True
    if newsong:
      #can't do direct songLookup access because names may have modifier chars
      for region in songLookup.keys:
	if sch0[1].find(region) >= 0:
	  print "New song!", sch0[1], "matches", region
	  song = songLookup[region]
	  pl = client.playlist()
	  if song['file'] in pl:
	    client.play(pl.index(song['file']))
	  else:
	    client.add(song['file'])
	    client.play(len(pl))      #length one higher now, with added song
	  break
      else:
	print "Could not find song match for", sch0[1]
    time.sleep(5)

except KeyboardInterrupt:
  df.close()
  client.close()
  client.disconnect();

