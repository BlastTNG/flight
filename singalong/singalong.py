#!/usr/bin/env /usr/bin/python

import time
import sys
import mpd
import re
import random
import pygetdata as gd
import numpy as np

#setup for the mpd connection
mpdHost = "galadriel.blast"
mpdPort = 6606
mpdPass = "250micron"

#setup for the dirfile
dirfilePath = "/data/etc/defile.lnk"


#region to song mapping
songNameLookup = {
    "VYCma":	  ('title',   "YMCA", \
		   'artist',  "Village People, The"),
    "CG12":	  ('title',   "Don't Stop Believin'", \
		   'artist',  "Journey"),
    "Spearhead":  ('title',   'Theme from "Shaft"', \
		   'artist',  "Hayes, Isaac"),
    "8470":	  ('title',   "Hey Mickey", \
		   'artist',  "Basil, Toni"),
    "Lupus1a":	  ('title',   "Hungry Like the Wolf", \
		   'artist',  "Duran Duran"),
    "Lupus1b":	  ('title',   "Walking on Sunshine", \
		   'artist',  "Katrina & the Waves"),
    "NatsRegion": ('title',   "The Good, the Bad, and the Ugly", \
		   'artist',  "Morricone, Ennio"),
    "IRAS15100":  ('title',   "Home", \
		   'artist',  "Edward Sharpe & The Magnetic Zeros"),
    "Nanten":	  ('title',   "Turning Japanese", \
		   'artist',  "Vapors, The"),
    "M83":	  ('title',   "Eye of the Tiger", \
		   'artist',  "Survivor"),
    "Axehead":	  ('artist',  "U2")
    }

#find songs in the database, make more useful lookup
songLookup = dict()
client = mpd.MPDClient();
client.connect(mpdHost, mpdPort)
client.password(mpdPass)

for region in songNameLookup.iterkeys():
  songs = client.find(*songNameLookup[region])
  songLookup[region] = songs
  print region, len(songs), "song matches"

client.close()
client.disconnect()

#function to pick random song from list for targets
def playSong(key):
  client.connect(mpdHost, mpdPort)
  client.password(mpdPass)
  songs = songLookup[key]
  song = songs[random.randint(0, len(songs)-1)]
  pl = client.playlist()
  if song['file'] in pl:
    client.play(pl.index(song['file']))
  else:
    client.add(song['file'])
    client.play(len(pl))      #length one higher now, with added song
  client.close()
  client.disconnect()

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

print "\nSchedule parsed with", len(schedule), "command lines"
print
print "************************************************************"
print "* Starting main loop, waiting for new scans                *"
print "************************************************************"
print

#main loop. read LST_SCHED and if it crosses a schedule threshold, play the song
df = gd.dirfile(dirfilePath, gd.RDONLY)
try:
  while True:
    lst = df.getdata("LST_SCHED", gd.FLOAT, \
	first_frame=df.nframes-1, num_frames=1)
    sch0 = schedule[0]
    newsong = False
    while lst > sch0[0]:
      try:
	sch0 = schedule.pop(0)
      except IndexError:
	print "Schedule out of commands! Do something! Forget about music!"
	sys.exit(1)
      print "Passed threshold", sch0[0], "<", lst, "for region", sch0[1]
      newsong = True
    if newsong:
      #can't do direct songLookup access because names may have modifier chars
      for region in songLookup.keys:
	if sch0[1].find(region) >= 0:
	  print "New song!", sch0[1], "matches", region
	  playSong(region)
	  break
      else:
	print "Could not find song match for", sch0[1]
    time.sleep(5)

except KeyboardInterrupt:
  df.close()

