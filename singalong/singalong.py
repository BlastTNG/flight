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
dirfilePath = "/data/etc/fox.lnk"


#region to song mapping
songNameLookup = {
    "VYCma":	  ('title',   "YMCA", \
		   'artist',  "Village People, The"),
    "CG12":	  ('title',   "Don't Stop Believin'", \
		   'artist',  "Journey"),
    "Spearhead":  ('title',   'Theme from "Shaft"', \
		   'artist',  "Hayes, Isaac"),
    "Mickey":	  ('title',   "Hey Mickey", \
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
    "Axehead":	  ('artist',  "U2"),
    "CarinaTan":  ('title',   "O... Saya", \
		   'artist',  "A.R. Rahman & M.I.A."),
    "G333":	  ('title',   "Not Ready to Make Nice", \
		   'artist',  "Dixie Chicks"),
    "G331":	  ('title',   "The Cave", \
		   'artist',  "Albarn, Damon / Nyman, Michael"),
    "G3219":	  ('title',   "To The Dogs Or Whoever", \
		   'artist',  "Josh Ritter"),
    "G3199":	  ('album',   "Thriller", \
		   'artist',  "Jackson, Michael"),
    "G3047":	  ('title',   "Oxford Comma", \
		   'artist',  "Vampire Weekend"),
    "G3237":	  ('title',   "Blame Canada", \
		   'album',   "South Park - Bigger, Longer, & Uncut"),
    "G3265":	  ('title',   "Hallelujah", \
		   'artist',  "Cohen, Leonard", \
		   'album',   "Various Positions"),
    "G3290":	  ('title',   "Throne Room and Finale", \
	      'artist', "Williams, John and the Skywalker Symphony Orchestra"),
    "G3168":	  ('title',   "Awesome God", \
		   'artist',  "Praise Band"),
    "CenA":	  ('title',   'Danger Zone (Theme from "Top Gun")', \
		   'artist',  "Loggins, Kenny"),
    "NGC4945":	  ('title',   "White Winter Hymnal", \
		   'artist',  "Fleet Foxes", \
		   'album',   "Fleet Foxes")
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
  cur = client.currentsong()
  if(pl and cur):
    idx = client.playlist().index(cur['file'])
    client.addid(song['file'], idx+1)
    client.play(idx+1)      #length one higher now, with added song
  else:
    client.add(song['file'])
    client.play(len(pl))
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
try:
  while True:
    df = gd.dirfile(dirfilePath, gd.RDONLY)
    lst = df.getdata("LST_SCHED", gd.FLOAT, \
	first_frame=df.nframes-1, num_frames=1)[0]
    df.close()
    newsong = False
    try:
      while lst > schedule[1][0]:
	schedule.pop(0)
	print "Passed threshold", schedule[0][0], "<", lst, \
	    "for region", schedule[0][1]
	newsong = True
      if newsong:
	#can't do direct songLookup access because names may have modifier chars
	for region in songLookup.iterkeys():
	  if schedule[0][1].find(region) >= 0:
	    print "New song!", schedule[0][1], "matches", region
	    playSong(region)
	    break
	else:
	  print "Could not find song match for", schedule[0][1]
      time.sleep(1)
    except IndexError:
      print "Schedule out of commands! Do something! Forget about music!"
      sys.exit(1)

except KeyboardInterrupt:
  print "bye"

