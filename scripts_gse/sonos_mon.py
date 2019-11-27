#!/usr/bin/python
import time
import numpy as np
from datetime import datetime
import pygetdata as gd
import soco

DEVICE_ADDRESS = "192.168.0.140"
DATAFILE = "/data/etc/mole.lnk"

sonos = soco.SoCo(DEVICE_ADDRESS)
#sonos = soco.discovery.any_soco()
df = gd.dirfile(DATAFILE, gd.RDONLY)

# find the tracks

def get_current_track_title():
  return sonos.get_current_track_info()['title']

class AutoSonos:
  def __init__(self, trackname, fieldname, trueval=None):
    self.trackname = trackname
    self.fieldname = fieldname
    self.trueval = trueval
    self.timeout = 0
    self.lastval = 0
    self.framenum = 0
    self.changed = False
    self.timesteady = 0

    self.update()
    self.track = sonos.music_library.get_tracks(search_term=self.trackname)[0]

  # Checks to see if there is new data in the dirfile
  # Increments a timeout when there is no new data
  def has_update(self):
    num = df.eof(self.fieldname)
    retval = False
    if (num != self.framenum):
      self.timeout = 0
      retval = True
    else:
      self.timeout += 1
    self.framenum = num
    return retval

  # General command to be called always in the loop
  # Loads data from dirfile if necessary
  def update(self):
    if (self.has_update()):
      val = df.getdata(self.fieldname,
                       first_frame=0,
                       first_sample=self.framenum-1,
                       num_frames=0,
                       num_samples=1)[0]
      if (val != self.lastval):
        self.changed = True
        self.timesteady = 0
      else:
        self.changed = False
        self.timesteady += 1
      self.lastval = val  

  # Queues the assigned song
  # If the song is already playing, doesn't try to play again 
  # If persist=True, will play even if queue is stopped
  def trigger(self, persist=True, volume=None):
    if (get_current_track_title() != self.track.title) or (persist and sonos.get_current_transport_info()['current_transport_state'] != 'PLAYING'):
      if (sonos.get_current_transport_info()['current_transport_state'] == 'PLAYING'): sonos.pause()
      sonos.add_to_queue(self.track, 1) # add to queue indexing by 1
      sonos.play_from_queue(0, True) # play from queue indexing from 0
      if volume is not None:
        sonos.ramp_to_volume(volume)
      print("Playing " + self.track.title)
      return 1
    return 0

  # Checks if the value in the dirfile has reached the assigned value
  def truestate(self):
    return self.lastval == self.trueval


# Looks at field "STATE_POTVALVE" and .truestate() == True when STATE_POTVALVE == 1
# When the .trigger() function is called, will play the requested song on the SONOS
PotvalveOpen = AutoSonos("Flagpole Sitta", "STATE_POTVALVE", 1)
PotvalveOpen2 = AutoSonos("Ooh Pot Valve Open", "STATE_POTVALVE", 1)
PotvalveYell = True
PotvavleSong = False

HWPMove = AutoSonos("You Spin Me Round", "MOVE_STAT_HWPR", 1)

Lunch = AutoSonos("Sandstorm", "TIME")
Leave = AutoSonos("End of the World as", "TIME")

while True:
  song_requested = False

  # Potvalve open
  # Logic is in the main loop
  PotvalveOpen.update() # must always be called in the loop
  
  # If the truestate is reached and has been true for 20 second, then trigger the song
  if (PotvalveOpen2.truestate() and (PotvalveOpen.timesteady >= 20) and (PotvalveYell == True)):
    print("Potvalve is OPEN!")
    if not song_requested: 
      PotvalveOpen.trigger()
    song_requested = True
    PotvalveYell = False
    PotvavleSong = True

  # If the truestate is reached and has been true for 20 second, then trigger the song
  if (PotvalveOpen.truestate() and (PotvalveOpen.timesteady >= 20) and (PotvalveSong == True)):
    print("Potvalve is OPEN!")
    if not song_requested: 
      PotvalveOpen.trigger()
    song_requested = True
    PotvalveYell = True
    PotvavleSong = False

  # If the HWP move state is 1 we are in state "ready" so we are sending a move command
  if (HWPMove.truestate()):
    print("Half-wave plate is moving!")
    if not song_requested:
      HWPMove.trigger()
    song_requested = True

  # time to leave
  Leave.update()
  date = datetime.utcfromtimestamp(Leave.lastval)
  if ((date.hour+13)%24 == 17) and (15 <= date.minute <= 25):
    print("Time to leave!")
    if not song_requested: 
      Leave.trigger(volume=65)
    song_requested = True

  # lunch
  Lunch.update()
  date = datetime.utcfromtimestamp(Lunch.lastval)
  if ((date.hour+13)%24 == 11) and (date.minute >= 56) and (date.second >= 17) and (datetime.today().isoweekday() != 7):
    print("Lunchtime!")
    if not song_requested: 
      Lunch.trigger(volume=50)
    song_requested = True


  # reload the datafile if necessary
  if (Lunch.timeout > 5):
    df.close()
    df = gd.dirfile(DATAFILE, gd.RDONLY)
    print("Reloading \"" + DATAFILE + "\"")

  time.sleep(1)
