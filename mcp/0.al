<DOCUMENT>

#--------------------------------------------------------------------------
#
# This is a help file on how to write the .al files for commanding the data
# compression from Frodo.  We will "walk through" an example specification.
#
# See also the readme file for information on the compression algorithms
#
# Adam Hincks
#
#--------------------------------------------------------------------------

# SETTINGS -- this section defines global settings
<SETTINGS>
  
  # GLOBALS
  #   maxbitrate: maximum rate at which SIP can send TDRSS data.  Currently
  #        6000 bps
  #   looplength: length (in seconds) of data to send down in each frame
  #   samplerate: the frequency at which the frames are written to disk on
  #        Frodo (Hz)
  <GLOBALS maxbitrate="6000" looplength="5" samplerate="5"/>

  # COMPRESSION -- compression settings
  #   maxover: ideally, the maximum percentage of data we want not to fit in
  #        the specified data chunk size.  If the more data is spilling over
  #        than maxover, the divider will be adjusted (if forcediv = false).
  #   minover: the ideal minimum value etc.
  <COMPRESSION maxover="11.23" minover="0.25"/>

  # PREDEFINED -- macros to avoid verbosity later.  If many tags share the
  #      same attributes, they can be predefined here
  #   name: a unique identifier for this macro
  #      see further down for explainations of the other attributes -- any
  #      attribute that is used in the SLOWDATA or FASTDATA sections anywhere
  #      may be entered, as few or as many as desired.
  <PREDEFINED name="diff1" numbits="4" overbits="1" divider="3"	forcediv="false"
              perframe="20" samplefreq="1"/>
  <PREDEFINED name="int1" numbits="3" overbits="1" forcediv="false"
              perframe="20" samplefreq="2" divider="1"/>
  <PREDEFINED name="u_to_8bits" numbits="8" overbits="8" perframe="1"
              maxval="0" maxval="65535"/>

</SETTINGS>

# SLOWDATA -- data that are sent down one sample per frame. These data will be 
#      put at the front of the frame.  The order in which they are entered is 
#      of no consequence
#
#   src: name of field to compress (from tx_struct.h)
#   predefined: use the attributes of one of the predefined tags above
#   numbits: number of bits to fit data into
#   overbits: number of bits for overflow chunks (see readme for explanation)
#      If the maxval/minval are chosen properly, there should be no need for
#      these overbits
#   minval/maxval: boundaries on this value -- for best precision, these should
#      be as exact as possible (i.e., smaller range = better precision)
#   perframe: number of samples per frame (currently the only options are 1 for 
#      slow fields and 20 for fast fields).
<SLOWDATA>

  # SINGLE -- send down only one value (the first one) per downlink frame
  <SINGLE src="t_gybox" numbits="11" overbits="3" perframe="20" minval="1024"
          maxval="60000"/>
  <SINGLE src="mcp_frame" numbits="32" overbits="3" perframe="20" minval="0"
          maxval="4294967295"/>

  # AVG -- average all samples and send down the arithmatic mean
  <AVG src="t_pv" predefined="u_to_8bits" minval="16" perframe="1"/>
  
</SLOWDATA>

# FASTDATA -- data that are sent down many samples per frame.  The order in 
#      which these fields is placed IS IMPORTANT -- if there is not enough room 
#      in the downlink, the sending priority is the order in which they are 
#      entered; i.e., the last fields are not included
#
#   src, predefined, numbits, overbits, perframe: same as in SLOWDATA
#   divider: values have to be divided to fit in numbits.  The divider you
#      specify will be the initial divider used for this purpose (you will have
#      to experimentally adjust it to maximise precision and compression).
#      N.B. that the divider must be an integer.
#   forcediv: if true, then the divider you specified will always be used.
#      Otherwise, the compressor will adjust the divider each time to optimise
#      the compression, using maxover and minover (from SETTINGS->COMPRESSION)
#      as a guide
#   samplefreq:  you can choose not to use all available samples, but only to
#      ssnd down, say, every 4th value.  This number MUST BE A POWER OF TWO.
#      If samplefreq is greater than one, a high frequency filter will be
#      applied to the stream before it is sent down
<FASTDATA>
  # DIFF -- differential compression (see readme file for more info)
#  DIFF src="n11c0" predefined="diff1" divider="3"/
  <DIFF src="n7ref" predefined="diff1" divider="4" overbits="2"/>

  # INT -- integral preserving compression (see readme file for more info)
  <INT src="gyro1" predefined="int1" numbits="9" samplefreq="1" divider="2"/>
  <INT src="gyro2" predefined="int1" samplefreq="1" divider="2"/>
  <INT src="gyro3" numbits="3" overbits="1" divider="6" forcediv="false"
       perframe="20" samplefreq="1"/>
  
</FASTDATA>


</DOCUMENT>
