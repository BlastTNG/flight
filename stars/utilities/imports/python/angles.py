from pylab import pi

def from_hours(angle):
    return angle*pi/12.

def to_hours(angle):
    return angle*12./pi

def from_degrees(angle):
    return angle*pi/180.

def to_degrees(angle):
    return angle*180./pi

def to_arcmin(angle):
    return angle*180./pi*60.

def from_arcmin(angle):
    return angle/60.*pi/180.

def to_arcsec(angle):
    return angle*180./pi*3600.

def from_arcsec(angle):
    return angle/3600.*pi/180.

