#!/usr/bin/env python

'''
example program to extract GPS data from a waypoint file, and create a GPX
file, for loading into google earth
'''

import sys, struct, time, os

from optparse import OptionParser
parser = OptionParser("wptogpx.py [options]")
(opts, args) = parser.parse_args()

from pymavlink import mavutil, mavwp

if len(args) < 1:
    print("Usage: wptogpx.py <WPFILE>")
    sys.exit(1)

def wp_to_gpx(infilename, outfilename):
    '''convert a wp file to a GPX file'''

    wp = mavwp.MAVWPLoader()
    wp.load(infilename)
    outf = open(outfilename, mode='w')

    def process_wp(w, i):
        t = time.localtime(i)
        outf.write('''<wpt lat="%s" lon="%s">
  <ele>%s</ele>
  <cmt>WP %u</cmt>
</wpt>
''' % (w.x, w.y, w.z, i))

    def add_header():
        outf.write('''<?xml version="1.0" encoding="UTF-8"?>
<gpx
  version="1.0"
  creator="pymavlink"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xmlns="http://www.topografix.com/GPX/1/0"
  xsi:schemaLocation="http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd">
''')

    def add_footer():
        outf.write('''
</gpx>
''')

    add_header()       

    count = 0
    for i in range(wp.count()):
        w = wp.wp(i)
        if w.frame == 3:
            w.z += wp.wp(0).z
        if w.command == 16:
            process_wp(w, i)
        count += 1
    add_footer()
    print("Created %s with %u points" % (outfilename, count))
    

for infilename in args:
    outfilename = infilename + '.gpx'
    wp_to_gpx(infilename, outfilename)
