#!/usr/bin/env python
import os
from gps import *
from time import *
import time
import threading
import rospy
import geonav_conversions

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

gpsd = None #seting the global variable
origin_lat = None
origin_lon = None
first = False

os.system('clear') #clear the terminal (optional)
 
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
 
def publish_gps(event):
    global origin_lat, origin_lon, first
    #It may take a second or two to get good data
    #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc

    os.system('clear')

    print
    print ' GPS reading'
    print '----------------------------------------'
    print 'latitude    ' , gpsd.fix.latitude
    print 'longitude   ' , gpsd.fix.longitude
    print 'time utc    ' , gpsd.utc,' + ', gpsd.fix.time
    print 'altitude (m)' , gpsd.fix.altitude
    print 'epx         ' , gpsd.fix.epx
    print 'epv         ' , gpsd.fix.epv
    print 'ept         ' , gpsd.fix.ept
    print 'speed (m/s) ' , gpsd.fix.speed
    print 'climb       ' , gpsd.fix.climb
    print 'track       ' , gpsd.fix.track
    print 'mode        ' , gpsd.fix.mode
    print
    print 'sats        ' , gpsd.satellites

    navsat = NavSatFix()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_footprint"
    navsat.header = header
    navsat_status = NavSatStatus()
    navsat_status.status = 0
    navsat_status.service = 1
    navsat.status = navsat_status
    navsat.latitude = gpsd.fix.latitude
    navsat.longitude = gpsd.fix.longitude
    navsat.altitude = gpsd.fix.altitude
    navsat.position_covariance_type = 2
    navsat.position_covariance = [2.5, 0, 0,
                                  0, 2.5, 0,
                                  0, 0, 2.5]

    if not origin_lat and not origin_lon:
      origin_lat = gpsd.fix.latitude
      origin_lon = gpsd.fix.longitude

    (x,y) = geonav_conversions.ll2xy(gpsd.fix.latitude,
                                    gpsd.fix.longitude,
                                    origin_lat,
                                    origin_lon)
    print ('Odometry: ')
    print (x, y)

    odometry = Odometry()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_footprint"
    odometry.header = header
    odometry.child_frame_id = "base_link"

    pub_navsat.publish(navsat)

if __name__ == '__main__':
  rospy.init_node("gps_comm")
  gpsp = GpsPoller() # create the thread
  pub_navsat = rospy.Publisher("/gps", NavSatFix, queue_size=1)

  timer = rospy.Timer(rospy.Duration(0.1), publish_gps)
  try:
    gpsp.start() # start it up
    while not rospy.is_shutdown():
      pass
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
    rospy.signal_shutdown()
    os._exit()
  print "Done.\nExiting."

