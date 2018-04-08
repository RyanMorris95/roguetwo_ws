#!/usr/bin/env python
import os
from gps import *
from time import *
import time
import threading
import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
 
gpsd = None #seting the global variable
 
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
 
if __name__ == '__main__':
  rospy.init_node("gps_comm")
  gpsp = GpsPoller() # create the thread
  pub_navsat = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)
  try:
    gpsp.start() # start it up
    while not rospy.is_shutdown():
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
      header.frame_id = "base_link"
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

      pub_navsat.publish(navsat)

      time.sleep(0.1) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print "Done.\nExiting."
  import sys

  sys.exit()
