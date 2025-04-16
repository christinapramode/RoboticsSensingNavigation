#!/usr/bin/env python3
import rospy
import serial
import rosbag
import utm
import os
import rospkg
from datetime import datetime, timezone
from gps_driver.msg import Customrtk

# Function to convert Lat/Lon in DDmm.mm to DD.dddd
def degMinstoDegDec(LatOrLong):
    deg = int(LatOrLong) // 100
    mins = LatOrLong - (deg * 100)
    degDec = mins / 60.0
    return (deg+degDec)

# Function to add sign to Lat/Lon
def LatLongSignConvention(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W" or LatOrLongDir == "S":
        LatOrLong = - LatOrLong
    return LatOrLong

# Function to convert signed Lat/Lon to UTM format
def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

# Function to convert GPGGA UTC time to UTC Epoch time
def UTCtoUTCEpoch(UTC):  
    # Converting the GPPGA float input to hrs, mins, secs
    hrs = int(UTC / 10000)
    mins = int((UTC - (hrs * 10000)) / 100)
    secs = UTC - (hrs * 10000) - (mins * 100)
    UTCinSecs = hrs*60*60 + mins*60 + secs

    # Getting the total seconds since the epoch
    CurrentUTCTime = datetime.utcnow()
    MidnightUTC = datetime.combine(CurrentUTCTime, datetime.min.time(), tzinfo=timezone.utc)    
    TimeSinceEpochBOD = MidnightUTC.timestamp()
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int(round(CurrentTime - CurrentTimeSec, 2) * pow(10, 9))
    return [CurrentTimeSec, CurrentTimeNsec]

if __name__ == '__main__':
    rospy.init_node('rtk_driver')
    
    # Setting the serial port address
    serialPortAddr = rospy.get_param('~port','/dev/pts/0')
    serialPort = serial.Serial(serialPortAddr, 4800)
    rospy.logdebug("Taking data from port " + serialPortAddr)

    # Set the custom msg publisher 
    gngga_msg_pub = rospy.Publisher('rtk_gnss', Customrtk, queue_size=5)

    # Get the path to the Catkin workspace
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('gps_driver')
    catkin_ws_path = os.path.dirname(os.path.dirname(package_path))

    # Defining the output bag file
    bag = rosbag.Bag(catkin_ws_path+'/data/rtk_output.bag', 'w')

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing gngga message...")

    # Defining the custom msg
    gngga_msg = Customrtk()
   
    try:
        while not rospy.is_shutdown():
            # Reading the string from the source
            gnggaRead = str(serialPort.readline())
            rospy.loginfo(gnggaRead)
            
            if gnggaRead == '':
                rospy.logwarn("No data")
            else:
                if "$GNGGA" in gnggaRead:              
                    
                    # Split the gngga string and assign to variables
                    try:
                        gnggaSplit = gnggaRead.split(',')
                        UTC = float(gnggaSplit[1])
                        Latitude = float(gnggaSplit[2])
                        LatitudeDir = gnggaSplit[3]
                        Longitude = float(gnggaSplit[4])
                        LongitudeDir = gnggaSplit[5]
                        FixQuality = int(gnggaSplit[6])
                        HDOP = float(gnggaSplit[8])
                        Altitude = float(gnggaSplit[9])  
                    except ValueError as e:
                        continue             

                    # Convert the Latitude/Longitude to UTM format
                    Latitude = degMinstoDegDec(Latitude)
                    Longitude = degMinstoDegDec(Longitude)
                    LatitudeSigned = LatLongSignConvention(Latitude, LatitudeDir)
                    LongitudeSigned = LatLongSignConvention(Longitude, LongitudeDir)
                    UTMEasting, UTMNorthing, UTMZone, UTMLetter = convertToUTM(LatitudeSigned, LongitudeSigned)
                    
                    # Convert UTC time to UTC epoch time
                    CurrentTimeSec, CurrentTimeNsec = UTCtoUTCEpoch(UTC)

                    # Initializing the custom gppga msg
                    gngga_msg.header.frame_id = 'RTK1_Frame'
                    gngga_msg.header.stamp = rospy.Time(CurrentTimeSec, CurrentTimeNsec)
                    gngga_msg.latitude = LatitudeSigned
                    gngga_msg.longitude = LongitudeSigned
                    gngga_msg.altitude = Altitude
                    gngga_msg.utm_easting = UTMEasting
                    gngga_msg.utm_northing = UTMNorthing
                    gngga_msg.zone = UTMZone
                    gngga_msg.letter = UTMLetter
                    gngga_msg.hdop = HDOP
                    gngga_msg.gngga_read = gnggaRead 
                    gngga_msg.fix_quality = FixQuality

                    # Publish the custom msg to the topic & write it to a bag
                    gngga_msg_pub.publish(gngga_msg)
                    print(gngga_msg)
                    bag.write('rtk_gnss', gngga_msg)
            
            rospy.sleep(1.0)
        bag.close()
            
    except rospy.ROSInterruptException:
        serialPort.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down rtk_driver node...")