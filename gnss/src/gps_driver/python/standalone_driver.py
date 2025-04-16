#!/usr/bin/env python3
import rospy
import utm
import serial
import rosbag
import os
import rospkg
from datetime import datetime, timezone
from gps_driver.msg import Customgps

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
    rospy.init_node('driver')
    
    # Setting the serial port address
    serialPortAddr = rospy.get_param('~port','/dev/pts/0')
    serialPort = serial.Serial(serialPortAddr, 4800)
    rospy.logdebug("Taking data from port " + serialPortAddr)

    # Set the custom msg publisher 
    gpgga_msg_pub = rospy.Publisher('gps', Customgps, queue_size=5)

    # Get the path to the Catkin workspace
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('gps_driver')
    catkin_ws_path = os.path.dirname(os.path.dirname(package_path))

    # Defining the output bag file
    bag = rosbag.Bag(catkin_ws_path+'/data/output.bag', 'w')

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing gpgga message...")

    # Defining the custom msg
    gpgga_msg = Customgps()
   
    try:
        while not rospy.is_shutdown():
            # Reading the string from the source
            gpggaRead = str(serialPort.readline())
            rospy.loginfo(gpggaRead)
            
            if gpggaRead == '':
                rospy.logwarn("No data")
            else:
                if "$GPGGA" in gpggaRead:              
                    
                    # Split the gpgga string and assign to variables
                    gpggaSplit = gpggaRead.split(',')
                    UTC = float(gpggaSplit[1])
                    Latitude = float(gpggaSplit[2])
                    LatitudeDir = gpggaSplit[3]
                    Longitude = float(gpggaSplit[4])
                    LongitudeDir = gpggaSplit[5]
                    HDOP = float(gpggaSplit[8])
                    Altitude = float(gpggaSplit[9])                    

                    # Convert the Latitude/Longitude to UTM format
                    Latitude = degMinstoDegDec(Latitude)
                    Longitude = degMinstoDegDec(Longitude)
                    LatitudeSigned = LatLongSignConvention(Latitude, LatitudeDir)
                    LongitudeSigned = LatLongSignConvention(Longitude, LongitudeDir)
                    UTMEasting, UTMNorthing, UTMZone, UTMLetter = convertToUTM(LatitudeSigned, LongitudeSigned)
                    
                    # Convert UTC time to UTC epoch time
                    CurrentTimeSec, CurrentTimeNsec = UTCtoUTCEpoch(UTC)

                    # Initializing the custom gppga msg
                    gpgga_msg.header.frame_id = 'GPS1_Frame'
                    gpgga_msg.header.stamp = rospy.Time(CurrentTimeSec, CurrentTimeNsec)
                    gpgga_msg.latitude = LatitudeSigned
                    gpgga_msg.longitude = LongitudeSigned
                    gpgga_msg.altitude = Altitude
                    gpgga_msg.utm_easting = UTMEasting
                    gpgga_msg.utm_northing = UTMNorthing
                    gpgga_msg.zone = UTMZone
                    gpgga_msg.letter = UTMLetter
                    gpgga_msg.hdop = HDOP
                    gpgga_msg.gpgga_read = gpggaRead 

                    # Publish the custom msg to the topic & write it to a bag
                    gpgga_msg_pub.publish(gpgga_msg)
                    bag.write('gps', gpgga_msg)
            
            rospy.sleep(1.0)
        bag.close()
            
    except rospy.ROSInterruptException:
        serialPort.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down driver node...")