#!/usr/bin/env python3
import rospy
import serial
import rosbag
import numpy as np
import re
import math
import rospkg, os
from vn_driver.msg import Vectornav
from geometry_msgs.msg import Quaternion, Vector3

# Function to convert from Euler angles to Quaternion
def ConverttoQuaternion(YPR):
    # Convert Euler angles to radians
    roll = math.radians(YPR[2])
    pitch = math.radians(YPR[1])
    yaw = math.radians(YPR[0])

    # Calculate Directional Cosine Matrix
    R1 = np.array([[1, 0, 0],
                   [0, math.cos(roll), math.sin(roll)],
                   [0, -math.sin(roll), math.cos(roll)]])
    R2 = np.array([[math.cos(pitch), 0, -math.sin(pitch)],
                   [0, 1, 0],
                   [math.sin(pitch), 0, math.cos(pitch)]])
    R3 = np.array([[math.cos(yaw), math.sin(yaw), 0],
                   [-math.sin(yaw), math.cos(yaw), 0],
                   [0, 0, 1]])
    C321 = np.dot(R1, np.dot(R2, R3))

    # Calculate quaternions
    q = Quaternion()
    q.w = 0.5 * math.sqrt(1 + C321[0][0] + C321[1][1] + C321[2][2])
    q.x = (C321[1][2] - C321[2][1]) / (4*q.w)
    q.y = (C321[2][0] - C321[0][2]) / (4*q.w)
    q.z = (C321[0][1] - C321[1][0]) / (4*q.w)
    return q


if __name__ == '__main__':
    rospy.init_node('driver')
    
    # Setting the serial port address
    serialPortAddr = rospy.get_param('~port','/dev/pts/0')
    serialPort = serial.Serial(serialPortAddr, 115200)
    rospy.logdebug("Taking data from port " + serialPortAddr)

    # Set VectorNav to get VNYMR output at 40Hz
    serialPort.write(str('$VNWRG,06,14*XX').encode('utf8') + b'\r\n')
    serialPort.write(str('$VNWRG,07,40*XX').encode('utf8') + b'\r\n')

    # Set the custom msg publisher 
    msg_pub = rospy.Publisher('imu', Vectornav, queue_size=5)

    # Get the path to the Catkin workspace
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('vn_driver')
    catkin_ws_path = os.path.dirname(os.path.dirname(package_path))

    # Defining the output bag file
    bag = rosbag.Bag(catkin_ws_path+'/data/imu_data.bag', 'w')

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing message...")

    # Defining the custom msg
    vectornav_msg = Vectornav()

    # Set the desired loop rate (in Hz)
    loop_rate = rospy.Rate(40)
   
    try:
        while not rospy.is_shutdown():
            # Reading the string from the source
            portRead = serialPort.readline()
            vnymrRead = portRead.decode('utf8')
            rospy.loginfo(vnymrRead)

            if "$VNYMR" in vnymrRead: 

                try:
                    # Extracting attributes from the string
                    msgSplit = re.split('[,*]', vnymrRead)
                    eulerAngles = np.array([float(msgSplit[1]),float(msgSplit[2]),float(msgSplit[3])])   # Yaw, Pitch, Roll in degrees 
                    magnetometer = np.array([float(msgSplit[4]),float(msgSplit[5]),float(msgSplit[6])])  # Magnetic Field in Gauss
                    acceleration = np.array([float(msgSplit[7]),float(msgSplit[8]),float(msgSplit[9])])  # Acceleration in m/s^2
                    gyro = np.array([float(msgSplit[10]),float(msgSplit[11]),float(msgSplit[12])])       # Angular rate in rad/sec              
                except:
                    continue

                # Populating attributes of the custom ros msg
                vectornav_msg.header.frame_id = 'imu1_frame'
                vectornav_msg.header.stamp = rospy.Time.now()
                vectornav_msg.imu.header = vectornav_msg.header
                vectornav_msg.imu.orientation = ConverttoQuaternion(eulerAngles)
                vectornav_msg.imu.angular_velocity = Vector3(gyro[0], gyro[1], gyro[2])
                vectornav_msg.imu.linear_acceleration = Vector3(acceleration[0], acceleration[1], acceleration[2])
                vectornav_msg.mag_field.header = vectornav_msg.header
                vectornav_msg.mag_field.magnetic_field = Vector3(magnetometer[0]/pow(10,4), magnetometer[1]/pow(10,4), magnetometer[2]/pow(10,4))
                vectornav_msg.raw_data = vnymrRead

                # Publish the custom msg to the topic & write it to a bag
                msg_pub.publish(vectornav_msg)
                bag.write('imu', vectornav_msg)

            loop_rate.sleep()
        bag.close()
            
    except rospy.ROSInterruptException:
        serialPort.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down driver node...")