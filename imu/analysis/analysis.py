#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
from vn_driver.msg import Vectornav
import os, rospkg
from geometry_msgs.msg import Quaternion, Vector3

def QuaterniontoEuler(q):
    # Calculate Euler angles
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = math.asin(2 * (q.w * q.y - q.z * q.x))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    # Convert angles to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    return [roll, pitch, yaw]

def ExtractAttributesfromMsg(msg_list):
    # Setting the attribute lists
    Gyro = np.zeros((3, len(msg_list)))
    Acceleration = np.zeros((3, len(msg_list)))
    EulerAngles = np.zeros((3, len(msg_list)))
    Time = np.zeros(len(msg_list))

    for i in range(len(msg_list)):
        msg = msg_list[i]
        Gyro[0][i] = msg.imu.angular_velocity.x
        Gyro[1][i] = msg.imu.angular_velocity.y
        Gyro[2][i] = msg.imu.angular_velocity.z
        Acceleration[0][i] = msg.imu.linear_acceleration.x
        Acceleration[1][i] = msg.imu.linear_acceleration.y
        Acceleration[2][i] = msg.imu.linear_acceleration.z
        Time[i] = msg.header.stamp.to_sec()

        # Save Euler angles to numpy array
        q = Quaternion(msg.imu.orientation.x,msg.imu.orientation.y,msg.imu.orientation.z,msg.imu.orientation.w)
        EulerAngles[:,i] = QuaterniontoEuler(q)
        
    # Convert Rotational rate from rad/sec to degrees/sec
    Gyro_degpsec = 180/math.pi * Gyro
    
    # Normalize Time
    Time_normalized = np.subtract(Time, Time[0])

    return [Gyro_degpsec, Acceleration, EulerAngles, Time_normalized]    


# Get the path to the Catkin workspace
rospack = rospkg.RosPack()
package_path = rospack.get_path('vn_driver')
catkin_ws_path = os.path.dirname(os.path.dirname(package_path))

# Defining the output bag file
bag = rosbag.Bag(catkin_ws_path+'/data/stationary_10min.bag', 'r') 

# Initialize list for data  
msg_list = []

# Iterate through messages in the specified topic
for topic, msg, t in bag.read_messages(topics='imu'):
    # Copy data from bag to msg list
    msg_list.append(msg)
        
bag.close()

# Get attributes list of each data
Gyro, Acceleration, EulerAngles, Time = ExtractAttributesfromMsg(msg_list)

# Plot Rotational Rate vs Time
fig1, axs = plt.subplots(3, 1, figsize=(14, 14))
fig1.subplots_adjust(hspace=0.8)
axs[0].plot(Time, Gyro[0,:], color='blue')
axs[0].set_xlabel('Time, sec')
axs[0].set_ylabel('Rotational Rate,\n deg/sec')
axs[0].set_title('Gyroscope X Rotational Rate')
axs[0].grid()
axs[1].plot(Time, Gyro[1,:], color='orange')
axs[1].set_xlabel('Time, sec')
axs[1].set_ylabel('Rotational Rate,\n deg/sec')
axs[1].set_title('Gyroscope Y Rotational Rate')
axs[1].grid()
axs[2].plot(Time, Gyro[2,:], color='green')
axs[2].set_xlabel('Time, sec')
axs[2].set_ylabel('Rotational Rate,\n deg/sec')
axs[2].set_title('Gyroscope Z Rotational Rate')
axs[2].grid()
fig1.savefig('fig0')

# Plot Acceleration vs Time
fig2, ax2 = plt.subplots()
ax2.plot(Time, Acceleration[0,:], label='X', color='blue')
ax2.plot(Time, Acceleration[1,:], label='Y', color='orange')
ax2.plot(Time, Acceleration[2,:], label='Z', color='green')
ax2.set_xlabel('Time, sec')
ax2.set_ylabel('Acceleration, m/s\u00b2')
ax2.legend()
ax2.set_title('Acceleration vs Time')
ax2.grid()
fig2.savefig('fig1')

# Plot Euler angles vs Time
fig3, ax3 = plt.subplots()
ax3.plot(Time, EulerAngles[0,:], label='Roll, X', color='blue')
ax3.plot(Time, EulerAngles[1,:], label='Pitch, Y', color='orange')
ax3.plot(Time, EulerAngles[2,:], label='Yaw, Z', color='green')
ax3.set_xlabel('Time, sec')
ax3.set_ylabel('Rotation, degrees')
ax3.legend()
ax3.set_title('Rotation vs Time')
ax3.grid()
fig3.savefig('fig2')

# Histogram plots for Euler angles
fig4, axs = plt.subplots(1, 3, figsize=(14, 4))
fig4.subplots_adjust(wspace=0.4)
axs[0].hist(EulerAngles[0], bins=30, color='blue', alpha=0.7, edgecolor='black')
axs[0].set_title('Histogram of Rotation in X-axis (Roll)')
axs[0].set_xlabel('X Rotation, degrees')
axs[0].set_ylabel('Frequency')
axs[1].hist(EulerAngles[1], bins=30, color='orange', alpha=0.7, edgecolor='black')
axs[1].set_title('Histogram of Rotation in Y-axis (Pitch)')
axs[1].set_xlabel('Y Rotation, degrees')
axs[1].set_ylabel('Frequency')
axs[2].hist(EulerAngles[2], bins=30, color='green', alpha=0.7, edgecolor='black')
axs[2].set_title('Histogram of Rotation in Z-axis (Yaw)')
axs[2].set_xlabel('Z Rotation, degrees')
axs[2].set_ylabel('Frequency')
fig4.savefig('fig3')

plt.show()
