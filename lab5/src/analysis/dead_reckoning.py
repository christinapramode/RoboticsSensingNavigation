#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math, os, rospkg
from scipy.optimize import least_squares
import scipy.integrate as integrate 
from scipy.signal import butter, filtfilt
from lab5.msg import Vectornav
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import euler_from_quaternion

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

def ExtractAttributesfromIMUMsg(msg_list):
    # Setting the attribute lists
    Gyro = np.zeros((3, len(msg_list)))
    Acceleration = np.zeros((3, len(msg_list)))
    EulerAngles = np.zeros((3, len(msg_list)))
    MagField = np.zeros((3, len(msg_list)))
    Time = np.zeros(len(msg_list))

    for i in range(len(msg_list)):
        msg = msg_list[i]
        Gyro[0][i] = msg.imu.angular_velocity.x
        Gyro[1][i] = msg.imu.angular_velocity.y
        Gyro[2][i] = msg.imu.angular_velocity.z
        Acceleration[0][i] = msg.imu.linear_acceleration.x
        Acceleration[1][i] = msg.imu.linear_acceleration.y
        Acceleration[2][i] = msg.imu.linear_acceleration.z
        MagField[0][i] = msg.mag_field.magnetic_field.x
        MagField[1][i] = msg.mag_field.magnetic_field.y
        MagField[2][i] = msg.mag_field.magnetic_field.z        
        Time[i] = msg.header.stamp.to_sec()

        # Save Euler angles to numpy array
        q = Quaternion(msg.imu.orientation.x,msg.imu.orientation.y,msg.imu.orientation.z,msg.imu.orientation.w)
        EulerAngles[:,i] = QuaterniontoEuler(q)

    # Convert Rotational rate from rad/sec to degrees/sec
    Gyro_degpsec = 180/math.pi * Gyro
    
    # Normalize Time
    Time_normalized = np.subtract(Time, Time[0])

    return [Gyro_degpsec, Acceleration, EulerAngles, MagField, Time_normalized]    

def ExtractAttributesfromGPSMsg(msg_list):
    # Setting the attribute lists
    UTMEasting = np.zeros(len(msg_list))
    UTMNorthing = np.zeros(len(msg_list))
    Latitude = np.zeros(len(msg_list))
    Longitude = np.zeros(len(msg_list))
    altitude = np.zeros(len(msg_list))
    time = np.zeros(len(msg_list))

    for i in range(len(msg_list)):
        msg = msg_list[i]
        UTMEasting[i] = msg.utm_easting
        UTMNorthing[i] = msg.utm_northing
        Latitude[i] = msg.latitude
        Longitude[i] = msg.longitude
        altitude[i] = msg.altitude
        time[i] = msg.header.stamp.to_sec()

    # Normalize Time
    Time_normalized = np.subtract(time, time[0])

    return [UTMEasting, UTMNorthing, Latitude, Longitude, Time_normalized]

def butter_filter(data, cutoff_freq, fs, filter_type, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    y = filtfilt(b, a, data)
    return y

def DeadReckoning(GyroZ, VelX, Time):
    xe = xn = theta = 0
    Trajectory = [[xe, xn]]

    for i in range(1, len(Time)):
        dt = Time[i] - Time[i-1]
        omega = GyroZ[i]
        delta_theta = omega * dt
        theta += delta_theta

        delta_e = np.cos(theta) * VelX[i] * dt
        delta_n = np.sin(theta) * VelX[i] * dt
        xe += delta_e
        xn += delta_n

        Trajectory.append([xe, xn])

    return np.array(Trajectory)

def AlignTrajectories(Traj1, Traj2):
    # Find translation
    translation = Traj2[0] - Traj1[0]
    
    # Find rotation angle
    theta1 = np.arctan2(Traj1[1][1] - Traj1[0][1], Traj1[1][0] - Traj1[0][0])
    theta2 = np.arctan2(Traj2[1][1] - Traj2[0][1], Traj2[1][0] - Traj2[0][0])
    angle = theta2 - theta1
    
    # Rotate Trajectory1 to align with Trajectory2
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
    [np.sin(angle), np.cos(angle)]])
    AlignedTraj = np.dot(Traj1, rotation_matrix.T) + translation
    return AlignedTraj


# Get the path to the Catkin workspace
rospack = rospkg.RosPack()
package_path = rospack.get_path('lab5')
catkin_ws_path = os.path.dirname(os.path.dirname(package_path))

# Bag names of datasets
bag_list = ['data_going_in_circles', 'data_driving']

# Initialize lists for each dataset
circle_msg_list = []
imu_msg_list = []
gps_msg_list = []

for bag_name in bag_list:
    # Opening each bag
    bag = rosbag.Bag(catkin_ws_path+'/data/'+bag_name+'.bag', 'r')   
    # Iterate through messages in the specified topic
    for topic, msg, t in bag.read_messages(topics='imu'):
        # Copy data from bag to respective msg list
        if bag_name == 'data_going_in_circles':
            circle_msg_list.append(msg) 
        elif bag_name == 'data_driving':
            imu_msg_list.append(msg) 
    for topic, msg, t in bag.read_messages(topics='gps'):
        # Copy data from bag to respective msg list
        if bag_name == 'data_driving':
            gps_msg_list.append(msg) 
    bag.close()

# Get attributes list of each data
_, _, _, cMagField, _ = ExtractAttributesfromIMUMsg(circle_msg_list)
Gyro, Acceleration, EulerAngles, MagField, Time_IMU = ExtractAttributesfromIMUMsg(imu_msg_list)
UTMEasting, UTMNorthing, _, _, Time_GPS = ExtractAttributesfromGPSMsg(gps_msg_list)

# Apply high-pass filter to remove bias
sampl_freq = 1 / (Time_IMU[1] - Time_IMU[0])
AccX_filtered = butter_filter(Acceleration[0], 0.01, sampl_freq, 'highpass')

# Integrate filtered acceleration to get filtered velocity
Vx_filtered = np.cumsum(AccX_filtered) / sampl_freq
Vx_filtered = np.maximum(Vx_filtered, 0)

# Calculate IMU Trajectory
Trajectory_IMU = DeadReckoning(np.radians(Gyro[2]), Vx_filtered, Time_IMU)

# Align IMU trajectory with GPS trajectory
Trajectory_GPS = np.array([UTMEasting, UTMNorthing]).T
AlignedTrajectory_IMU = AlignTrajectories(Trajectory_IMU, Trajectory_GPS)

# Plot
plt.figure(figsize=(14, 14))
plt.plot(Trajectory_GPS[:, 0], Trajectory_GPS[:, 1], label='GPS', color='red')
plt.plot(AlignedTrajectory_IMU[:, 0], AlignedTrajectory_IMU[:, 1], label='IMU', color='blue')
plt.title('Estimated Trajectory from IMU and GPS')
plt.xlabel('Easting, m')
plt.ylabel('Northing, m')
plt.legend()
plt.grid()
plt.axis('equal')

plt.show()