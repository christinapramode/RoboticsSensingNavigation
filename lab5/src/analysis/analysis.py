#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math, os, rospkg
from scipy.optimize import least_squares
import scipy.integrate as integrate 
from scipy.signal import butter, sosfilt
from lab5.msg import Vectornav
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

# This function describes my mapping from measured data back to a circle
def distortion_model(X_meas, dist_params):
    x = dist_params[0] * (X_meas[0] - dist_params[4]) + dist_params[1]*(X_meas[1] - dist_params[5])
    y = dist_params[2] * (X_meas[0] - dist_params[4]) + dist_params[3]*(X_meas[1] - dist_params[5])
    X = np.array([x,y])
    return X

# This function finds the difference between a circle and my transformed measurement
def residual(p, X_mag, X_meas):
    return (X_mag - distortion_model(X_meas, p)).flatten()

def CalibrateMagField(cMagField):
    #Completely made up data set for "perfect" magnetometer readings
    field_strength = 20509e-9 #The horizontal magnetic field strength in Boston is approx 20,500 nT 
    angle = np.linspace(5*-np.pi, 5*np.pi, cMagField.shape[1])
    x_mag = field_strength * np.sin(angle) 
    y_mag = field_strength * np.cos(angle) 
    X_mag = np.array([x_mag, y_mag])
    X_meas = np.array([cMagField[0], cMagField[1]])
    
    #Least squares optimization to find model coefficients
    p0 = [0,0,0,0,0,0]
    lsq_min = least_squares(residual, p0, args=(X_mag, X_meas))
    print("least square fitting values are: ")
    print(lsq_min.x)
    return lsq_min.x

def PlotCalibration(MagField, MagField_calibrated, calibration_params):
    fig1, ax1 = plt.subplots(1, 2, figsize=(14, 14))
    fig1.subplots_adjust(wspace=0.3)
    ax1[0].scatter(MagField[0], MagField[1], label="Measured data",color='orange',s=2)
    ax1[0].legend()
    ax1[0].grid()
    ax1[0].set_xlim(-0.00006, 0.00006)
    ax1[0].set_ylim(-0.00006, 0.00006)
    ax1[0].tick_params(axis='both', labelsize=8)
    ax1[0].set_xlabel('X component of magnetic field (T)')
    ax1[0].set_ylabel('Y component of magnetic field (T)')
    ax1[0].set_title('X vs Y Magnetic Field\nBefore Calibration')
    ax1[1].scatter(MagField_calibrated[0], MagField_calibrated[1], label="Calibrated data",color='lightblue',s=2)
    ax1[1].legend()
    ax1[1].grid()
    ax1[1].set_xlim(-0.00006, 0.00006)
    ax1[1].set_ylim(-0.00006, 0.00006) 
    ax1[1].tick_params(axis='both', labelsize=8)
    ax1[1].set_xlabel('X component of magnetic field (T)')
    ax1[1].set_ylabel('Y component of magnetic field (T)')
    ax1[1].set_title('X vs Y Magnetic Field\nAfter Calibration')
    lsq = ['{:.6f}'.format(x) for x in calibration_params]
    fig1.text(0.005, 0.015, 'least square fitting values='+str(lsq), fontsize=7, color='blue', ha='left', va='center')

def PlotMagHeading(MagHeading, MagHeading_calibrated, Time):
    fig3, ax3 = plt.subplots(2, 1, figsize=(14, 14))
    fig3.subplots_adjust(hspace=0.5)
    ax3[0].plot(Time, MagHeading, color='blue')
    ax3[0].set_xlabel('Time, sec')
    ax3[0].set_ylabel('Rotation, deg')
    ax3[0].set_title('Rotation from MagHeading before Calibration')
    ax3[0].grid()
    ax3[1].plot(Time, MagHeading_calibrated, color='red')
    ax3[1].set_xlabel('Time, sec')
    ax3[1].set_ylabel('Rotation, deg')
    ax3[1].set_title('Rotation from MagHeading after Calibration')
    ax3[1].grid()

def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
    sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

def FilterYawValues(MagYaw, GyroYaw, ActualYaw, Time):
    # Setting filter requirements
    order = 2
    sampl_freq = 40
    low_pass_cutoff = 0.015
    high_pass_cutoff = 0.0125
    alpha = 0.6

    # Low-pass filter magnetometer yaw (removes drift)
    _, MagYaw_filtered = butter_filter(MagYaw, low_pass_cutoff, sampl_freq, "lowpass", order)
    
    # High-pass filter gyroscope yaw rate (preserves rapid changes)
    _, GyroYaw_filtered = butter_filter(GyroYaw, high_pass_cutoff, sampl_freq, "highpass", order)

    # Complementary filter to fuse the lowpass and highpass filter results
    FusedYaw = alpha * MagYaw_filtered + (1-alpha) * GyroYaw_filtered

    # Plotting the results
    fig1, ax1 = plt.subplots(2, 2, figsize=(14, 14))
    fig1.subplots_adjust(hspace=0.5)
    ax1[0, 0].plot(Time, MagYaw, label='Mag Yaw', color='orange')
    ax1[0, 0].plot(Time, MagYaw_filtered, label='Filtered Yaw', color='blue')
    ax1[0, 0].set_xlabel('Time, sec')
    ax1[0, 0].set_ylabel('Rotation, deg')
    ax1[0, 0].set_title('Low pass filter of Magnetometer data')
    ax1[0, 0].grid()
    ax1[0, 0].legend()
    ax1[0, 1].plot(Time, GyroYaw, label='Gyro Yaw', color='orange')
    ax1[0, 1].plot(Time, GyroYaw_filtered, label='Filtered Yaw', color='blue')
    ax1[0, 1].set_xlabel('Time, sec')
    ax1[0, 1].set_ylabel('Rotation, deg')
    ax1[0, 1].set_title('High pass filter of Gyroscope data')
    ax1[0, 1].grid()  
    ax1[0, 1].legend()  
    ax1[1, 0].plot(Time, FusedYaw, color='green')
    ax1[1, 0].set_xlabel('Time, sec')
    ax1[1, 0].set_ylabel('Rotation, deg')
    ax1[1, 0].set_title('Complementary filter')
    ax1[1, 0].grid()
    ax1[1, 1].plot(Time, ActualYaw)
    ax1[1, 1].set_xlabel('Time, sec')
    ax1[1, 1].set_ylabel('Rotation, deg')
    ax1[1, 1].set_title('Yaw from IMU')
    ax1[1, 1].grid()

    return [MagYaw_filtered, GyroYaw_filtered, FusedYaw]

def CalculateDistance(lat1, lon1, lat2, lon2):
        # Convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

        # Calculate distance between latitudes and longitudes using Haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        r = 6371000  # Radius of earth in m
        distance = r * c
        return distance
        
def AdjustForwardVelocity(AccX, Time):
    # Integrating acceleration to get velocity
    ForwardVelocity = integrate.cumulative_trapezoid(AccX, Time, initial = 0)
    
    # Passing acceleration through a high-pass filter to remove bias
    _, AccX_filtered = butter_filter(AccX, 0.02, 40, "highpass", 1)
    
    # Integrate Acceleration to get velocity
    ForwardVelocity_adjusted = integrate.cumulative_trapezoid(AccX_filtered, Time, initial = 0)
    ForwardVelocity_corrected = ForwardVelocity_adjusted
    ForwardVelocity_adjusted = np.maximum(ForwardVelocity_adjusted, 0)
    
    # Plotting Velocity vs Time
    fig, ax = plt.subplots(2,1,figsize=(14, 14))
    fig.subplots_adjust(hspace=0.5)
    ax[0].plot(Time, ForwardVelocity, color='blue')
    ax[0].set_xlabel('Time, sec')
    ax[0].set_ylabel('Velocity, m/s')
    ax[0].set_title('Forward Velocity from IMU vs Time (Before Adjustment)')
    ax[0].grid()
    ax[1].plot(Time, ForwardVelocity_adjusted, color='red')
    ax[1].set_xlabel('Time, sec')
    ax[1].set_ylabel('Velocity, m/s')
    ax[1].set_title('Forward Velocity from IMU vs Time (After Adjustment)')
    ax[1].grid()
    return [ForwardVelocity_adjusted, ForwardVelocity_corrected]

def CalculateVelocityfromGPS(Latitude, Longitude, Time):
    velocities = []
    distances = []

    for i in range(1, len(Time)):
        # Calculate distance using Haversine formula
        distance = CalculateDistance(Latitude[i-1], Longitude[i-1], Latitude[i], Longitude[i])
        delta_t = Time[i] - Time[i-1]
        velocity = distance / delta_t
        velocities.append(velocity)
        distances.append(distance)
    
    # Plot Velocity vs Time
    fig, ax = plt.subplots(figsize=(14, 14))
    ax.plot(Time[1:], velocities)
    ax.set_xlabel('Time, sec')
    ax.set_ylabel('Velocity, m/s')
    ax.set_title('Forward Velocity from GPS vs Time')
    ax.grid()

    return velocities

def AdjustLinAccY(AccY, GyroZ, VelX, Time):
    # Acceleration in Y = VelX * w
    AccY_obs = np.multiply(GyroZ, VelX)

    # Pass through GyroZ LPF to remove noise
    _, GyroZ_filtered = butter_filter(GyroZ, 0.01, 40, "lowpass", 1)
    AccY_obs_filtered = np.multiply(GyroZ_filtered, VelX)

    fig, ax = plt.subplots(2, 1, figsize=(14, 14))
    fig.subplots_adjust(hspace=0.5)
    ax[0].plot(Time, AccY_obs, color='orange', label='Before Correction')
    ax[0].plot(Time, AccY_obs_filtered, color='blue', label='After Correction')
    ax[0].set_xlabel('Time, sec')
    ax[0].set_ylabel('Acceleration, m/s\u00b2')
    ax[0].set_title('Observed Linear Acceleration Y')
    ax[0].grid()
    ax[0].legend()
    ax[1].plot(Time, AccY, color='red')
    ax[1].set_xlabel('Time, sec')
    ax[1].set_ylabel('Acceleration, m/s\u00b2')
    ax[1].set_title('IMU Linear Acceleration Y')
    ax[1].grid()

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
UTMEasting, UTMNorthing, Latitude, Longitude, Time_GPS = ExtractAttributesfromGPSMsg(gps_msg_list)

# Calibration for hard and soft iron
calibration_params = CalibrateMagField(cMagField)
MagField_calibrated = distortion_model(MagField, calibration_params)
PlotCalibration(MagField, MagField_calibrated, calibration_params)

MagHeading = np.degrees(np.arctan2(MagField[0], MagField[1]))
MagHeading_calibrated = np.degrees(np.arctan2(MagField_calibrated[0], MagField_calibrated[1]))
PlotMagHeading(MagHeading, MagHeading_calibrated, Time_IMU)

GyroYaw = integrate.cumulative_trapezoid(Gyro[2], Time_IMU, initial = 0)
fig, ax = plt.subplots(figsize=(14, 14))
ax.plot(Time_IMU, GyroYaw)
ax.set_xlabel('Time, sec')
ax.set_ylabel('Rotation, deg')
ax.set_title('Yaw Angle from Integrating Rotational Rate')
ax.grid()

# High-pass, Low-pass, and Complementary filter implementation
FilteredYawValues = FilterYawValues(MagHeading_calibrated, GyroYaw, EulerAngles[2], Time_IMU)

# IMU Velocity adjustment and plotting
ForwardVelocity_IMU = AdjustForwardVelocity(Acceleration[0], Time_IMU)

# GPS Velocity calculation and plotting
ForwardVelocity_GPS = CalculateVelocityfromGPS(Latitude, Longitude, Time_GPS)

# Displacement Comparsion
Displacement_IMU = np.abs(np.cumsum(ForwardVelocity_IMU[1]) / 40)
Displacement_GPS = integrate.cumulative_trapezoid(ForwardVelocity_GPS, Time_GPS[1:], initial = 0)
Displacement_GPS = Displacement_GPS * 10
fig, ax = plt.subplots(figsize=(14, 14))
ax.plot(Time_IMU, Displacement_IMU, color='blue', label='IMU')
ax.plot(Time_GPS[1:], Displacement_GPS, color='red', label='GPS')
ax.set_xlabel('Time, sec')
ax.set_ylabel('Distance, m')
ax.set_title('Displacement vs Time')
ax.grid()
ax.legend()

# Linear Acceleration in Y Comparison Plots
AdjustLinAccY(Acceleration[1], Gyro[2], ForwardVelocity_IMU[0], Time_IMU)

plt.show()