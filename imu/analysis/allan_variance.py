#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
import re
import os, rospkg
from allantools import oadev

def CalculateARW():
    # Find the index where tau is closest to 1 for each axis
    index_x = np.argmin(np.abs(gyro_allan_var_x[0] - 1))
    index_y = np.argmin(np.abs(gyro_allan_var_y[0] - 1))
    index_z = np.argmin(np.abs(gyro_allan_var_z[0] - 1))
    t1_X = gyro_allan_var_x[1][index_x]
    t1_Y = gyro_allan_var_y[1][index_y]
    t1_Z = gyro_allan_var_z[1][index_z]
    # Mark points at tau=1 for each axis
    plt.scatter(gyro_allan_var_x[0][index_x], gyro_allan_var_x[1][index_x], color='blue', marker='^', label='X (1,'+str(round(t1_X,4))+')')
    plt.scatter(gyro_allan_var_y[0][index_y], gyro_allan_var_y[1][index_y], color='orange', marker='^', label='Y (1,'+str(round(t1_Y,4))+')')
    plt.scatter(gyro_allan_var_z[0][index_z], gyro_allan_var_z[1][index_z], color='green', marker='^', label='Z (1,'+str(round(t1_Z,4))+')')

    # Calculation of ARW parameters
    ARW_X = t1_X * 60
    ARW_Y = t1_Y * 60
    ARW_Z = t1_Z * 60
    ARW = math.sqrt(pow(ARW_X,2) + pow(ARW_Y,2) + pow(ARW_Z,2))
    ARW_text = 'Angle Random Walk, N:\nNx='+str(round(t1_X,4))+'*60='+str(round(ARW_X,4))+' deg/\u221ahr\nNy='+str(round(t1_Y,4))+'*60='+str(round(ARW_Y,4))+' deg/\u221ahr\nNz='+str(round(t1_Z,4))+'*60='+str(round(ARW_Z,4))+' deg/\u221ahr\nN=\u221a(Nx\u00b2+Ny\u00b2+Nz\u00b2)='+str(round(ARW,4))+' deg/\u221ahr\n'
    return ARW_text

def CalculateBI():
    # Calculate slope
    slope_values_x = np.gradient(gyro_allan_var_x[1], gyro_allan_var_x[0])
    slope_values_y = np.gradient(gyro_allan_var_y[1], gyro_allan_var_y[0])
    slope_values_z = np.gradient(gyro_allan_var_z[1], gyro_allan_var_z[0])
    # Find the index of the first point where the slope is close to 0
    index_x = np.argmin(np.abs(slope_values_x))
    index_y = np.argmin(np.abs(slope_values_y))
    index_z = np.argmin(np.abs(slope_values_z))

    # Mark points in plot
    plt.scatter(gyro_allan_var_x[0][index_x], gyro_allan_var_x[1][index_x], color='blue',marker='x', label='X m=0 ('+str(round(gyro_allan_var_x[0][index_x],4))+','+str(round(gyro_allan_var_x[1][index_x],4))+')')
    plt.scatter(gyro_allan_var_y[0][index_y], gyro_allan_var_y[1][index_y], color='orange',marker='x', label='Y m=0 ('+str(round(gyro_allan_var_y[0][index_y],4))+','+str(round(gyro_allan_var_y[1][index_y],4))+')')
    plt.scatter(gyro_allan_var_z[0][index_z], gyro_allan_var_z[1][index_z], color='green',marker='x', label='Z m=0 ('+str(round(gyro_allan_var_z[0][index_z],4))+','+str(round(gyro_allan_var_z[1][index_z],4))+')')
    
    x_val = gyro_allan_var_x[1][index_x]
    y_val = gyro_allan_var_y[1][index_y]
    z_val = gyro_allan_var_z[1][index_z]
    BI_X = x_val * (1/0.664) * 3600
    BI_Y = y_val * (1/0.664) * 3600
    BI_Z = z_val * (1/0.664) * 3600
    BI = math.sqrt(pow(BI_X,2) + pow(BI_Y,2) + pow(BI_Z,2))
    BI_text = '\nBias Instability, BI:\nBIx='+str(round(x_val,4))+'*(1/0.664)*3600='+str(round(BI_X,4))+' deg/hr\nBIy='+str(round(y_val,4))+'*(1/0.664)*3600='+str(round(BI_Y,4))+' deg/hr\nBIz='+str(round(z_val,4))+'*(1/0.664)*3600='+str(round(BI_Z,4))+' deg/hr\nBI=\u221a(BIx\u00b2+BIy\u00b2+BIz\u00b2)='+str(round(BI,4))+' deg/hr\n'
    return BI_text

def CalculateRRW():
    # Calculate slope
    slope_values_x = np.gradient(gyro_allan_var_x[1], gyro_allan_var_x[0])
    slope_values_y = np.gradient(gyro_allan_var_y[1], gyro_allan_var_y[0])
    slope_values_z = np.gradient(gyro_allan_var_z[1], gyro_allan_var_z[0])
    # Find the index of the first point where the slope is close to 0
    index_x = np.argmin(np.abs(slope_values_x - 0.5))
    index_y = np.argmin(np.abs(slope_values_y - 0.5))
    index_z = np.argmin(np.abs(slope_values_z - 0.5))

    # Mark points in plot
    plt.scatter(gyro_allan_var_x[0][index_x], gyro_allan_var_x[1][index_x], color='blue',marker='o', label='X m=0.5 ('+str(round(gyro_allan_var_x[0][index_x],4))+','+str(round(gyro_allan_var_x[1][index_x],6))+')')
    plt.scatter(gyro_allan_var_y[0][index_y], gyro_allan_var_y[1][index_y], color='orange',marker='o', label='Y m=0.5 ('+str(round(gyro_allan_var_y[0][index_y],4))+','+str(round(gyro_allan_var_y[1][index_y],6))+')')
    plt.scatter(gyro_allan_var_z[0][index_z], gyro_allan_var_z[1][index_z], color='green',marker='o', label='Z m=0.5 ('+str(round(gyro_allan_var_z[0][index_z],4))+','+str(round(gyro_allan_var_z[1][index_z],6))+')')
    
    x_val = gyro_allan_var_x[1][index_x]
    y_val = gyro_allan_var_y[1][index_y]
    z_val = gyro_allan_var_z[1][index_z]
    RRW_X = x_val * 60
    RRW_Y = y_val * 60
    RRW_Z = z_val * 60
    RRW = math.sqrt(pow(RRW_X,2) + pow(RRW_Y,2) + pow(RRW_Z,2))
    RRW_text = '\nRate Random Walk, K:\nKx='+str(round(x_val,6))+'*60='+str(round(RRW_X,4))+' deg/\u221ahr\nKy='+str(round(y_val,6))+'*60='+str(round(RRW_Y,4))+' deg/\u221ahr\nKz='+str(round(z_val,6))+'*60='+str(round(RRW_Z,4))+' deg/\u221ahr\nK=\u221a(Kx\u00b2+Ky\u00b2+Kz\u00b2)='+str(round(RRW,4))+' deg/\u221ahr\n'
    return RRW_text

# Get the path to the Catkin workspace
rospack = rospkg.RosPack()
package_path = rospack.get_path('vn_driver')
catkin_ws_path = os.path.dirname(os.path.dirname(package_path))

# Open 5hr imu data bag
bag = rosbag.Bag(catkin_ws_path+'/data/LocationC.bag', 'r')   

# Initialize lists for data  
GyroX_rad = []
GyroY_rad = []
GyroZ_rad = []

# Iterate through messages in the specified topic
for topic, msg, t in bag.read_messages(topics='/vectornav'): 
    # Copy required data from bag to list
    try:
        msgSplit = re.split('[,*]', msg.data)
        # Angular rate in rad/sec
        GyroX_rad.append(float(msgSplit[10]))
        GyroY_rad.append(float(msgSplit[11]))
        GyroZ_rad.append(float(msgSplit[12]))

    except:
        continue

bag.close()

# Convert unit to deg/sec
GyroX = np.degrees(GyroX_rad)
GyroY = np.degrees(GyroY_rad)
GyroZ = np.degrees(GyroZ_rad)

# Calculate and plot Allan variance for gyro data
gyro_allan_var_x = oadev(GyroX, rate=40.0, data_type='freq', taus='all')
gyro_allan_var_y = oadev(GyroY, rate=40.0, data_type='freq', taus='all')
gyro_allan_var_z = oadev(GyroZ, rate=40.0, data_type='freq', taus='all')
plt.loglog(gyro_allan_var_x[0], gyro_allan_var_x[1], label='X', color='blue')
plt.loglog(gyro_allan_var_y[0], gyro_allan_var_y[1], label='Y', color='orange')
plt.loglog(gyro_allan_var_z[0], gyro_allan_var_z[1], label='Z', color='green')

# Calculate and plot noise parameters
ARW_text = CalculateARW()
BI_text = CalculateBI()
RRW_text = CalculateRRW()
plt.text(0.017, 0.0002, ARW_text+BI_text+RRW_text, fontsize=8.5, color='blue', ha='left', va='center', bbox=dict(facecolor='white', edgecolor='white', boxstyle='round,pad=0.01'))

plt.title('Gyroscope Allan Variance')
plt.xlabel('Tau (s)')
plt.ylabel('Allan Deviation (deg/sec)')
plt.legend(loc='upper right', fontsize=8.5)
plt.grid(True, which="both")
plt.savefig('gyro_allanvar')
plt.show()