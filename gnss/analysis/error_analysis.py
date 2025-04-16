#!/usr/bin/env python3
import sys
import rosbag
import numpy as np
import math
from gps_driver.msg import Customgps
from gps_driver.msg import Customrtk

# Constants storing known positions
GPS_KNOWN_POS_OPEN = [327964, 4689637]
GPS_KNOWN_POS_OCCLUDED = [327966, 4689437]
RTK_KNOWN_POS_OPEN = [328113, 4689442]
RTK_KNOWN_POS_OCCLUDED = [328077, 4689333]

def ExtractAttributesfromMsg(msg_list):
    # Setting the attribute lists
    UTMEasting = np.zeros(len(msg_list))
    UTMNorthing = np.zeros(len(msg_list))
    altitude = np.zeros(len(msg_list))
    time = np.zeros(len(msg_list))

    for i in range(len(msg_list)):
        msg = msg_list[i]
        UTMEasting[i] = msg.utm_easting
        UTMNorthing[i] = msg.utm_northing
        UTMNorthing[i] = msg.utm_northing
        altitude[i] = msg.altitude
        time[i] = msg.header.stamp.to_sec()

    return [UTMEasting, UTMNorthing, altitude, time]     

def GetDRMSError(UTMEasting, UTMNorthing, KNOWN_POS):
    # Initialize np arrays for normalized values
    UTMEasting_normalized = np.zeros(np.size(UTMEasting))
    UTMNorthing_normalized = np.zeros(np.size(UTMNorthing))  

    # Normalize the values
    UTMEasting_normalized = UTMEasting - KNOWN_POS[0]
    UTMNorthing_normalized = UTMNorthing - KNOWN_POS[1]
    
    # Calculate the error
    UTMEasting_dist_centroid = np.square(UTMEasting_normalized)
    UTMNorthing_dist_centroid = np.square(UTMNorthing_normalized)
    UTMEasting_sigma = np.sqrt(np.mean(UTMEasting_dist_centroid))
    UTMNorthing_sigma = np.sqrt(np.mean(UTMNorthing_dist_centroid)) 
    DRMS = math.sqrt(UTMEasting_sigma**2 + UTMNorthing_sigma**2)
    return DRMS


# Check if the script is called with the correct number of arguments
if len(sys.argv) != 2:
    print("Usage: python3 error_analysis.py [gps or rtk]")
    sys.exit(1)

# Bag file path and names
bag_file_path = '/home/christina/Documents/git/EECE5554/gnss/data/'
bag_list = ['stationary_free', 'stationary_occluded', 'moving']

# Determine whether standalone or rtk data
if (sys.argv[1] == 'gps'):
    topic_name = 'gps'
    KNOWN_POS_OPEN = GPS_KNOWN_POS_OPEN
    KNOWN_POS_OCCLUDED = GPS_KNOWN_POS_OCCLUDED

elif (sys.argv[1] == 'rtk'):
    topic_name = 'rtk_gnss'
    bag_file_path += 'rtk_'
    KNOWN_POS_OPEN = RTK_KNOWN_POS_OPEN
    KNOWN_POS_OCCLUDED = RTK_KNOWN_POS_OCCLUDED    

# Initialize lists for each data  
stationary_open_msg_list = []
stationary_occluded_msg_list = []
moving_msg_list = []

for bag_name in bag_list:
    # Opening each bag in the data folder
    bag = rosbag.Bag(bag_file_path+bag_name+'.bag', 'r')   
    
    # Iterate through messages in the specified topic
    for topic, msg, t in bag.read_messages(topics=topic_name):
        
        # Copy data from bag to respective list
        if (bag_name == 'stationary_free'):
            stationary_open_msg_list.append(msg)
        if (bag_name == 'stationary_occluded'):
            stationary_occluded_msg_list.append(msg)
        if (bag_name == 'moving'):
            moving_msg_list.append(msg)

# Get attributes list of each data
sopen_UTMEasting, sopen_UTMNorthing, sopen_altitude, sopen_time = ExtractAttributesfromMsg(stationary_open_msg_list)
soccl_UTMEasting, soccl_UTMNorthing, soccl_altitude, soccl_time = ExtractAttributesfromMsg(stationary_occluded_msg_list)
mov_UTMEasting, mov_UTMNorthing, mov_altitude, mov_time = ExtractAttributesfromMsg(moving_msg_list)


# Find the 2D position error 
sopen_DRMS = GetDRMSError(sopen_UTMEasting, sopen_UTMNorthing, KNOWN_POS_OPEN)
soccl_DRMS = GetDRMSError(soccl_UTMEasting, soccl_UTMNorthing, KNOWN_POS_OCCLUDED)
print('Open DRMS: ', sopen_DRMS)
print('Occluded DRMS: ', soccl_DRMS)

# Find the error from line of best fit to moving data
slope, intercept = np.polyfit(mov_UTMEasting, mov_UTMNorthing, 1)
model_y = slope * mov_UTMEasting + intercept
mov_UTMNorthing_diff = np.subtract(mov_UTMNorthing, model_y)
mov_UTMNorthing_MSE = np.mean(np.square(mov_UTMNorthing_diff))
mov_UTMNorthing_RMSE = math.sqrt(mov_UTMNorthing_MSE)
print('Moving Error: ', mov_UTMNorthing_RMSE)



if topic_name == 'rtk_gnss':

    # Get avg fix quality for open data
    print('\nOpen Fix Q:')
    sopen_fixq = np.zeros(len(stationary_open_msg_list))
    for i in range (len(stationary_open_msg_list)):
        sopen_fixq[i] = stationary_open_msg_list[i].fix_quality
    print(sopen_fixq)
    print('Average StationaryOpen Fix Quality:',np.mean(sopen_fixq))

    # Get avg fix quality for occluded data
    print('\nOccluded Fix Q:')
    soccl_fixq = np.zeros(len(stationary_occluded_msg_list))
    for i in range (len(stationary_occluded_msg_list)):
        soccl_fixq[i] = stationary_occluded_msg_list[i].fix_quality
    print(soccl_fixq)
    print('Average StationaryOccluded Fix Quality:',np.mean(soccl_fixq))
