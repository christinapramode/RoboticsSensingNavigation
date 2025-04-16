#!/usr/bin/env python3
import sys
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import MinMaxScaler
from gps_driver.msg import Customgps
from gps_driver.msg import Customrtk

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

def NormalizeData(UTMEasting, UTMNorthing):
    # Initialize np arrays for normalized values
    UTMEasting_normalized = np.zeros(np.size(UTMEasting))
    UTMNorthing_normalized = np.zeros(np.size(UTMNorthing))  
    
    # Find centroid values
    centroid = [0, 0]
    centroid[0] = np.mean(UTMEasting)
    centroid[1] = np.mean(UTMNorthing)

    # Normalize the values
    UTMEasting_normalized = UTMEasting - centroid[0]
    UTMNorthing_normalized = UTMNorthing - centroid[1]  

    return [UTMEasting_normalized, UTMNorthing_normalized, centroid]

# Check if the script is called with the correct number of arguments
if len(sys.argv) != 2:
    print("Usage: python3 analysis.py [gps or rtk]")
    sys.exit(1)

# Bag file path and names
bag_file_path = '/home/christina/Documents/git/EECE5554/gnss/data/'
bag_list = ['stationary_free', 'stationary_occluded', 'moving']

# Determine whether standalone or rtk data
if (sys.argv[1] == 'gps'):
    topic_name = 'gps'
elif (sys.argv[1] == 'rtk'):
    topic_name = 'rtk_gnss'
    bag_file_path += 'rtk_'

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


# Stationary Northing vs Easting scatterplot
fig1, ax1 = plt.subplots()
# Normalize values and get centroid
sopen_UTMEasting_normalized, sopen_UTMNorthing_normalized, sopen_centroid = NormalizeData(sopen_UTMEasting, sopen_UTMNorthing)
soccl_UTMEasting_normalized, soccl_UTMNorthing_normalized, soccl_centroid = NormalizeData(soccl_UTMEasting, soccl_UTMNorthing)

ax1.scatter(sopen_UTMEasting_normalized, sopen_UTMNorthing_normalized, label='Stationary_Open', color='blue')
ax1.scatter(soccl_UTMEasting_normalized, soccl_UTMNorthing_normalized, label='Stationary_Occluded', color='red')
ax1.set_xlabel('Difference in Easting, m')
ax1.set_ylabel('Difference in Northing, m')
ax1.set_title('Stationary Northing vs Easting Scatterplot')
legend_text = ['', '']
legend_text[0] = 'Stationary Open,\nC:(' + str(round(sopen_centroid[0],2)) + ',' + str(round(sopen_centroid[1],2)) + ') \nNormalizedC:' + str(round(np.mean(sopen_UTMEasting_normalized),2)) + ',' + str(round(np.mean(sopen_UTMNorthing_normalized),2)) + ')'
legend_text[1] = 'Stationary Occluded,\nC:(' + str(round(soccl_centroid[0],2)) + ',' + str(round(soccl_centroid[1],2)) + ') \nNormalizedC:' + str(round(np.mean(soccl_UTMEasting_normalized),2)) + ',' + str(round(np.mean(soccl_UTMNorthing_normalized),2)) + ')'
ax1.legend(legend_text)
ax1.grid()
fig1.savefig(topic_name+'_stationary_N_vs_E')

# Stationary altitude vs time plot
fig2, ax2 = plt.subplots()
# Normalizing time data
scaler = MinMaxScaler()
sopen_time_normalized = scaler.fit_transform([[t] for t in sopen_time])
soccl_time_normalized = scaler.fit_transform([[t] for t in soccl_time])
ax2.plot(sopen_time_normalized, sopen_altitude, label='Stationary_Open', color='blue')
ax2.plot(soccl_time_normalized, soccl_altitude, label='Stationary_Occluded', color='red')
ax2.set_xlabel('Time, s')
ax2.set_ylabel('Altitude, m')
ax2.set_title('Stationary Altitude vs Time')
ax2.legend()
ax2.grid()
fig2.savefig(topic_name+'_stationary_alt_vs_time')

# Stationary open position histogram
fig3, ax3 = plt.subplots()
# Get distances from centroid
distances = np.sqrt((sopen_UTMEasting - sopen_centroid[0])**2 + (sopen_UTMNorthing - sopen_centroid[1])**2)
# Plot histogram
ax3.hist(distances, bins=10, edgecolor='black')
ax3.set_title('Stationary Open \nHistogram of Euclidean Distances from Centroid')
ax3.set_xlabel('Euclidean Distance, m')
ax3.set_ylabel('Frequency')
fig3.savefig(topic_name+'_stationary_open_histogram')

# Stationary occluded position histogram
fig4, ax4 = plt.subplots()
# Get distances from centroid
distances = np.sqrt((soccl_UTMEasting - soccl_centroid[0])**2 + (soccl_UTMNorthing - soccl_centroid[1])**2)
# Plot histogram
ax4.hist(distances, bins=10, edgecolor='black')
ax4.set_title('Stationary Occluded \nHistogram of Euclidean Distances from Centroid')
ax4.set_xlabel('Euclidean Distance, m')
ax4.set_ylabel('Frequency')
fig4.savefig(topic_name+'_stationary_occluded_histogram')

# Moving Northing vs Easting and line of best fit
fig5, ax5 = plt.subplots()
ax5.scatter(mov_UTMEasting, mov_UTMNorthing, label='Moving Northing vs Easting Scatterplot', color='green')
z = np.polyfit(mov_UTMEasting, mov_UTMNorthing, 1)
model_y = z[0] * mov_UTMEasting + z[1]  # z[0]:slope, z[1]:y-intercept
ax5.plot(mov_UTMEasting, model_y, label='Line of best fit', color='red', linewidth=2)
ax5.set_xlabel('Easting, m')
ax5.set_ylabel('Northing, m')
ax5.set_title('Moving Northing vs Easting \nand \nLine of Best Fit')
ax5.legend()
ax5.grid()
fig5.savefig(topic_name+'_moving_N_vs_E')

# Moving Altitude vs Time
fig6, ax6 = plt.subplots()
ax6.plot(mov_time, mov_altitude)
ax6.set_xlabel('Time, s')
ax6.set_ylabel('Altitude, m')
ax6.set_title('Moving Altitude vs Time')
ax6.grid()
fig6.savefig(topic_name+'_moving_alt_vs_time')

plt.show()
