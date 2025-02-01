#!/usr/bin/env python3
# Generate csv from rosbag file

import rosbag
import rospy
import sys
import os
import csv
import math

args = sys.argv

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
output_csv=os.path.normpath(os.path.join(os.getcwd(),args[2]))

# read the bag file
global data_num
global sumx, sumy, sumz
global count
count = 0
sumx = 0
sumy = 0
sumz = 0


bag = rosbag.Bag(filename)


with open(output_csv, 'w') as csvfile:
    filewriter = csv.writer(csvfile, delimiter = ',')
    header = ["北方向のばらつき[m]" , "東方向のばらつき[m]", "鉛直方向のばらつき[m]"]
    filewriter.writerow(header)
    
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        p = msg.position_covariance
        sumx += math.sqrt(p[0])
        sumy += math.sqrt(p[4])
        sumz += math.sqrt(p[8])
        count += 1
        ax = sumx/count
        ay = sumy/count
        az = sumz/count
        filewriter.writerow([math.sqrt(p[0]), math.sqrt(p[4]), math.sqrt(p[8])])
    
    footer = ["北方向のばらつきの平均[m]" , "東方向のばらつきの平均[m]", "鉛直方向のばらつきの平均[m]"]
    filewriter.writerow(footer)
    filewriter.writerow([ax,ay,az])
# with open(output_csv, "a") as csvfile:
#     filewriter.writerow()
print("北方向のばらつきの平均[m], 東方向のばらつきの平均[m], 鉛直方向のばらつきの平均[m]")
print(ax,ay,az)

bag.close()