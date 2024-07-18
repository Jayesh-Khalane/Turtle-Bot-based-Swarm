#!/usr/bin/env python3
#Author : Jayesh Khalane
"""
This code is used to calucalte ratio of dectected groups.
"""

import rospy
from sensor_msgs.msg import LaserScan
import math

def find_patches(data, max_patch_length=55, diff_threshold=0.1):
    patches = []
    current_patch = []
    start_indices = []

    for i in range(1, len(data)):
        if abs(data[i] - data[i - 1]) > diff_threshold:
            if current_patch:
                if len(current_patch) <= max_patch_length:
                    patches.append((current_patch, start_indices[0]))
                current_patch = []
                start_indices = []
        current_patch.append(data[i])
        start_indices.append(i)

    # Append the last patch if it exists and meets the length criteria
    if current_patch and len(current_patch) <= max_patch_length:
        patches.append((current_patch, start_indices[0]))

    return patches, len(patches)

def process_lidar_data(data):
    patches, count = find_patches(data.ranges, diff_threshold=0.1)
    angle_increment = data.angle_increment
    angle_min = data.angle_min

    if not patches:
        return

    # Find the smallest patch
    smallest_patch = min(patches, key=lambda patch: len(patch[0]))

    patch, start_index = smallest_patch
    if len(patch) == 0:
        return

    patch_no_a = patch[0]
    patch_no_b = patch[-1]
    patch_no_distance = patch[len(patch) // 2]
    deg = (len(patch) )/2.0

    # Calculate the start angle in radians
    start_angle = start_index * angle_increment + angle_min

    # Apply cosine rule
    c_squared = patch_no_a ** 2 + patch_no_b ** 2 - 2 * patch_no_a * patch_no_b * math.cos(math.radians(deg))
    c = math.sqrt(c_squared)
    ratio = c/patch_no_distance 

    # Print the ratio
    print(f"Ratio = {ratio}, Distance = {patch_no_distance}, Angle = {math.degrees(start_angle):.2f}°")

def lidar_callback(data):
    process_lidar_data(data)

def main():
    rospy.init_node('lidar_processor', anonymous=True)
    rospy.Subscriber('/tb3_0/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
