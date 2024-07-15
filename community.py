#!/usr/bin/env python3
#Author: Jayesh Khalane
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import math

def find_patches(data, diff_threshold=0.1):
    patches = []
    current_patch = []
    start_indices = []

    for i in range(1, len(data)):
        if abs(data[i] - data[i - 1]) > diff_threshold:
            if current_patch:
                patches.append((current_patch, start_indices[0]))
                current_patch = []
                start_indices = []
        current_patch.append(data[i])
        start_indices.append(i)

    # Append the last patch if it exists
    if current_patch:
        patches.append((current_patch, start_indices[0]))

    return patches, len(patches)

def process_lidar_data(data):
    patches, count = find_patches(data.ranges, diff_threshold=0.1)
    angle_increment = data.angle_increment
    angle_min = data.angle_min

    closest_bot = None
    closest_distance = float('inf')

    for i, (patch, start_index) in enumerate(patches):
        

        patch_no_a = patch[0]
        patch_no_b = patch[-1]
        patch_no_distance = patch[len(patch) // 2]
        deg = (len(patch) )/3.0

        # Calculate the start angle in radians
        start_angle = start_index * angle_increment + angle_min

        # Apply cosine rule
        c_squared = patch_no_a ** 2 + patch_no_b ** 2 - 2 * patch_no_a * patch_no_b * math.cos(math.radians(deg))
        c = math.sqrt(c_squared)
        ratio = c / patch_no_distance

        # Check if the ratio lies within the desired range
        if (0.7 < patch_no_distance < 1.0 and 0.1 < ratio < 0.18) or \
           (1.0 < patch_no_distance < 1.5 and 0.07 < ratio < 0.2) or \
           (1.5 < patch_no_distance < 2.0 and 0.05 < ratio < 0.08) or \
           (2.0 < patch_no_distance < 2.7 and 0.04 < ratio < 0.06):
            if patch_no_distance < closest_distance:
                closest_distance = patch_no_distance
                closest_bot = (patch_no_distance, start_angle, ratio)

    return closest_bot

def move_towards_bot(bot_distance, bot_angle):
    twist = Twist()
    if bot_distance > 1.0:
        twist.linear.x = 0.2
        twist.angular.z = bot_angle * 0.5
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

def random_motion():
    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = random.uniform(-1.0, 1.0)
    cmd_vel_pub.publish(twist)

def avoid_obstacle():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = random.choice([-1.0, 1.0])
    cmd_vel_pub.publish(twist)

def lidar_callback(data):
    closest_bot = process_lidar_data(data)

    if closest_bot:
        bot_distance, bot_angle, ratio = closest_bot
        if bot_distance > 0.9:
            print(f"Moving towards bot at distance: {bot_distance:.2f}, angle: {math.degrees(bot_angle):.2f}°, ratio: {ratio}")
            move_towards_bot(bot_distance, bot_angle)
        else:
            print(f"Bot detected within 1 meter, stopping. Distance: {bot_distance:.2f}, angle: {math.degrees(bot_angle):.2f}°, ratio: {ratio}")
            move_towards_bot(bot_distance, bot_angle)
    else:
        obstacle_distance = min(data.ranges)
        if obstacle_distance < 0.6:
            print("Avoiding obstacle")
            avoid_obstacle()
        else:
            print("No bot detected, moving randomly")
            random_motion()

def main():
    global cmd_vel_pub

    rospy.init_node('lidar_processor', anonymous=True)
    namespace = rospy.get_namespace()
    lidar_topic = f'{namespace}/scan'
    cmd_vel_topic = f'{namespace}/cmd_vel'

    rospy.Subscriber(lidar_topic, LaserScan, lidar_callback)
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
