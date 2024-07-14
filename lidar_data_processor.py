#!/usr/bin/env python3

# Author: Jayesh Khalane
'''
The purpose of this code is to process LIDAR data from multiple TurtleBot3 robots
in a Gazebo simulation to detect other bots and obstacles, estimate their positions,
and determine their movement status.
'''

import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import math

# Global variables
bot_pose = {}  # Dictionary to store the pose of each bot
prev_bot_positions = {}  # Dictionary to store the previous positions of each bot

# Function to find patches in the LIDAR data
def find_patches(data, diff_threshold=0.1):
    patches = []  # List to store detected patches
    current_patch = []  # Current patch being processed
    start_indices = []  # Start indices of the current patch

    for i in range(1, len(data)):
        # Check if the difference between consecutive data points exceeds the threshold
        if abs(data[i] - data[i - 1]) > diff_threshold:
            # If so, append the current patch to patches and reset
            if current_patch:
                patches.append((current_patch, start_indices[0]))
                current_patch = []
                start_indices = []
        current_patch.append(data[i])
        start_indices.append(i)

    # Append the last patch if it exists
    if current_patch:
        patches.append((current_patch, start_indices[0]))

    return patches, len(patches)  # Return the patches and their count

# Function to estimate the global position of a detected object
def estimate_position(distance, angle, bot_pose):
    # Calculate the position relative to the origin
    x_relative = distance * math.cos(angle)
    y_relative = distance * math.sin(angle)

    # Transform relative position to the global position
    bot_x = bot_pose.position.x
    bot_y = bot_pose.position.y
    bot_yaw = 2 * math.atan2(bot_pose.orientation.z, bot_pose.orientation.w)  # Convert quaternion to yaw

    x_global = bot_x + x_relative * math.cos(bot_yaw) - y_relative * math.sin(bot_yaw)
    y_global = bot_y + x_relative * math.sin(bot_yaw) + y_relative * math.cos(bot_yaw)

    return x_global, y_global  # Return the global position

# Function to calculate the movement status of a bot
def calculate_movement_status(bot_id, new_position):
    global prev_bot_positions  # Use the global dictionary
    if bot_id in prev_bot_positions:
        prev_position = prev_bot_positions[bot_id]
        distance_moved = math.sqrt((new_position[0] - prev_position[0]) ** 2 + (new_position[1] - prev_position[1]) ** 2)
        if distance_moved > 0.03:  # Threshold to determine if the bot is moving
            status = "Moving"
        else:
            status = "Static"
    else:
        status = "Unknown"  # For the first detection, we can't determine movement
    prev_bot_positions[bot_id] = new_position  # Update the previous position
    return status  # Return the movement status

# Function to process the LIDAR data
def process_lidar_data(data, namespace):
    if namespace not in bot_pose:
        rospy.logwarn(f"Bot pose for {namespace} not received yet")
        return

    angle_increment = data.angle_increment
    angle_min = data.angle_min

    # Convert -60 and +60 degrees to radians
    min_angle_radians = math.radians(-60)
    max_angle_radians = math.radians(60)

    # Calculate the indices for the -60 to +60 degree range
    start_index = int((min_angle_radians - angle_min) / angle_increment)
    end_index = int((max_angle_radians - angle_min) / angle_increment) + 1

    # Restrict the ranges and angles to the -60 to +60 degree range
    restricted_ranges = data.ranges[start_index:end_index]

    patches, count = find_patches(restricted_ranges, diff_threshold=0.1)  # Find patches in the restricted range

    bot_count = 0  # Initialize bot count
    for i, (patch, patch_start_index) in enumerate(patches):
        patch_no_a = patch[0]
        patch_no_b = patch[-1]
        patch_no_distance = patch[len(patch) // 2]
        deg = len(patch) / 2

        # Calculate the start angle in radians for the restricted range
        start_angle = (patch_start_index + start_index) * angle_increment + angle_min

        # Apply cosine rule to calculate the ratio
        c_squared = patch_no_a ** 2 + patch_no_b ** 2 - 2 * patch_no_a * patch_no_b * math.cos(math.radians(deg))
        c = math.sqrt(c_squared)
        ratio = c / patch_no_distance

        # Conditions to identify a bot
        if 0.7 < patch_no_distance < 1.7 and 0.06 < ratio < 0.162:
            bot_count += 1
            x_global, y_global = estimate_position(patch_no_distance, start_angle, bot_pose[namespace])
            status = calculate_movement_status(f"{namespace}_bot_{bot_count}", (x_global, y_global))
            print(f"{namespace} Bot {bot_count} Dist = {patch_no_distance:.2f}, θ = {math.degrees(start_angle):.2f}°, Estimated Pose= ({x_global:.2f}, {y_global:.2f}), Status = {status}")

        elif 1.7 < patch_no_distance < 2.7 and 0.03 < ratio < 0.06:
            bot_count += 1
            x_global, y_global = estimate_position(patch_no_distance, start_angle, bot_pose[namespace])
            status = calculate_movement_status(f"{namespace}_bot_{bot_count}", (x_global, y_global))
            print(f"{namespace} Bot {bot_count}  Dist = {patch_no_distance:.2f}, θ = {math.degrees(start_angle):.2f}°, Estimated Pose= ({x_global:.2f}, {y_global:.2f}), Status = {status}")

    if bot_count == 0:  # If no bots are detected
        if all(math.isinf(r) for r in restricted_ranges):
            print(f"No visuals for {namespace}")
        else:
            print(f"Obstacle detected by {namespace}")

# LIDAR callback function
def lidar_callback(data, namespace):
    process_lidar_data(data, namespace)

# Model states callback function
def model_states_callback(data):
    global bot_pose  # Use the global dictionary
    for i, name in enumerate(data.name):
        if name.startswith("tb3_"):  # Filter for TurtleBot3 models
            bot_pose[name] = data.pose[i]

# Main function
def main():
    rospy.init_node('lidar_processor', anonymous=True)
    namespace = rospy.get_namespace().strip('/')  # Get the namespace, e.g., 'tb3_0'
    rospy.Subscriber(f'/{namespace}/scan', LaserScan, lidar_callback, callback_args=namespace)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
