# Turtle-Bot Based Swarm Robots
![Screenshot](https://github.com/Jayesh-Khalane/Turtle-Bot-based-Swarm/blob/main/Screenshot%202024-12-20%20221032.png?raw=true)


## Key Features
- **Decentralized Multi-Robot System:** Utilizes 2D LiDAR sensors to achieve collision avoidance and swarm aggregation without inter-robot communication.
- **Scalable and Fault-Tolerant:** The algorithm ensures robustness and scalability in various environments.
- **Mathematical Modeling:** Employs a ratio threshold model to accurately detect the presence of nearby robots.

## How It Works
1. **Sensing:** Each TurtleBot uses 2D LiDAR sensors to scan the environment within a 120-degree field.
2. **Detection:** The algorithm differentiates between other TurtleBots and obstacles based on sensor data.
   

   Following points are not included in the lidar_data_processor.py code for that please checkout community.py
4. **Aggregation:** Continuously monitors and estimates the positions of individual robots, enabling swarm behavior.
5. **Collision Avoidance:** Ensures that robots avoid obstacles and each other to maintain smooth operation.

## Installation
1. **Setup TurtleBot3 Environment:** Follow the instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) to set up the TurtleBot3 environment.
2. **Create a Python Package:** Clone this repository in `catkin_ws/src/your_pkg/src/scripts`. For example, my package is called `turtlebot3_lidar_processing`.
3. **Set TurtleBot3 Model:**
    ```bash
    export TURTLEBOT3_MODEL=burger
    ```
4. **Launch Multi-Robot Simulation:** Make sure you have followed the simulation steps for the TurtleBot3 burger model.
    ```bash
    roslaunch turtlebot3_gazebo multi_turtlebot3.launch
    ```
5. **Build and Run the Code:**
    ```bash
    catkin_make
    ```
    ```bash
    rosrun turtlebot3_lidar_processing data_processing.py
    ```

## Explanation Videos
### Project Demonstration
[Project Demo](https://youtu.be/Zxg1iteGq_Y?si=wjVbnHYKQ7Kt8mHI)

### Identification Demonstration
[Click here](https://youtu.be/2JdhxI7-Lgw)

### Static and Dynamic
[Click here](https://youtu.be/9UPoltkLEds)

### Basic Clustering without Obstacles
[Click here](https://youtu.be/FyDi8gXWTsE)

### Basic Clustering with Obstacles
- [Without LiDAR Visualization](https://youtu.be/xLgDODAIH-8)
- [With LiDAR Visualization](https://youtu.be/H-vEGUo1Xfo)

## Bug Report
Any obstacle with features similar to those of the TurtleBot would be incorrectly classified as another TurtleBot.


