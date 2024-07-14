# Turtle-Bot Based Swarm Robots


## Key Features
- **Decentralized Multi-Robot System:** Utilizes 2D LiDAR sensors to achieve collision avoidance and swarm aggregation without inter-robot communication.
- **Scalable and Fault-Tolerant:** The algorithm ensures robustness and scalability in various environments.
- **Mathematical Modeling:** Employs a ratio threshold model to accurately detect the presence of nearby robots.

## How It Works
1. **Sensing:** Each TurtleBot uses 2D LiDAR sensors to scan the environment within a 120-degree field.
2. **Detection:** The algorithm differentiates between other TurtleBots and obstacles based on sensor data.

   yet to implement in the lidar_data_procesosr code to .....to refer below points  can refer community.py code
4. **Aggregation:** The system continuously monitors and estimates the positions of individual robots, enabling swarm behavior.
5. **Collision Avoidance:** Ensures that robots avoid obstacles and each other to maintain smooth operation.

## Installation
1. Setup turtlebot3 environtment[from here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
2. Create a py pkg and clone this repo in its catkin_ws/src/your_pkg/src/scripts.
3. export TURTLEBOT3_MODEL=burger and then  lauch multi robot simultaion (make sure you have follwowed the simulation steps for turtle bot3 burger model)
4.  **Start the simulation:**
    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```

## Videos
### Project Demonstration
[![Project Demo](https://youtu.be/Zxg1iteGq_Y)

### Algorithm 
[![Algorithm ](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/VIDEO_ID)

in a certain ratio range are identified as other TurtleBots, while those that do not are considered obstacles.

### Bug Report
 Any obstacle with features similar to those of the TurtleBot would be incorrectly classified as another TurtleBot. This represents a significant limitation of the current code.

## Acknowledgements
Special thanks to my mentors, Prof. Leena Vachhani and Sweksha Jain from ARMS lab Systems and Control Engineering, IIT-Bombay, for their guidance and support throughout this project.



