# Turtle-Bot Based Swarm Robots


## Key Features
- **Decentralized Multi-Robot System:** Utilizes 2D LiDAR sensors to achieve collision avoidance and swarm aggregation without inter-robot communication.
- **Scalable and Fault-Tolerant:** The algorithm ensures robustness and scalability in various environments.
- **Mathematical Modeling:** Employs a ratio threshold model to accurately detect the presence of nearby robots.

## How It Works
1. **Sensing:** Each TurtleBot uses 2D LiDAR sensors to scan the environment within a 120-degree field.
2. **Detection:** The algorithm differentiates between other TurtleBots and obstacles based on sensor data.

   follwing points are not implemented in the lidar_data_procesosr code to .....to refer below points please  refer community.py code
4. **Aggregation:** The system continuously monitors and estimates the positions of individual robots, enabling swarm behavior.
5. **Collision Avoidance:** Ensures that robots avoid obstacles and each other to maintain smooth operation.

## Installation
1. Setup turtlebot3 environtment[from here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
2. Create a python pkg and clone this repo in  catkin_ws/src/your_pkg/src/scripts.my pkg is called turtlebot3_lidar_processing.
3. export TURTLEBOT3_MODEL=burger and then  lauch multi robot simultaion (make sure you have follwowed the simulation steps for turtle bot3 burger model)
4.  **Start the simulation:**
    ```bash
    export TURTLEBOT3_MODEL=burger
    ```
    ```bash
 
    roslaunch turtlebot3_gazebo multi_turtlebot3.launch
    ```

5. catkin_make and Run the code.
 ```bash
 
    rosrun turtlebot3_lidar_processing data_proccesing.py

    ```
    


## Explanation Videos
### Project Demonstration
[Project Demo](https://youtu.be/Zxg1iteGq_Y?si=wjVbnHYKQ7Kt8mHI)

### Deomstration Videos
[Algorithm ](https://img.youtube.com/vi/VIDEO_ID/0.jpg)



### Bug Report
 Any obstacle with features similar to those of the TurtleBot would be incorrectly classified as another TurtleBot. This represents a significant limitation of the current code.

## Acknowledgements
Special thanks to my mentors, Prof. Leena Vachhani and Sweksha Jain from ARMS lab Systems and Control Engineering, IIT-Bombay, for their guidance and support throughout this project.



