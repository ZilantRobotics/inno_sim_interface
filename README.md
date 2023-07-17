# Code-CT Innopolis 3d Simulator

This package is an example of using the Code-CT Innopolis 3d Simulator 
for visual rendering and sensor simulation in combination with a UAV dynamics source from ROS.

# Code-CT Innopolis 3d Simulator Interface

[![VTOL flight in InnoSimulator](img/Sim.jpg)](https://youtu.be/pXbG89qAtq0)

## Code-CT Innopolis 3d Simulator UAV API

Code-CT Innopolis 3d Simulator is currently used as a visual renderer and source of simulated sensor data. Dynamics simulation and control is done externally. The API is adapted for such use.

Code-CT Innopolis 3d Simulator currently supports the following API to control the pose and actuator state of a copter or VTOL aircraft UAV:

### To Simulator

ROS --> Sim

#### GPS position
Topic name is `/sim/gps_position`.
Topic type is `sensor_msgs/NavSatFix`.

#### Orientation
Topic name is `/sim/attitude`.
Topic type is `geometry_msgs/QuaternionStamped`.

#### Actuators state
Topic name is `/sim/actuators`.
Topic type is `sensor_msgs/Joy`.

The axes in the Joy message have the following meanings:

1. FR, cw, rate, rpm (front right motor speed)
2. RL, cw, rate, rpm (rear left motor speed)
3. FL, ccw, rate, rpm (front left motor speed)
4. RR, ccw, rate, rpm (rear right motor speed)
5. left aileron, cw, deg
6. right aileron, cw, deg
7. elevator, cw, deg
8. rudder, cw, deg
9. thrust, throttle, rate, rpm

#### Gimbal
Topic name is `/sim/gimbal_angle`.
Topic type is `geometry_msgs/Vector3Stamped`.

- `vector.x` is roll in degrees, 
- `vector.y` is pitch in degrees, 
- `vector.z` is yaw in degrees.

The roll, pitch and yaw values are global.

### From the 3d simulator

Sim --> ROS

#### Camera

Topic name is `/sim/camera/compressed`.
Topic type is `sensor_msgs/CompressedImage`.


#### LIDAR

Topic name is `/sim/velodyne_points`.
Topic type is `sensor_msgs/PointCloud2`.

## Installation and startup

Please refer to the main project repo: [innopolis_vtol_dynamics](https://github.com/RaccoonlabDev/innopolis_vtol_dynamics)