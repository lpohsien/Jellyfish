# Overview

## Dependancies

Do note that to use all of these, the following packages are relevant:

- dji_sdk (must for HITL, and really, to run the actual drone)
- dji_sdk_demo (if you wna run dji's demo)
- rotors_simulator (must for SITL)
- video_stream_opencv (CV stuff)
- image_pipeline (CV stuff)

## Features
Currently this repository includes the following features: 

### HITL simulation

- Using this mode would allow the simulation of the drone more accurately, and access to all of dji_sdk topics, features etc. BUT you need the drone   with you and a windows lappy with DJI assistant 2. The dji_sdk the jellyfish uses (version 3.8) is already included in this repository
- To use the dji_sdk ros package, a UserConfig.txt file (found in the root directory of this repo) needs to be in /home/{user}/.ros

### SITL simulation

- Based off the rotors_simulator package
- Drone is simulated with a gimbal.
- Included a urdf model of the dji m100 (due to lack of a mesh for the dji m210). Model can be edited for different drones
- Simulation of individual thrusts being generated by the drone's propeller. Controllers can be written and simulated using this feature. Currently       using basic position controller for controls in this simulation
- Basic position controller (Off which the current dji_sdk mimicing node is based off on)

### A fake dji_sdk node

Mimics some basic functionalities of dji's sdk. It should be noted that the movement controls does not have the drone moving very cleanly (jerky movements over longer distances etc). However, it should be sufficient for testing of drone's logic (missions, behaviour tree etc). Mimiced features include:

- GPS position
- IMU
- Control Authority requesting
- Take off drone task
- Movement based off on the dji_sdk/flight_control_setpoint_ENUposition_yaw
- Gimbal control

### Very basic movement control code 

- Moves the drone to a desired GPS coordinate. 
- This uses dji_sdk/flight_control_setpoint_ENUposition_yaw to move, and can runs in both the HITL simulation (using dji's simulator) and the SITL simulation

### Some aruco tag stuff 

- Stuff im working on for DTLS

# Usage

To run a demonstration of this package, clone this repository into your workspace and build. After that, run the SITL simulation without the basic drone controller using the following command.

`
roslaunch jellyfish_sim simulate.launch
`

In another terminal,run the following command to command the props to spin and generate thrust. Increasing the thrust values would cause the drone to start ascending

`
rostopic pub /dji_m210/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100]}'
`

## SITL

To run a demonstration of movement control using the fake_dji_sdk, run the following 

`
roslaunch jellyfish_sim simulate.launch
`

In a seperate terminal, run

`
rosrun jellyfish_sim fake_dji_sdk.py
`

Then, use the jellyfish launch file. This would start up the relevant nodes for gimbal controls, cartesian and GPS movement

`
roslaunch jellyfish_movement jellyfish.launch
`

Drone movement is done by either using the GPS movement action or the cartesian movement action. The cartesian movement action moves the drone relative to a pose publisher. By default, the drone is spawned in with a landing platform, which publishes pose messages similar to the way marvelmind publishes. Do note that due to how the orientation of the drone relative to the platform is calculated, the drone's movement in the SITL sim might be a little erratic. However, it gets the job done. Am considering to apply some filtering to make it more robust to noise. We can test this functionality by running the following command.

`
rostopic pub /cartesian_coordinates_handler_server/goal jellyfish_movement/MoveToCartesianCoordinatesActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {x_goal: 0, y_goal: 0, z_goal: 4, yaw_goal: 0.0, pose_topic: 'marvelmind_pose'}"
`

This positions the drone directly over the landing platform. The pose topic can be changed according to what we would like to position the drone relative to (say the aruco tag).

Otherwise, we can use the GPS movement action to move the drone

`
rostopic pub /gps_coordinates_handler_server/goal jellyfish_movement/MoveToGPSCoordinatesActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  latitude: 0.0
  longitude: 0.0001 
  height: 5.0" 
publishing and latching message. Press ctrl-C to terminate

`

## HITL

To run a demonstration of movement control using the the actual dji_sdk and the bonkers amount of hardware required to do so, run the following 

`
roslaunch dji_sdk sdk.launch
`

Next, start the simulation in DJI assistant 2. The HITL sim bridge uses the local position of the drone and mimics it in gazebo. Hence, call the following service

`
rosservice call /dji_sdk/set_local_pos_ref
`

In a seperate terminal, run

`
roslaunch jellyfish_sim simulate.launch is_SITL:=false
`

Then, use the jellyfish launch file. This would start up the relevant nodes for gimbal controls, cartesian and GPS movement

`
roslaunch jellyfish_movement jellyfish.launch
`

Movement is the same as in the SITL

Alternatively, control via dji_sdk/flight_control_setpoint_ENUposition_yaw can be tested using the following command. Do note that the altitude input needs to be higher than the spawn altitude (Configurable in dji assistant).

`
rostopic pub /dji_sdk/flight_control_setpoint_ENUposition_yaw sensor_msgs/Joy "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, axes: [1.0, 1.0, your desired altitude, 0.0], buttons:[]}"
`
