#!/usr/bin/env python  

#TODO: Drone activation service

"""
This node simulates the dji_ros_sdk for SITL use. At the moment, only movement control using /dji_sdk/flight_control_setpoint_ENUposition_yaw
can be simulated
"""

import rospy
import sensor_msgs.msg
import math
import geometry_msgs.msg
import mav_msgs.msg
import dji_sdk.srv

drone_activated = False
control_authority = False
command_in_queue = False

# As this uses the equirectangular projection approximation, this latitude allows for the best approximation
spawn_latitude = 0
spawn_longitude = 0
spawn_altitude = 0

current_pose = geometry_msgs.msg.Pose() 
current_pose.position.x = 0
current_pose.position.y = 0
current_pose.position.z = 0
current_pose.orientation.x = 0
current_pose.orientation.y = 0
current_pose.orientation.z = 0
current_pose.orientation.w = 1
command_trajectory_msg = mav_msgs.msg.CommandTrajectory()
command_trajectory_msg.position.x = 0
command_trajectory_msg.position.y = 0
command_trajectory_msg.position.z = 0
command_trajectory_msg.yaw = 0

def handle_sdk_control_authority_request(req):
    global control_authority
    if req.control_enable == 1:
        rospy.loginfo("SDK control authority obtained")
        control_authority = True
        return [True, 0, 0, 0]
    elif req.control_enable == 0:
        rospy.loginfo("SDK control authority released")
        control_authority = False
        return [False, 0, 0, 0]
    else:
        rospy.logerr("Invalid input for SDK control authority request")
        return [False, 0, 0, 0]

def handle_drone_task_control_request(req):
    if not control_authority:
        rospy.logwarn("Invalid: no drone control authority. Failed drone task control request")
    else:
        if req.task == 4:
            global command_in_queue
            global command_trajectory_msg
            command_trajectory_msg.position.z = 1
            command_in_queue = True
            return [True, 0, 0, 0]
        
def handle_current_pose(msg):
    global current_pose
    current_pose = msg


def handle_joy_command_control_setpoint_ENUposition_yaw(msg):
    #Dji's SDK controls the drone by publishing a sensor_msgs/Joy controlVelYawRate msg on /dji_sdk/flight_control_setpoint_ENUposition_yaw
    #We are going to mimic it by processing same message, and moving the drone accordingly
    if not control_authority:
        rospy.logwarn("Error: no drone control authority")
    else:
        global command_trajectory_msg
        global command_in_queue
        x, y, z, yaw = msg.axes
        x_goal = current_pose.position.x + x
        y_goal = current_pose.position.y + y
        z_goal = z
        msg = mav_msgs.msg.CommandTrajectory()
        msg.position.x = x_goal
        msg.position.y = y_goal
        msg.position.z = z_goal
        msg.yaw = yaw
        command_trajectory_msg = msg
        command_in_queue = True

def gps_publisher(publisher):
    global current_pose
    gps_message = sensor_msgs.msg.NavSatFix()
    gps_message.latitude = spawn_latitude + math.degrees(math.atan(current_pose.position.y/6371000))
    gps_message.longitude = spawn_longitude + math.degrees(math.atan(current_pose.position.x/6371000))
    gps_message.altitude = spawn_altitude + current_pose.position.z
    publisher.publish(gps_message)

def command_trajectory_publisher(publisher):
    global command_in_queue
    global command_trajectory_msg
    if command_in_queue:
        publisher.publish(command_trajectory_msg)
        command_in_queue = False

def attitude_publisher(publisher):
    attitude_message = geometry_msgs.msg.QuaternionStamped()
    attitude_message.quaternion.x = current_pose.orientation.x
    attitude_message.quaternion.y = current_pose.orientation.y
    attitude_message.quaternion.z = current_pose.orientation.z
    attitude_message.quaternion.w = current_pose.orientation.w
    publisher.publish(attitude_message)

if __name__ == '__main__':
    #Subscribers
    rospy.init_node("fake_dji_sdk")
    rospy.loginfo("Started fake_dji_sdk node! ")
    rospy.Subscriber("/dji_m210/ground_truth/pose",
                    geometry_msgs.msg.Pose,
                    handle_current_pose)
    rospy.Subscriber("/dji_sdk/flight_control_setpoint_ENUposition_yaw",
                    sensor_msgs.msg.Joy,
                    handle_joy_command_control_setpoint_ENUposition_yaw)

    #Services
    sdk_control_authority_server = rospy.Service("/dji_sdk/sdk_control_authority", dji_sdk.srv.SDKControlAuthority, handle_sdk_control_authority_request)
    drone_task_control_server = rospy.Service("/dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl, handle_drone_task_control_request)

    #Publishers
    r = rospy.Rate(8)
    gps_pub = rospy.Publisher("/dji_sdk/gps_position", sensor_msgs.msg.NavSatFix, queue_size=5)
    command_trajectory_pub = rospy.Publisher("/dji_m210/position_command/trajectory", mav_msgs.msg.CommandTrajectory, queue_size=5)
    attitude_pub = rospy.Publisher("/dji_sdk/attitude", geometry_msgs.msg.QuaternionStamped, queue_size = 5)
    while not rospy.is_shutdown():
        gps_publisher(gps_pub)
        command_trajectory_publisher(command_trajectory_pub)
        attitude_publisher(attitude_pub)
        r.sleep()
    rospy.spin()