#!/usr/bin/env python  

"""
This node simulates the dji_ros_sdk for SITL use. At the moment, only movement control using /dji_sdk/flight_control_setpoint_ENUposition_yaw
can be simulated
"""

import std_msgs.msg
import rospy
import sensor_msgs.msg
import math
import geometry_msgs.msg
import mav_msgs.msg
import dji_sdk.srv
import dji_sdk.msg
import tf.transformations

GIMBAL_ROLL_MIN_ANGLE = -0.45
GIMBAL_ROLL_MAX_ANGLE = 0.45
GIMBAL_PITCH_MIN_ANGLE = -1.57
GIMBAL_PITCH_MAX_ANGLE = 1.57
GIMBAL_YAW_MIN_ANGLE = -1.57
GIMBAL_YAW_MAX_ANGLE = 1.57

local_pos_ref_set = False
drone_activated = False
control_authority = False
command_in_queue = False
gimbal_command_in_queue = False

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

current_gimbal_orientation = geometry_msgs.msg.Vector3Stamped()
current_gimbal_orientation.vector.x = 0
current_gimbal_orientation.vector.y = 0
current_gimbal_orientation.vector.z = 0

gimbal_roll_command = std_msgs.msg.Float64()
gimbal_pitch_command = std_msgs.msg.Float64()
gimbal_yaw_command = std_msgs.msg.Float64()

gimbal_roll_command.data = 0.0
gimbal_pitch_command.data = 0.0
gimbal_yaw_command.data = 0.0

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

def handle_set_local_position_ref_request(req):
    global local_pos_ref_set
    rospy.loginfo("local position ref set")
    local_pos_ref_set = True
    return True
        
def handle_current_pose(msg):
    global current_pose
    current_pose = msg

def handle_current_gimbal_pose(msg):
    global current_gimbal_orientation
    quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    current_gimbal_orientation.vector.x = euler[0]
    current_gimbal_orientation.vector.y = -euler[1]
    current_gimbal_orientation.vector.z = euler[2]

def handle_gimbal_command(msg):
    # Currently this only handles the absolute value mode. Dont think that we are using the relative positioning command yet, so we can cross
    # the bridge when we come to it
    global gimbal_roll_command
    global gimbal_pitch_command
    global gimbal_yaw_command
    global gimbal_command_in_queue

    gimbal_roll_command.data = msg.roll
    gimbal_pitch_command.data = -msg.pitch
    gimbal_yaw_command.data = msg.yaw
    gimbal_command_in_queue = True

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

def local_position_publisher(publisher):
    if local_pos_ref_set:
        local_position_message = geometry_msgs.msg.PointStamped()
        local_position_message.point.x = current_pose.position.x
        local_position_message.point.y = current_pose.position.y
        local_position_message.point.z = current_pose.position.z
        publisher.publish(local_position_message)

def gimbal_position_publisher(publisher):
    publisher.publish(current_gimbal_orientation)

def gimbal_command_publisher(roll_publisher, pitch_publisher, yaw_publisher):
    global gimbal_command_in_queue

    if gimbal_command_in_queue:
        roll_publisher.publish(gimbal_roll_command)
        pitch_publisher.publish(gimbal_pitch_command)
        yaw_publisher.publish(gimbal_yaw_command)
        gimbal_command_in_queue = False

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
    rospy.Subscriber("/dji_m210/gimbal_ground_truth/pose",
                    geometry_msgs.msg.Pose,
                    handle_current_gimbal_pose)
    rospy.Subscriber("/dji_sdk/gimbal_angle_cmd",
                    dji_sdk.msg.Gimbal,
                    handle_gimbal_command
                    )

    #Services
    sdk_control_authority_server = rospy.Service("/dji_sdk/sdk_control_authority", dji_sdk.srv.SDKControlAuthority, handle_sdk_control_authority_request)
    drone_task_control_server = rospy.Service("/dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl, handle_drone_task_control_request)
    set_local_pos_ref_server = rospy.Service("/dji_sdk/set_local_pos_ref", dji_sdk.srv.SetLocalPosRef, handle_set_local_position_ref_request)

    #Publishers
    r = rospy.Rate(10)
    gps_pub = rospy.Publisher("/dji_sdk/gps_position", sensor_msgs.msg.NavSatFix, queue_size=5)
    command_trajectory_pub = rospy.Publisher("/dji_m210/position_command/trajectory", mav_msgs.msg.CommandTrajectory, queue_size=5)
    attitude_pub = rospy.Publisher("/dji_sdk/attitude", geometry_msgs.msg.QuaternionStamped, queue_size = 5)
    local_position_pub = rospy.Publisher("/dji_sdk/local_position", geometry_msgs.msg.PointStamped, queue_size = 5)
    gimbal_position_pub = rospy.Publisher("/dji_sdk/gimbal_angle", geometry_msgs.msg.Vector3Stamped, queue_size = 5)
    gimbal_roll_pub = rospy.Publisher("/dji_m210/gimbal_roll_controller/command", std_msgs.msg.Float64, queue_size = 5)
    gimbal_pitch_pub = rospy.Publisher("/dji_m210/gimbal_pitch_controller/command", std_msgs.msg.Float64, queue_size = 5)
    gimbal_yaw_pub = rospy.Publisher("/dji_m210/gimbal_yaw_controller/command", std_msgs.msg.Float64, queue_size = 5)
     
    while not rospy.is_shutdown():
        gps_publisher(gps_pub)
        command_trajectory_publisher(command_trajectory_pub)
        attitude_publisher(attitude_pub)
        local_position_publisher(local_position_pub)
        gimbal_position_publisher(gimbal_position_pub)
        gimbal_command_publisher(gimbal_roll_pub, gimbal_pitch_pub, gimbal_yaw_pub)
        r.sleep()
    rospy.spin()