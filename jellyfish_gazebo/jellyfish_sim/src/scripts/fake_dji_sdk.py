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
import bb_msgs.msg
import dji_sdk.srv
import dji_sdk.msg
import tf_conversions
import tf2_ros

drone_name = rospy.get_param("drone_name", "dji_m210")

local_pos_ref_set = False
drone_activated = False
control_authority = False
command_in_queue = False
active = False # toggled by calling the /dji_sdk/drone_task_control service
gimbal_command_in_queue = False

# As this uses the equirectangular projection approximation, this latitude allows for the best approximation
spawn_latitude = rospy.get_param("spawn_latitude", "0")
spawn_longitude = rospy.get_param("spawn_longitude", "0")
spawn_altitude = rospy.get_param("spawn_altitude", "0")

current_pose = geometry_msgs.msg.Pose() 
current_pose.position.x = 0
current_pose.position.y = 0
current_pose.position.z = 0
current_pose.orientation.x = 0
current_pose.orientation.y = 0
current_pose.orientation.z = 0
current_pose.orientation.w = 1

gimbal_roll_command = std_msgs.msg.Float64()
gimbal_pitch_command = std_msgs.msg.Float64()
gimbal_yaw_command = std_msgs.msg.Float64()

gimbal_roll_command.data = 0.0
gimbal_pitch_command.data = 0.0
gimbal_yaw_command.data = 0.0

goal_msg = bb_msgs.msg.MoveToCartesianCoordinatesActionGoal()
goal_msg.goal.x_goal = 0
goal_msg.goal.y_goal = 0
goal_msg.goal.z_goal = 0
goal_msg.goal.yaw_goal = 0

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
    global active
    global command_in_queue
    global goal_msg
    if not control_authority:
        rospy.logwarn("Invalid: no drone control authority. Failed drone task control request")
    elif req.task == 4:
        rospy.loginfo("Drone task control request: takeoff")
        goal_msg.goal.z_goal = 1
        command_in_queue = True
        active = True
        return [True, 0, 0, 0]
    elif req.task == 6:
        rospy.loginfo("Drone task control request: land")
        active = False
        return [True, 0, 0, 0]

def handle_set_local_position_ref_request(req):
    global local_pos_ref_set
    rospy.loginfo("local position ref set")
    local_pos_ref_set = True
    return True
        
def handle_current_pose(msg):
    global current_pose
    current_pose = msg

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
        global goal_msg
        global command_in_queue
        x, y, z, yaw = msg.axes
        x_goal = current_pose.position.x + x
        y_goal = current_pose.position.y + y
        z_goal = z
        goal_msg = bb_msgs.msg.MoveToCartesianCoordinatesActionGoal()
        goal_msg.goal.x_goal = x_goal
        goal_msg.goal.y_goal = y_goal
        goal_msg.goal.z_goal = z_goal
        goal_msg.goal.yaw_goal = yaw
        command_in_queue = True

def gps_publisher(publisher):
    global current_pose
    gps_message = sensor_msgs.msg.NavSatFix()
    gps_message.latitude = spawn_latitude + math.degrees(math.atan(current_pose.position.y/6371000))
    gps_message.longitude = spawn_longitude + math.degrees(math.atan(current_pose.position.x/6371000))
    gps_message.altitude = spawn_altitude + current_pose.position.z
    publisher.publish(gps_message)

def goal_publisher(publisher):
    global command_in_queue
    global goal_msg
    if command_in_queue:
        publisher.publish(goal_msg)
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

def gimbal_position_publisher(publisher, tf_buffer):
    try:
        gimbal_roll_q = tf_buffer.lookup_transform("{}/gimbal_base_plate".format(drone_name), "{}/gimbal_roll_link".format(drone_name), rospy.Time()).transform.rotation
        gimbal_roll = tf_conversions.transformations.euler_from_quaternion([gimbal_roll_q.x, gimbal_roll_q.y, gimbal_roll_q.z, gimbal_roll_q.w])[0]
        gimbal_pitch_q = tf_buffer.lookup_transform("{}/gimbal_roll_link".format(drone_name), "{}/gimbal_pitch_link".format(drone_name), rospy.Time()).transform.rotation
        gimbal_pitch = tf_conversions.transformations.euler_from_quaternion([gimbal_pitch_q.x, gimbal_pitch_q.y, gimbal_pitch_q.z, gimbal_pitch_q.w])[1]
        gimbal_yaw_q = tf_buffer.lookup_transform("{}/gimbal_pitch_link".format(drone_name), "{}/gimbal_yaw_link".format(drone_name), rospy.Time()).transform.rotation
        gimbal_yaw = tf_conversions.transformations.euler_from_quaternion([gimbal_yaw_q.x, gimbal_yaw_q.y, gimbal_yaw_q.z, gimbal_yaw_q.w])[2]
        gimbal_msg = geometry_msgs.msg.Vector3Stamped()
        gimbal_msg.header.stamp = rospy.Time.now()
        gimbal_msg.vector.x = gimbal_roll
        gimbal_msg.vector.y = -gimbal_pitch
        gimbal_msg.vector.z = gimbal_yaw
        publisher.publish(gimbal_msg)
        rospy.logdebug_once("tf2 found!")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logdebug_throttle_identical(5, "Waiting for tf2 ... {}".format(e))
        r.sleep()

def gimbal_command_publisher(roll_publisher, pitch_publisher, yaw_publisher):
    global gimbal_command_in_queue

    if gimbal_command_in_queue:
        roll_publisher.publish(gimbal_roll_command)
        pitch_publisher.publish(gimbal_pitch_command)
        yaw_publisher.publish(gimbal_yaw_command)
        gimbal_command_in_queue = False
        rospy.logdebug_throttle_identical(5, "Gimbal command published: {} {} {}".format(gimbal_roll_command.data, gimbal_pitch_command.data, gimbal_yaw_command.data))

def get_rosparam(key, rate, timeout):
    if(timeout <= 0): return
    if rospy.has_param(key):
        rospy.logdebug("Rosparam {}: {} found!".format(key, rospy.get_param(key)))
        return rospy.get_param(key)
    else:
        rospy.logerr_throttle_identical(5, "Cannot find rosparam named {}{}. Set it using 'rosparam set' or change it in fake_dji_sdk.launch"
            .format(rospy.get_namespace(), key))
        rospy.Rate(rate).sleep()
        return get_rosparam(key, rate, timeout - 1.0/rate)

def repub_gazebo_command(command):
    global active
    global gazebo_command_publisher
    if active:
        gazebo_command_publisher.publish(command)
    else:
        gazebo_command_publisher.publish(mav_msgs.msg.Actuators(angular_velocities=[0, 0, 0, 0]))


if __name__ == '__main__':
    #Subscribers
    rospy.init_node("fake_dji_sdk", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Started fake_dji_sdk node! ")

    drone_name = rospy.get_param("drone_name")

    current_pose_topic = get_rosparam("current_pose_topic", 1, 2)
    joy_command_topic = get_rosparam("joy_command_topic", 1, 2)
    gimbal_command_topic = get_rosparam("gimbal_command_topic", 1, 2)
    control_authority_topic = get_rosparam("control_authority_topic", 1, 2)
    drone_task_topic = get_rosparam("drone_task_topic", 1, 2)
    local_pos_ref_topic = get_rosparam("local_pos_ref_topic", 1, 2)
    gps_position_topic = get_rosparam("gps_position_topic", 1, 2)
    goal_topic = get_rosparam("goal_topic", 1, 2)
    gazebo_command_topic = get_rosparam("gazebo_command_topic", 1, 2)
    gazebo_command_output_topic = "{}/gazebo/command/motor_speed".format(drone_name)
    attitude_topic = get_rosparam("attitude_topic", 1, 2)
    local_position_topic = get_rosparam("local_position_topic", 1, 2)
    gimbal_angle_topic = get_rosparam("gimbal_angle_topic", 1, 2)
    gimbal_roll_command_topic = get_rosparam("gimbal_roll_command_topic", 1, 2)
    gimbal_pitch_command_topic = get_rosparam("gimbal_pitch_command_topic", 1, 2)
    gimbal_yaw_command_topic = get_rosparam("gimbal_yaw_command_topic", 1, 2)

    #Publishers
    r = rospy.Rate(10)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    gps_pub = rospy.Publisher(gps_position_topic, sensor_msgs.msg.NavSatFix, queue_size=5)
    goal_pub = rospy.Publisher(goal_topic, bb_msgs.msg.MoveToCartesianCoordinatesActionGoal, queue_size=5)
    attitude_pub = rospy.Publisher(attitude_topic, geometry_msgs.msg.QuaternionStamped, queue_size = 5)
    local_position_pub = rospy.Publisher(local_position_topic, geometry_msgs.msg.PointStamped, queue_size = 5)
    gimbal_position_pub = rospy.Publisher(gimbal_angle_topic, geometry_msgs.msg.Vector3Stamped, queue_size = 5)
    gimbal_roll_pub = rospy.Publisher(gimbal_roll_command_topic, std_msgs.msg.Float64, queue_size = 5)
    gimbal_pitch_pub = rospy.Publisher(gimbal_pitch_command_topic, std_msgs.msg.Float64, queue_size = 5)
    gimbal_yaw_pub = rospy.Publisher(gimbal_yaw_command_topic, std_msgs.msg.Float64, queue_size = 5)
    gazebo_command_publisher = rospy.Publisher(gazebo_command_output_topic, mav_msgs.msg.Actuators, queue_size = 5)

    #Subscribers
    rospy.Subscriber(current_pose_topic,
                    geometry_msgs.msg.Pose,
                    handle_current_pose)
    rospy.Subscriber(joy_command_topic,
                    sensor_msgs.msg.Joy,
                    handle_joy_command_control_setpoint_ENUposition_yaw)
    rospy.Subscriber(gimbal_command_topic,
                    dji_sdk.msg.Gimbal,
                    handle_gimbal_command
                    )

    rospy.Subscriber(gazebo_command_topic,
                    mav_msgs.msg.Actuators,
                    repub_gazebo_command)

    #Services
    sdk_control_authority_server = rospy.Service(control_authority_topic, dji_sdk.srv.SDKControlAuthority, handle_sdk_control_authority_request)
    drone_task_control_server = rospy.Service(drone_task_topic, dji_sdk.srv.DroneTaskControl, handle_drone_task_control_request)
    set_local_pos_ref_server = rospy.Service(local_pos_ref_topic, dji_sdk.srv.SetLocalPosRef, handle_set_local_position_ref_request)
    rospy.logwarn(str(goal_pub) + "@"+ str(goal_topic))
    while not rospy.is_shutdown():
        gps_publisher(gps_pub)
        goal_publisher(goal_pub)
        attitude_publisher(attitude_pub)
        local_position_publisher(local_position_pub)
        gimbal_position_publisher(gimbal_position_pub, tf_buffer)
        gimbal_command_publisher(gimbal_roll_pub, gimbal_pitch_pub, gimbal_yaw_pub)
        r.sleep()
    rospy.loginfo("Stopped fake_dji_sdk node! ")