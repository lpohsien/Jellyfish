#!/usr/bin/env python

"""
This code provides:

1   A service to set or update a goalpoint for the drone, and have the drone fly to it
2   A service to cancle the previously set goalpoint for the drone and have it stay still
3   TODO A service to set a series of waypoints for the drone
"""

#Fused global position of vehicle, updated by the /dhi_sdk/gps_position topic
current_latitude = None
current_longitude = None
current_altitude = None

initial_altitude = None #In meters

goal_latitude = None
goal_longitude = None
goal_altitude = None #In meters
goal_yaw = None #In radians

max_speed = 5
min_speed = 0.75
speed = 5 #Distance between generated intermediate waypoints. Controls the speed of the drone betwwen goalpoints
moving_towards_goal = False

import rospy
import sensor_msgs.msg
import std_msgs.msg
import math
import tf.transformations
import dji_sdk.srv

from jellyfish.srv import set_goalpoint

def initialise():
    global initial_altitude

    rospy.loginfo("Requesting drone control authority...")
    rospy.wait_for_service("/dji_sdk/sdk_control_authority")
    try:
        srv = rospy.ServiceProxy("/dji_sdk/sdk_control_authority", dji_sdk.srv.SDKControlAuthority)
        if srv(1).result:
            rospy.loginfo("Obtained control of the drone!")
        else:
            rospy.logerr("Failed to obtain control of the drone :<")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    rospy.loginfo("Taking off...")    
    rospy.wait_for_service("/dji_sdk/drone_task_control")
    try:
        srv = rospy.ServiceProxy("/dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)
        if srv(4).result:
            rospy.loginfo("We are flying!")
        else:
            rospy.logerr("Failed to take off :<")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    initial_coordinates = rospy.client.wait_for_message("/dji_sdk/gps_position", sensor_msgs.msg.NavSatFix)
    initial_altitude = initial_coordinates.altitude


def handle_current_gps_coordinates(msg):
    global current_latitude 
    global current_longitude
    global current_altitude

    current_latitude = msg.latitude
    current_longitude = msg.longitude
    current_altitude = msg.altitude

def handle_IMU_data(msg):
    #Uses IMU data to get yaw of drone.
    #TODO perhaps can fuse with GPS data in the future?
    global current_yaw
    current_orientation_quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    current_yaw = tf.transformations.euler_from_quaternion(current_orientation_quaternion)[2] #Get yaw in radians

def handle_set_goalpoint(req):
    global goal_latitude
    global goal_longitude
    global goal_altitude
    global goal_yaw
    global speed
    global moving_towards_goal

    goal_latitude = req.latitude
    goal_longitude = req.longitude
    goal_altitude = req.altitude
    goal_yaw = req.yaw
    speed = req.speed
    moving_towards_goal = True
    msg = f"moving towards goalpoint at \nlatitude: {goal_latitude}\nlongitude: {goal_longitude}\naltitude: {goal_altitude}\nyaw: {goal_yaw}\nwith speed {speed}"
    rospy.loginfo(msg)
    return msg

def waypoint_handler_server():
    waypoint_service = rospy.Service("set_goalpoint", set_goalpoint, handle_set_goalpoint)
    rospy.loginfo("Drone_movement_control Server ready")

def joy_broadcaster():
    #Broadcasts command to control drone on ddji_sdk/flight_control_setpoint_ENUposition_yaw. Message is a sensor_msgs.msg.Joy message with
    #the following axes: [X position offset, Y position offset, Z position offset, yaw angle] in ENU ground frame
    #Currently programmed to move a straight line path
    global moving_towards_goal
    global speed

    update_rate = 5
    pub = rospy.Publisher("/dji_sdk/flight_control_setpoint_ENUposition_yaw", sensor_msgs.msg.Joy, queue_size=1)
    r = rospy.Rate(update_rate)
    goal_reach_timer = 0
    while not rospy.is_shutdown():
        if moving_towards_goal:
            if speed < min_speed:
                speed = min_speed
            if speed > max_speed:
                speed = max_speed
            crow_fly_distance, altitude_difference, bearing, distance = get_distance_and_bearings_from_goal()
            axes = []
            command = sensor_msgs.msg.Joy()
            if distance > speed:
                goal_reach_timer = 0
                plane_offset = speed * math.cos(math.atan(altitude_difference/crow_fly_distance))
                z_offset = speed * math.sin(math.atan(altitude_difference/crow_fly_distance))
                axes.append(plane_offset*math.sin(bearing)) #offset in the x direction for the shortest distance great circle arc
                axes.append(plane_offset*math.cos(bearing)) #offset in the y direction for the shortest distance great circle arc
                axes.append(current_altitude-initial_altitude + z_offset) #offset in the z direction
                axes.append(0)
            else:
                goal_reach_timer += 1/update_rate
                if goal_reach_timer < 5: #If within 5m of goal position, check if the drone remains in that location for 5s
                    axes.append(crow_fly_distance*math.sin(bearing)) #offset in the x direction for the shortest distance great circle arc
                    axes.append(crow_fly_distance*math.cos(bearing)) #offset in the y direction for the shortest distance great circle arc
                    axes.append(goal_altitude-initial_altitude) #offset in the z direction
                    axes.append(0)
                else:
                    rospy.loginfo("drone has reached goal point")
                    moving_towards_goal = False
            if axes == []:
                continue
            command.axes = axes
            pub.publish(command)
            r.sleep()



def get_distance_and_bearings_from_goal():
    #Uses the harversine formula to get distance
    current_latitude_rad = current_latitude * math.pi /180
    goal_latitude_rad = goal_latitude * math.pi /180
    latitude_difference_rad = (goal_latitude - current_latitude) * math.pi /180
    longitude_difference_rad = (goal_longitude - current_longitude) * math.pi /180
    altitude_difference = goal_altitude - current_altitude

    a = ((math.sin(latitude_difference_rad/2)) ** 2) + math.cos(current_latitude_rad) * math.cos(goal_latitude_rad) * ((math.sin(longitude_difference_rad/2)) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    crow_fly_distance = 6371000 * c

    #Get bearings. 0 is North pi/2 is East
    y = math.sin(longitude_difference_rad) * math.cos(goal_latitude_rad)
    x = math.cos(current_latitude_rad) * math.sin(goal_latitude_rad) - math.sin(current_latitude_rad) * math.cos(goal_latitude_rad) * math.cos(longitude_difference_rad)
    bearing = math.atan2(y, x)

    distance = math.sqrt(crow_fly_distance**2 + altitude_difference**2)

    return [crow_fly_distance, altitude_difference, bearing, distance]


if __name__ == "__main__":
    rospy.init_node("waypoint_handler_server")
    initialise()
    rospy.Subscriber("/dji_sdk/gps_position",
                    sensor_msgs.msg.NavSatFix,
                    handle_current_gps_coordinates)
    rospy.Subscriber("/dji_sdk/imu",
                    sensor_msgs.msg.Imu,
                    handle_IMU_data)
    waypoint_handler_server()
    joy_broadcaster()
    rospy.spin()

