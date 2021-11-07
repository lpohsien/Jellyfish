#!/usr/bin/env python

#TODO I have changed the service to use height instead of altitude. Modify this code and the others relevant ones
#to fit the new service call

"""
This code provides:

1   A service to set or update a goalpoint for the drone, and have the drone fly to it
2   A service to cancle the previously set goalpoint for the drone and have it stay still
3   TODO A service to set a series of waypoints for the drone
"""

#Initial altitude of vehicle, since flight_control_setpoint_ENUPosition_yaw uses height and the initial height is taken as reference.
initial_altitude = None

#Fused global position of vehicle, updated by the /dhi_sdk/gps_position topic. Height (above initial altitude) is used here since
#flight_control_setpoint_ENUPosition_yaw uses height 
current_latitude = None
current_longitude = None
current_height = None
current_yaw = None

goal_latitude = None
goal_longitude = None
goal_height = None #In meters
goal_yaw = None #In radians

moving_towards_goal = False
inbound_counter = 0

import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import math
import tf.transformations
import dji_sdk.srv

from jellyfish.srv import set_goalpoint

def activate_drone():
    rospy.loginfo("Activating drone...")
    rospy.wait_for_service("/dji_sdk/activation")
    try:
        srv = rospy.ServiceProxy("/dji_sdk/activation", dji_sdk.srv.Activation)
        if srv().result:
            rospy.loginfo("Drone activated successfully!")
        else:
            rospy.logerr("Drone activation failed! :<")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def request_control_authority():
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

def set_local_pos_ref():
    rospy.loginfo("Taking off...")    
    rospy.wait_for_service("/dji_sdk/drone_task_control")
    try:
        srv = rospy.ServiceProxy("/dji_sdk/set_local_pos_ref", dji_sdk.srv.SetLocalPosRef)
        if srv().result:
            rospy.loginfo("local position reference set")
        else:
            rospy.logerr("dammit")
    except rospy.ServiceException as e:
        rospy.logerr("dammit")

def get_initial_flight_informations():
    global initial_altitude
    global current_height
    global current_longitude
    global current_latitude
    global current_yaw

    #Get relevant GPS data
    initial_coordinates = rospy.client.wait_for_message("/dji_sdk/gps_position", sensor_msgs.msg.NavSatFix)
    initial_altitude = initial_coordinates.altitude
    current_latitude = initial_coordinates.latitude
    current_longitude = initial_coordinates.longitude
    current_height = 0
    rospy.loginfo("initial coordinates received")

    #Get relevant orientation data (basically just yaw)
    inititial_attitude = rospy.client.wait_for_message("/dji_sdk/attitude", geometry_msgs.msg.QuaternionStamped)
    inititial_attitude_quaternion = [inititial_attitude.quaternion.x, inititial_attitude.quaternion.y, inititial_attitude.quaternion.z, inititial_attitude.quaternion.w]
    current_yaw = tf.transformations.euler_from_quaternion(inititial_attitude_quaternion)[2] #Get yaw in radians
    rospy.loginfo("initial orientation received")

def take_off():
    try:
        srv = rospy.ServiceProxy("/dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)
        if srv(4).result:
            rospy.loginfo("We are flying!")
        else:
            rospy.logerr("Failed to take off :<")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def initialise():
    #launching dji_sdk using the dji_sdk launch file would call this already. You can call this just in case
    #activate_drone()
    request_control_authority()
    set_local_pos_ref()
    get_initial_flight_informations()
    take_off()

def handle_current_gps_coordinates(msg):
    global current_latitude 
    global current_longitude
    global current_height

    current_latitude = msg.latitude
    current_longitude = msg.longitude
    current_height = msg.altitude - initial_altitude

def handle_attitude_data(msg):
    #Uses attitude data to get yaw of drone.
    global current_yaw

    current_orientation_quaternion = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
    current_yaw = tf.transformations.euler_from_quaternion(current_orientation_quaternion)[2] #Get yaw in radians

def handle_set_goalpoint(req):
    global goal_latitude
    global goal_longitude
    global goal_height
    global goal_yaw
    global moving_towards_goal

    goal_latitude = req.latitude
    goal_longitude = req.longitude
    goal_height = req.height
    goal_yaw = req.yaw
    moving_towards_goal = True
    msg = f"moving towards goalpoint at \nlatitude: {goal_latitude}\nlongitude: {goal_longitude}\nheight: {goal_height}\nyaw: {goal_yaw}\n"
    rospy.loginfo(msg)
    return msg

def waypoint_handler_server():
    waypoint_service = rospy.Service("set_goalpoint", set_goalpoint, handle_set_goalpoint)
    rospy.loginfo("Drone_movement_control Server ready")

def joy_broadcaster():
    #Broadcasts command to control drone on dji_sdk/flight_control_setpoint_ENUposition_yaw. Message is a sensor_msgs.msg.Joy message with
    #the following axes: [X position offset, Y position offset, Z position, yaw angle] in ENU ground frame
    #Currently programmed to move a straight line path
    global moving_towards_goal
    global inbound_counter

    update_rate = 5
    pub = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUposition_yaw", sensor_msgs.msg.Joy, queue_size=1)
    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        if moving_towards_goal:
            crow_fly_distance, altitude_difference, bearing, distance = get_distance_and_bearings_from_goal()
            yaw_difference = goal_yaw - current_yaw
            #Note that the treshold used below is arbitrary. The simulated controller is abit garbage so it is off by 0.7m in steady state for wtv reason. 
            #In reality we can set altitude difference threshold to be lower
            if crow_fly_distance < 0.5 and altitude_difference < 1 and abs(yaw_difference) < 0.1:
                inbound_counter += 1
                if inbound_counter > (update_rate * 3): #If more than 3 second has passed since reaching within 0.5m of the goal
                    moving_towards_goal = False
                    inbound_counter = 0
                    rospy.loginfo("Reached goal point!")
                    continue
            else:
                inbound_counter = 0
            axes = []
            #Currently we are just publising a "dumb message" without any caps on the speed (since sending an x-axis movement of 500 and an
            #x-axis movement of 50 are both going to be capped at 5). We can adjust this using the distance received from 
            #get_distance_and_bearings_from_goal()
            axes.append(crow_fly_distance*math.sin(bearing)) #offset in the x direction for the shortest distance great circle arc
            axes.append(crow_fly_distance*math.cos(bearing)) #offset in the y direction for the shortest distance great circle arc
            axes.append(goal_height) #z height above initial position
            axes.append(yaw_difference)
            command = sensor_msgs.msg.Joy()
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
    altitude_difference = goal_height - current_height

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
    rospy.Subscriber("/dji_sdk/attitude",
                    geometry_msgs.msg.QuaternionStamped,
                    handle_attitude_data)
    waypoint_handler_server()
    joy_broadcaster()
    rospy.spin()

