#!/usr/bin/env python

import jellyfish_movement.msg
import rospy
import actionlib
import geometry_msgs.msg
import dji_sdk.srv
import sensor_msgs.msg
import tf_conversions
import math
import numpy as np

initial_altitude = None

current_altitude = None
current_longitude = None
current_latitude = None

current_yaw = None

to_publish = None

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

def get_initial_flight_informations():
    global initial_altitude
    global current_altitude
    global current_longitude
    global current_latitude
    global current_yaw

    #Get relevant GPS data
    try:
        initial_GPS_coordinates = rospy.client.wait_for_message("/dji_sdk/gps_position", sensor_msgs.msg.NavSatFix, timeout=5)
        current_altitude = initial_GPS_coordinates.altitude
        current_latitude = initial_GPS_coordinates.latitude
        current_longitude = initial_GPS_coordinates.longitude
        initial_altitude = current_altitude
        rospy.loginfo("initial GPS coordinates received")
    except rospy.ServiceException:
        rospy.logerr("/dji_sdk/gps_position is not being published... Check GPS health?")

    #Get relevant orientation data (basically just yaw)
    try:
        inititial_attitude = rospy.client.wait_for_message("/dji_sdk/attitude", geometry_msgs.msg.QuaternionStamped, timeout=5)
        inititial_attitude_quaternion = [inititial_attitude.quaternion.x, inititial_attitude.quaternion.y, inititial_attitude.quaternion.z, inititial_attitude.quaternion.w]
        current_yaw = tf_conversions.transformations.euler_from_quaternion(inititial_attitude_quaternion)[2] #Get yaw in radians
        rospy.loginfo("initial orientation received")
    except rospy.ServiceException:
        rospy.logerr("/dji_sdk/attitude is not being published...")

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
    get_initial_flight_informations()
    take_off()

def handle_attitude_data(msg):
    #Uses attitude data to get yaw of drone.
    global current_yaw

    current_orientation_quaternion = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
    current_yaw = tf_conversions.transformations.euler_from_quaternion(current_orientation_quaternion)[2] #Get yaw in radians

def handle_gps_coordinates(msg):
    global current_altitude
    global current_longitude
    global current_latitude

    current_altitude = msg.altitude
    current_longitude = msg.longitude
    current_latitude = msg.latitude

def joy_broadcaster():
    #Broadcasts command to control drone on dji_sdk/flight_control_setpoint_ENUposition_yaw. Message is a sensor_msgs.msg.Joy message with
    #the following axes: [X position offset, Y position offset, Z position, yaw angle] in ENU ground frame
    global to_publish

    update_rate = 5
    pub = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUposition_yaw", sensor_msgs.msg.Joy, queue_size=1)
    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        if to_publish:
            command = sensor_msgs.msg.Joy()
            command.axes = to_publish
            pub.publish(command)
            to_publish = None
        r.sleep()

"""
Move_to_cartesian_coordinates_action makes use of the /dji_sdk/flight_control_setpoint_ENUposition_yaw 
topic to control the drone's position. It takes in ENU cartesian coordinates and yaw, 
a pose stamped topic. The action will move the drone to the coordinates based on the pose stamped topic. 

Setting a goal on this action will cancel the goal on the Move_to_gps_coordinates_action action
"""
class move_to_cartesian_coordinates_action():
    _feedback = jellyfish_movement.msg.MoveToCartesianCoordinatesFeedback()
    _result = jellyfish_movement.msg.MoveToCartesianCoordinatesResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            jellyfish_movement.msg.MoveToCartesianCoordinatesAction,
            execute_cb = self.execute_cb,
            auto_start = False)
        self._as.start()
        self.sub = None
        self.x = None
        self.y = None
        self.z = None
        self.yaw = None

    def update_position(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        quaternion = msg.pose.orientation
        quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        self.yaw = tf_conversions.transformations.euler_from_quaternion(quaternion)[2]
        rospy.loginfo(f"q = {quaternion}")
        
    def execute_cb(self, goal):
        global to_publish
        rospy.loginfo("Executing")

        # Cancels Move_to_gps_coordinates_action action
        client = actionlib.SimpleActionClient(
            "gps_coordinates_handler_server", 
            jellyfish_movement.msg.MoveToGPSCoordinatesAction)
        client.cancel_all_goals()
        update_rate = 5
        r = rospy.Rate(update_rate)
        success = 1
        try:
            current_position = rospy.wait_for_message(goal.pose_topic, geometry_msgs.msg.PoseStamped, 5)
            self.x = current_position.pose.position.x
            self.y = current_position.pose.position.y
            self.z = current_position.pose.position.z
            quaternion = current_position.pose.orientation
            quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            self.yaw = tf_conversions.transformations.euler_from_quaternion(quaternion)[2]
        except rospy.ROSException:
            rospy.logerr("Could not receive pose_stamped message from topic, or topic is not publishing")
            self._result.success = 0
            self._as.set_aborted(self._result)
            return
        self.sub =  rospy.Subscriber(
            goal.pose_topic, 
            geometry_msgs.msg.PoseStamped,
            self.update_position)
        feedback = jellyfish_movement.msg.MoveToCartesianCoordinatesFeedback()
        inbound_counter = 0
        while True:
            if self._as.is_preempt_requested():
                self.sub.unregister()
                rospy.loginfo("prempted")
                success = 0
                self._result.success = 0
                self._as.set_preempted(self._result)
                break
            x_diff_raw = goal.x_goal - self.x
            y_diff_raw = goal.y_goal - self.y
            z_diff_raw = goal.z_goal - self.z
            current_z = current_altitude - initial_altitude
            z_pub = current_z + z_diff_raw
            yaw_diff_raw = goal.yaw_goal - self.yaw # Relatice to the coordinate frame of the pose publisher. The yaw diff relative to the drone is hence the negative of this
            yaw_diff = yaw_diff_raw
            x_diff = math.cos(yaw_diff - current_yaw) * x_diff_raw + math.sin(yaw_diff - current_yaw) * y_diff_raw
            y_diff = -math.sin(yaw_diff - current_yaw) * x_diff_raw + math.cos(yaw_diff - current_yaw) * y_diff_raw
            yaw_pub = current_yaw - yaw_diff
            axes = [x_diff, y_diff, z_pub, yaw_pub] #yaw pub is kinda unstable and hence disabled. Replace 0 with yaw_pub for yaw control relative to pose being published
            to_publish = axes
            feedback.x_diff = x_diff
            feedback.y_diff = y_diff
            feedback.z_diff = z_diff_raw
            self._as.publish_feedback(feedback)
            rospy.loginfo(f"yaw= {self.yaw}")
            rospy.loginfo(f"xdiff: {x_diff}, y_diff: {y_diff}, z_diff: {abs(z_diff_raw) < 1}, yaw_diff: {yaw_diff}")
            if abs(x_diff) < 1 and abs(y_diff) < 1 and abs(z_diff_raw) < 1 and abs(yaw_diff) < (10/180*3.14159):
                inbound_counter += 1
                rospy.loginfo(f"inbound counter: {inbound_counter}")
                if inbound_counter > (update_rate * 3):
                    break
            else:
                inbound_counter = 0
                rospy.loginfo("out of bounds!")
            r.sleep()
        if success:
            self.sub.unregister()
            self._result.success = 1
            rospy.loginfo("reached!")
            self._as.set_succeeded(self._result)

"""
Move_to_gps_coordinates_action makes use of the /dji_sdk/flight_control_setpoint_ENUposition_yaw 
topic to control the drone's position. It takes in a goal latitude, longitude and height (relative
to initial height) 

The drone will first take to the goal height before flying to it

Setting a goal on this action will cancel the goal on the Move_to_cartesian_coordinates_action action
"""
class move_to_gps_coordinates_action():
    _feedback = jellyfish_movement.msg.MoveToGPSCoordinatesFeedback()
    _result = jellyfish_movement.msg.MoveToGPSCoordinatesResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            jellyfish_movement.msg.MoveToGPSCoordinatesAction,
            execute_cb = self.execute_cb,
            auto_start = False)
        self._as.start()
        self.goal_height = None
        self.goal_latitude = None
        self.goal_longitude = None

    def get_distance_and_bearings_from_goal(self):
        #Uses the harversine formula to get distance
        current_latitude_rad = current_latitude * math.pi /180
        goal_latitude_rad = self.goal_latitude * math.pi /180
        latitude_difference_rad = (self.goal_latitude - current_latitude) * math.pi /180
        longitude_difference_rad = (self.goal_longitude - current_longitude) * math.pi /180
        altitude_difference = self.goal_height - (current_altitude - initial_altitude)
        print(current_altitude)
        print(initial_altitude)

        a = ((math.sin(latitude_difference_rad/2)) ** 2) + math.cos(current_latitude_rad) * math.cos(goal_latitude_rad) * ((math.sin(longitude_difference_rad/2)) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        crow_fly_distance = 6371000 * c

        #Get bearings. 0 is North pi/2 is East
        y = math.sin(longitude_difference_rad) * math.cos(goal_latitude_rad)
        x = math.cos(current_latitude_rad) * math.sin(goal_latitude_rad) - math.sin(current_latitude_rad) * math.cos(goal_latitude_rad) * math.cos(longitude_difference_rad)
        bearing = math.atan2(y, x)

        return [crow_fly_distance, altitude_difference, bearing]
    
    def execute_cb(self, goal):
        global to_publish

        # Cancels Move_to_cartesian_coordinates_action action
        client = actionlib.SimpleActionClient(
            "cartesian_coordinates_handler_server", 
            jellyfish_movement.msg.MoveToCartesianCoordinatesAction)
        client.cancel_all_goals()
        update_rate = 5
        r = rospy.Rate(update_rate)
        self.goal_height = goal.height
        self.goal_latitude = goal.latitude
        self.goal_longitude = goal.longitude
        feedback = jellyfish_movement.msg.MoveToGPSCoordinatesFeedback()
        mission_state = 0
        success = 1
        inbound_counter = 0
        while True:
            if self._as.is_preempt_requested():
                rospy.loginfo("prempted")
                success = 0
                self._result.success = 0
                self._as.set_preempted(self._result)
                break
            crow_fly_distance, altitude_difference, bearing = self.get_distance_and_bearings_from_goal()
            x_diff = crow_fly_distance*math.sin(bearing)
            y_diff = crow_fly_distance*math.cos(bearing)
            # First fly to goal altitude and orientate the drone to face the goal gps coordinates
            if mission_state == 0:
                axes = [0, 0, goal.height, bearing]
                to_publish = axes
                if altitude_difference < 1 :
                    inbound_counter += 1
                    rospy.loginfo(f"inbound counter: {inbound_counter}")
                    if inbound_counter > (update_rate * 3):
                        rospy.loginfo("Altitude reached, flying towards goal gps coordinates...")
                        inbound_counter = 0
                        mission_state = 1
                else:
                    inbound_counter = 0
                    rospy.loginfo("out of bounds!")
            # Then fly to the goal gps coordinates
            elif mission_state == 1:
                axes = [x_diff, y_diff, goal.height, current_yaw]
                to_publish = axes
                if crow_fly_distance < 1 and altitude_difference < 1:
                    inbound_counter += 1
                    rospy.loginfo(f"inbound counter: {inbound_counter}")
                    if inbound_counter > (update_rate * 3):
                        break
                else:
                    inbound_counter = 0
                    rospy.loginfo("out of bounds!")
            feedback.x_diff = x_diff
            feedback.y_diff = y_diff
            feedback.z_diff = altitude_difference
            self._as.publish_feedback(feedback)
            rospy.loginfo(f"xdiff: {abs(x_diff) < 1}, y_diff: {abs(y_diff) < 1}, z_diff: {altitude_difference}")
            r.sleep()
        if success:
            rospy.loginfo("Reached destination!")
            self._as.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node("movement_handler")
    # Action servers
    move_to_cartesian_coordinates_server = move_to_cartesian_coordinates_action("cartesian_coordinates_handler_server")
    move_to_gps_coordinates_server = move_to_gps_coordinates_action("gps_coordinates_handler_server")
    initialise()
    # Subscribers
    rospy.Subscriber(
        "/dji_sdk/attitude",
        geometry_msgs.msg.QuaternionStamped,
        handle_attitude_data)
    rospy.Subscriber(
        "/dji_sdk/gps_position",
        sensor_msgs.msg.NavSatFix,
        handle_gps_coordinates)
    # Publishers
    joy_broadcaster()
    rospy.spin()