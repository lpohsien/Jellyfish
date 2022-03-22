#!/usr/bin/env python

import jellyfish_movement.msg
import rospy
import actionlib
import geometry_msgs.msg
import dji_sdk.srv
import sensor_msgs.msg
import tf.transformations

initial_altitude = None

current_altitude = None
current_longitude = None
current_latitude = None

current_x = None
current_y = None
current_z = None
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
    global current_altitude
    global current_longitude
    global current_latitude
    global current_yaw
    global current_x
    global current_y
    global current_z

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

    #Get relevant initial coordinates
    try:
        initial_cartesian_coordinates = rospy.client.wait_for_message("/dji_sdk/local_position", geometry_msgs.msg.PointStamped, timeout=5)
        current_x = initial_cartesian_coordinates.point.x
        current_y = initial_cartesian_coordinates.point.y
        current_z = initial_cartesian_coordinates.point.z
    except rospy.ROSException:
        rospy.logerr("/dji_sdk/local_position is not being published... Check GPS health?")

    #Get relevant orientation data (basically just yaw)
    try:
        inititial_attitude = rospy.client.wait_for_message("/dji_sdk/attitude", geometry_msgs.msg.QuaternionStamped, timeout=5)
        inititial_attitude_quaternion = [inititial_attitude.quaternion.x, inititial_attitude.quaternion.y, inititial_attitude.quaternion.z, inititial_attitude.quaternion.w]
        current_yaw = tf.transformations.euler_from_quaternion(inititial_attitude_quaternion)[2] #Get yaw in radians
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
    set_local_pos_ref()
    get_initial_flight_informations()
    take_off()

def handle_current_cartesian_coordinates(msg):
    global current_x
    global current_y
    global current_z

    current_x = msg.point.x
    current_y = msg.point.y
    current_z = msg.point.z

def handle_attitude_data(msg):
    #Uses attitude data to get yaw of drone.
    global current_yaw

    current_orientation_quaternion = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
    current_yaw = tf.transformations.euler_from_quaternion(current_orientation_quaternion)[2] #Get yaw in radians

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
MoveToCartesianCoordinates action makes use of the /dji_sdk/flight_control_setpoint_ENUposition_yaw 
topic to control the drone's position. It takes in ENU cartesian coordinates and yaw, 
a pose stamped topic. The action will move the drone to the 
coordinates based on the pose stamped topic. Otherwise it will move the drone based on the
local position reference set by set_local_pos_ref
Setting a goal on this action will cancel the goal on the MoveToGPSCoordinates action
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
        self.yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        
    def execute_cb(self, goal):
        global to_publish

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
            self.yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
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
            x_diff = goal.x_goal - self.x
            y_diff = goal.y_goal - self.y
            z_diff = goal.z_goal - self.z
            z_pub = current_z + z_diff
            yaw_diff = goal.yaw_goal - self.yaw
            yaw_pub = current_yaw + yaw_diff
            axes = [x_diff, y_diff, z_pub, yaw_pub]
            to_publish = axes
            feedback.x_diff = x_diff
            feedback.y_diff = y_diff
            feedback.z_diff = z_diff
            self._as.publish_feedback(feedback)
            rospy.loginfo(f"xdiff: {abs(x_diff) < 1}, y_diff: {abs(y_diff) < 1}, z_diff: {abs(z_diff) < 1}, yaw_diff: {abs(yaw_diff) < (10/180*3.14159)}")
            if abs(x_diff) < 1 and abs(y_diff) < 1 and abs(z_diff) < 1 and abs(yaw_diff) < (10/180*3.14159):
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
        
    def execute_cb(self, goal):
        global to_publish

        update_rate = 5
        r = rospy.Rate(update_rate)
        success = 1
        inbound_counter = 0
        while True:
            if self._as.is_preempt_requested():
                self.sub.unregister()
                rospy.loginfo("prempted")
                success = 0
                self._result.success = 0
                self._as.set_preempted(self._result)
                break
            
            r.sleep()
        if success:
            self.sub.unregister()
            self._result.success = 1
            rospy.loginfo("reached!")
            self._as.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node("movement_handler")
    # Action servers
    move_to_cartesian_coordinates_server = move_to_cartesian_coordinates_action("cartesian_coordinates_handler_server")
    move_to_gps_coordinates_server = move_to_gps_coordinates_action("gps_coordinates_handler_server")
    initialise()
    # Subscribers
    rospy.Subscriber(
        "/dji_sdk/local_position",
        geometry_msgs.msg.PointStamped,
        handle_current_cartesian_coordinates)
    rospy.Subscriber(
        "/dji_sdk/attitude",
        geometry_msgs.msg.QuaternionStamped,
        handle_attitude_data)
    # Publishers
    joy_broadcaster()
    rospy.spin()