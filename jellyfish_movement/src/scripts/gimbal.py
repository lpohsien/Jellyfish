#!/usr/bin/env python

#TODO The gimbal position published is damn wonky, might need to take a look at the axes again and make sure that they tally correctly
#TODO Implement a way to check that the gimbal is not locked into a singularity (a way to count perhaps?)
#TODO Implement differing goal checking depending on the mode of the gimbal
#TODO Implement limits on command since the drone's gimbal cannot rotate pass a certain limit
#TODO Video and camera functions

import math
import rospy
import actionlib
import jellyfish_movement.msg as jellyfish_messages
import dji_sdk.msg as dji_messages
import geometry_msgs.msg as geometry_messages
import numpy as np
import tf_conversions
import tf2_ros

current_gimbal_position = None
current_gimbal_goal = None
to_publish = False
drone_name = "djeye_m210" #Default drone name, can be set through param servers
gimbal_transform = geometry_messages.TransformStamped()
gimbal_transform.transform.rotation.x = 0
gimbal_transform.transform.rotation.y = 0
gimbal_transform.transform.rotation.z = 0
gimbal_transform.transform.rotation.w = 1
gimbal_transform.transform.translation.x = 0.1
gimbal_transform.transform.translation.y = 0
gimbal_transform.transform.translation.z = 0.15

def initialise():
    global drone_name
    global current_gimbal_position
    global gimbal_transform
    try:
        drone_name = rospy.get_param("drone_name")
        gimbal_transform.header.frame_id = "{}_base_link".format(drone_name)
        gimbal_transform.child_frame_id = "{}_camera".format(drone_name)
    except rospy.ROSException:
        rospy.logerr("Drone name not received, defaulting to '{}'".format(drone_name))
    try:
        gimbal_transform.transform.translation.x = rospy.get_param("gimbal_x")
        gimbal_transform.transform.translation.y = rospy.get_param("gimbal_y")
        gimbal_transform.transform.translation.z = rospy.get_param("gimbal_z")
    except rospy.ROSException:
        rospy.logerr("Gimbal position not received, defaulting to ({}, {}, {})".format(gimbal_transform.transform.position.x, gimbal_transform.transform.position.y, gimbal_transform.transform.position.z))
    try:
        initial_gimbal_position = rospy.client.wait_for_message("/dji_sdk/gimbal_angle", geometry_messages.Vector3Stamped, timeout=5)
        current_gimbal_position = initial_gimbal_position.vector
        rospy.loginfo("Got gimbal position from dji_sdk")
    except rospy.ROSException:
        rospy.logerr("Failed to get gimbal position from dji_sdk")

class gimbal_angle_control_action():
    _feedback = jellyfish_messages.ControlGimbalFeedback()
    _result = jellyfish_messages.ControlGimbalResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, 
            jellyfish_messages.ControlGimbalAction, 
            execute_cb = self.execute_cb, 
            auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        global current_gimbal_goal
        global to_publish

        r = rospy.Rate(5)
        success = True
        #Start executing the goal
        rospy.loginfo("new gimbal goal received, moving...")
        new_gimbal_goal = dji_messages.Gimbal()
        mode = np.uint8(goal.mode)
        mode = mode | goal.ignore_yaw << 1
        mode = mode | goal.ignore_roll << 2
        mode = mode | goal.ignore_pitch << 3
        new_gimbal_goal.ts = np.int32(goal.time)
        new_gimbal_goal.mode = mode
        new_gimbal_goal.yaw = np.float32(math.radians(goal.yaw))
        new_gimbal_goal.roll = np.float32(math.radians(goal.roll))
        new_gimbal_goal.pitch = np.float32(math.radians(goal.pitch))
        current_gimbal_goal = new_gimbal_goal
        to_publish = True
        feedback = jellyfish_messages.ControlGimbalFeedback()
        #If incremental mode is used
        if goal.mode == 0:
            goal.yaw = current_gimbal_position.z + goal.yaw
            goal.roll = current_gimbal_position.x + goal.roll
            goal.pitch = current_gimbal_position.y + goal.pitch
        #Otherwise if absolute mode is used, goal yaw, roll and pitch would remain the same
        while not self.reached_gimbal_goal(goal):
            if self._as.is_preempt_requested():
                rospy.loginfo("preempted")
                success = False
                self._as.set_preempted(self._result)
                break
            feedback.yaw = current_gimbal_position.z
            feedback.roll = current_gimbal_position.x
            feedback.pitch = current_gimbal_position.y
            self._as.publish_feedback(feedback)
            r.sleep()
        
        if success:
            self._result.success = 1
            rospy.loginfo("succeeded")
            self._as.set_succeeded(self._result)
            
    def reached_gimbal_goal(self, goal):
        if not goal.ignore_yaw:
            if abs(goal.yaw - current_gimbal_position.z) > 2:
                return False
        if not goal.ignore_roll:
            if abs(goal.roll - current_gimbal_position.x) > 2:
                return False
        if not goal.ignore_pitch:
            if abs(goal.pitch - current_gimbal_position.y) > 2:
                return False
        return True

def handle_gimbal_angle_update(msg):
    global current_gimbal_position
    current_gimbal_position = msg.vector

def gimbal_command_publisher(pub, r):
    global to_publish

    if to_publish:
        pub.publish(current_gimbal_goal)
        to_publish = False
    r.sleep()

def gimbal_transform_publisher(tf_pub, r):
    global gimbal_transform

    quat = tf_conversions.transformations.quaternion_from_euler(current_gimbal_position.x, current_gimbal_position.y, current_gimbal_position.z)
    gimbal_transform.transform.rotation.x = quat[0]
    gimbal_transform.transform.rotation.y = quat[1]
    gimbal_transform.transform.rotation.z = quat[2]
    gimbal_transform.transform.rotation.w = quat[3]
    gimbal_transform.header.stamp = rospy.Time.now()
    tf_pub.sendTransform(gimbal_transform)
    r.sleep()

if __name__ == "__main__":
    rospy.init_node('gimbal_control')
    initialise()
    server = gimbal_angle_control_action(rospy.get_name())
    rospy.Subscriber("/dji_sdk/gimbal_angle",
                    geometry_messages.Vector3Stamped,
                    handle_gimbal_angle_update)

    #Publishers
    gimbal_angle_cmd_pub = rospy.Publisher("/dji_sdk/gimbal_angle_cmd", dji_messages.Gimbal, queue_size=5)
    gimbal_angle_cmd_pub_rate = rospy.Rate(5)
    gimbal_transform_pub = tf2_ros.TransformBroadcaster()
    gimbal_transform_pub_rate = rospy.Rate(10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        gimbal_command_publisher(gimbal_angle_cmd_pub, gimbal_angle_cmd_pub_rate)
        gimbal_transform_publisher(gimbal_transform_pub, gimbal_transform_pub_rate)
    rospy.spin()
        