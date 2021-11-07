#!/usr/bin/env python

#TODO The gimbal position published is damn wonky, might need to take a look at the axes again and make sure that they tally correctly
#TODO Implement a way to check that the gimbal is not locked into a singularity (a way to count perhaps?)
#TODO Implement differing goal checking depending on the mode of the gimbal
#TODO Implement limits on command since the drone's gimbal cannot rotate pass a certain limit
#TODO Video and camera functions

import dji_sdk.srv
import math
import rospy
import actionlib
import jellyfish_movement.msg as jellyfish_messages
import dji_sdk.msg as dji_messages
import geometry_msgs.msg as geometry_messages
import numpy as np

current_gimbal_position = None
current_gimbal_goal = None
to_publish = False

def initialise():
    global current_gimbal_position
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
    initial_gimbal_position = rospy.client.wait_for_message("/dji_sdk/gimbal_angle", geometry_messages.Vector3Stamped)
    current_gimbal_position = initial_gimbal_position.vector

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

def gimbal_command_publisher():
    global to_publish

    r = rospy.Rate(5)
    pub = rospy.Publisher("/dji_sdk/gimbal_angle_cmd", dji_messages.Gimbal, queue_size=1)
    while not rospy.is_shutdown():
        if to_publish:
            pub.publish(current_gimbal_goal)
            to_publish = False
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('gimbal_control')
    initialise()
    server = gimbal_angle_control_action(rospy.get_name())
    rospy.Subscriber("/dji_sdk/gimbal_angle",
                    geometry_messages.Vector3Stamped,
                    handle_gimbal_angle_update)
    gimbal_command_publisher()
    rospy.spin()
        