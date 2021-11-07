#!/usr/bin/env python

"""
The ros service set_flight_state sets the flight state of the vehicle. The mission and standby states can be utilised when the drone is out
performing its drone stuff. Setting the flight state to follow would lead it to fly towards the asv and follow the asv's landing pad, ideally
within the landing zone.

The land state can only be requested when the drone has view of the landing pad. Requesting it otherwise would set the flight state to follow.
Requesting for the land state will cause the drone to attempt to make a landing. Failing to do so will revert it to the following state.

The take off state can only be requested in the land state

Requesting for the emergency home state will cause the drone to drop everything and return to a pre-set home location

Jellyfish flight states
0   Mission
1   Standby
2   Follow
3   Land
4   take off
5   Emergency home
"""

import rospy
import sensor_msgs.msg 

from jellyfish.srv import set_flight_state

emergency_home_latitude = 22.5428
emergency_home_longitude = 113.1598


def handle_set_flight_state(req):
    rospy.loginfo(f"flight state set to {req}")
    return (f"flight state set to {req}")

def DTLS_server():
    rospy.init_node("DTLS_server")
    DTLS_service = rospy.Service("set_flight_state", set_flight_state, handle_set_flight_state)
    rospy.loginfo("DTLS Server ready")
    rospy.spin()

if __name__ == "__main__":
    DTLS_server()
