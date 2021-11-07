#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import jellyfish_sim.msg
import numpy as np

landing_platform_pose = geometry_msgs.msg.Pose()
drone_pose = geometry_msgs.msg.Pose()
service_area = None
within_zone = False

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class service_zone:
    def __init__(self, vertices):
        vertices.sort(key = lambda vertice: vertice.x)
        above = []
        below = []
        for i in range(1, len(vertices) - 1):
            orientation = self.orientation(vertices[0], vertices[-1], vertices[i])
            if orientation == 1:
                above.append(vertices[i])
            else:
                below.append(vertices[i])
        above.sort(key = lambda vertice: vertice.x)
        below.sort(key = lambda vertice: vertice.x, reverse = True)
        self.edges = []
        current_vertice = vertices[0]
        for vertice in above:
            self.edges.append((current_vertice, vertice))
            current_vertice = vertice
        self.edges.append((current_vertice, vertices[-1]))
        current_vertice = vertices[-1]
        for vertice in below:
            self.edges.append((current_vertice, vertice))
            current_vertice = vertice
        self.edges.append((current_vertice, vertices[0]))

    def orientation(self, point1, point2, point3):
        #Uses the determinant of the triangle formed by the 3 points to get the orientation of point 3 
        #relative to the line segment formed by points 1 and 2
        det = (point2.x - point1.x) * (point3.y - point1.y) - (point3.x - point1.x) * (point2.y - point1.y)
        #Determinant would be 0 if the 3 points are colinear
        if det == 0:
            return 0
        #Determinant would be more than 0 if point 3 is anticlockwise
        elif det > 0:
            return 1
        #Determinant would be less than 0 if point 3 is clockwise
        else:
            return -1

    def intersect(self, point1, point2, point3, point4):
        #Determines if the line segments formed by points 1 and 2 intersect the line segments formed by points 3 and 4
        a = self.orientation(point1, point2, point3)
        b = self.orientation(point1, point2, point4)
        c = self.orientation(point3, point4, point1)
        d = self.orientation(point3, point4, point2)
        #If there are any 3 points that are colinear, project the line segments on the x axis and compare
        if a == 0:
            if max(point1.x, point2.x) >= point3.x >= min(point1.x, point2.x):
                return 2
        if b == 0:
            if max(point1.x, point2.x) >= point4.x >= min(point1.x, point2.x):
                return 2
        if c == 0:
            if max(point3.x, point4.x) >= point1.x >= min(point3.x, point4.x):
                return 2
        if d == 0:
            if max(point3.x, point4.x) >= point2.x >= min(point3.x, point4.x):
                return 2
        #Else if the orientations of points 3 and 4 relative to the line segment formd by points 1 and 2 are different and
        #the orientations of points 1 and 2 relative to the line segment formd by points 3 and 4 are different
        if a != b and c != d:
            return 1
        #Else there is no intersection!
        return 0

    def quaternion_mult(self, q, r):
        return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]

    def get_relative_position_of_drone_to_platform(self):
        #Gets the transformation from the platform to the drone, and rotates it  by the conjugate of the orientation 
        #quaternion of the landing platform
        relative_x = drone_pose.position.x - landing_platform_pose.position.x
        relative_y = drone_pose.position.y - landing_platform_pose.position.y 
        relative_z = drone_pose.position.z - landing_platform_pose.position.z
        r = [0, relative_x, relative_y, relative_z]
        q = [landing_platform_pose.orientation.w, landing_platform_pose.orientation.x, landing_platform_pose.orientation.y, landing_platform_pose.orientation.z]
        q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
        return self.quaternion_mult(self.quaternion_mult(q_conj,r),q)[1:]

    def within(self, coordinates):
        #This method assumes that the position of the drone is already transformed to the vector space relative to the landing platform
        #Projects a line from the point to (point.x + 100, point.y). 100 was chosen because that is in theory the max range of marvelmind beacons
        #If it intersects the polygon's edges twice, or if the point/ the projected point falls on the edge of the polygon, return True
        number_of_intersections = 0
        for edge in self.edges:
            intersect_val = self.intersect(edge[0], edge[1], coordinates, point(coordinates.x + 100, coordinates.y))
            if intersect_val == 1:
                number_of_intersections += 1
            elif intersect_val == 2:
                return True
        if number_of_intersections == 1:
            return True
        return False

def update_current_pose(msg):
    global landing_platform_pose
    
    landing_platform_pose = msg

def update_drone_pose(msg):
    global drone_pose

    drone_pose = msg

def setup_service_area():
    global service_area

    if rospy.has_param("service_vertices"):
        vertices = rospy.get_param("service_vertices")
        vertices_processed = []
        for vertices in vertices:
            vertices_processed.append(point(vertices[0], vertices[1]))
        service_area = service_zone(vertices_processed)
    else:
        rospy.logerr("Cannot find rosparam 'service_vertices' on parameter server. Might want to check that")

def marvelmind_publisher():
    update_rate = 12
    pub = rospy.Publisher("/hedge_pos", jellyfish_sim.msg.hedge_pos, queue_size = 1)
    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        #Project the point onto the plane of the landing platform and check if it falls within the service area
        relative_position = service_area.get_relative_position_of_drone_to_platform()
        projected_position = point(relative_position[0], relative_position[1])
        if service_area.within(projected_position):
            hedge_msg = jellyfish_sim.msg.hedge_pos()
            hedge_msg.timestamp_ms = round(rospy.get_time() * 1000)
            hedge_msg.x_m = relative_position[0]
            hedge_msg.y_m = relative_position[1]
            hedge_msg.z_m = relative_position[2]
            hedge_msg.flags = np.uint8(0)
            pub.publish(hedge_msg)
        
        r.sleep()


if __name__ == "__main__":
    rospy.init_node("fake_marvelmind")
    #Initialise service area
    setup_service_area()

    #Subscribers
    #Check if landing platform pose is being published. If so, subscribe
    try:
        rospy.client.wait_for_message("ground_truth/pose", geometry_msgs.msg.Pose, timeout=3)
    except:
        rospy.logerr("Cannot find platform's pose info. Is platform's odometry being published? Might want to check if odometry plugin is working")
    rospy.Subscriber("ground_truth/pose", geometry_msgs.msg.Pose, update_current_pose)
    #Check is drone pose is being published. If so, subscribe
    if rospy.has_param("drone_name"):
        drone_name = rospy.get_param("drone_name")
        try:
            rospy.client.wait_for_message(f"/{drone_name}/ground_truth/pose", geometry_msgs.msg.Pose, timeout=3)
        except:
            rospy.logerr(f"Cannot find drone's pose information. Currently looking for it in '/{drone_name}/ground_truth/pose'")
        rospy.Subscriber(f"/{drone_name}/ground_truth/pose", geometry_msgs.msg.Pose, update_drone_pose)
    else:
        rospy.logerr("Please set drone name. Cannot find drone")
    
    #Publishers
    marvelmind_publisher()