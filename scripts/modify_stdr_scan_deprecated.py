#!/usr/bin/env python
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PoseStamped, Quaternion
from move_base_msgs.msg import *
import actionlib
from actionlib_msgs.msg import GoalStatus

import rospy
import subprocess
import os
import time
import operator

modified_scan_pub = rospy.Publisher('/robot4/mod_laser0', LaserScan, queue_size=1)
goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

'''
commands
roslaunch stdr_launchers server_with_map.launch
roslaunch stdr_gui stdr_gui.launch
'''
class StdrScanModifier:
    def __init__(self, n):
        self.n = n  # number of obstacles
        self.other_scans_id_dict = {}
        self.robot_radius = 0.2
        self.x_obs = 0.0
        self.y_obs = 0.0
        self.theta_obs = 0.0
        self.pt1 = np.array([self.x_obs, self.y_obs], dtype=float)

    def own_odom_callback(self, odom_msg):
        # print('odom msg: ', odom_msg)
        position = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        # returns value between -pi and pi
        orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # print('position: ', position)
        # print('orientation: ', orientation)
        #if self.first_call:
        #    # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
        #    self.first_call = False
        #    self.theta_0 = orientation
        #    # theta = theta_0
        #    Mrot = np.matrix([[np.cos(self.theta_0), np.sin(self.theta_0)], [-np.sin(self.theta_0), np.cos(self.theta_0)]])
        #    self.x_0 = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y
        #    self.y_0 = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y

        # Mrot = np.matrix([[np.cos(self.theta_0), np.sin(self.theta_0)], [-np.sin(self.theta_0), np.cos(self.theta_0)]])

        # We subtract the initial values
        self.x_obs = position.x  # - self.x_0
        self.y_obs = position.y  # - self.y_0
        self.theta_obs = orientation  # - self.theta_0
        # print('X OBS: ', self.x_obs)
        # print('Y OBS: ', self.y_obs)
        # print('THETA OBS: ', self.theta_obs)

        self.pt1 = np.array([self.x_obs, self.y_obs], dtype=float)
        marker_pub = rospy.Publisher('robot' + str(self.n) + '/pos_marker', Marker, queue_size=5)
        self.publish_marker(marker_pub, odom_msg, color=[0.0, 0.0, 1.0])

    def publish_marker(self, publisher, odom_msg, color):
        marker = Marker()
        marker.header = odom_msg.header
        marker.ns = "robot_position_markers"
        marker.id = 0

        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.pose.position.x = odom_msg.pose.pose.position.x
        marker.pose.position.y = odom_msg.pose.pose.position.y
        marker.pose.position.z = odom_msg.pose.pose.position.z
        marker.lifetime = rospy.Duration()
        marker.scale.x = 2 * self.robot_radius
        marker.scale.y = 2 * self.robot_radius
        marker.scale.z = 0.000001
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        publisher.publish(marker)

    def update_other_robot_odoms(self, odom_msg):
        # print('topic for other odom: ', odom_msg.child_frame_id + '/odom')
        self.other_scans_id_dict[odom_msg.child_frame_id + '/odom'] = np.array([odom_msg.pose.pose.position.x,
                                                                                odom_msg.pose.pose.position.y], dtype=float)
        marker_pub = rospy.Publisher(odom_msg.child_frame_id + '/pos_marker', Marker, queue_size=5)
        self.publish_marker(marker_pub, odom_msg, color=[1.0, 0.0, 0.0])

    def sgn_star(self, dy):
        if dy < 0:
            return -1
        else:
            return 1

    def modify_ego_scan(self, scan_msg):
        modified_laser_scan = LaserScan()
        modified_laser_scan.header = scan_msg.header
        # print('modified scan header:', modified_laser_scan.header)
        modified_laser_scan.angle_min = scan_msg.angle_min
        modified_laser_scan.angle_max = scan_msg.angle_max
        modified_laser_scan.angle_increment = scan_msg.angle_increment


        new_ranges = np.asarray(scan_msg.ranges)
        #print('new_ranges: ', new_ranges)
        #print('new_ranges: ', len(scan_msg.ranges))
        # scan goes from angle_min to angle_max counter-clockwise
        for i in range(0, len(new_ranges)):
            rad = scan_msg.angle_min + i*scan_msg.angle_increment
            #print('rad: ', rad)
            # check distance of scan, if distance is infinity, just set as max range distance
            if new_ranges[i] == np.inf:
                dist = scan_msg.range_max
            else:
                dist = new_ranges[i]
            # print('dist: ', dist)
            #print('pt1: ', self.pt1)
            pt2 = self.pt1 + np.array([dist*np.cos(rad + self.theta_obs), dist*np.sin(rad + self.theta_obs)], dtype=float)
            #print('line segment: (' + str(self.pt1) + ', ' + str(pt2) + ')')
            #  running line-circle intersection check from here:
            # https://mathworld.wolfram.com/Circle-LineIntersection.html
            dist_sorted_dict = dict(sorted(self.other_scans_id_dict.items(), key= lambda val: (val[1][0] - self.x_obs)**2 + (val[1][1] - self.y_obs)**2, reverse=True))
            # print('sorted dict: ', dist_sorted_dict)
            for key in dist_sorted_dict.keys():
                other_robot_state = self.other_scans_id_dict[key]
                # print('other robot state: ', other_robot_state)
                centered_pt1 = self.pt1 - other_robot_state
                if np.linalg.norm(centered_pt1) > new_ranges[i] or np.linalg.norm(centered_pt1) < scan_msg.range_min:
                    continue
                centered_pt2 = pt2 - other_robot_state
                # print('centered_pt1: ', centered_pt1)
                # print('centered_pt2: ', centered_pt2)
                dx = centered_pt2[0] - centered_pt1[0]
                dy = centered_pt2[1] - centered_pt1[1]
                dr = np.linalg.norm([dx, dy])
                D = centered_pt1[0]*centered_pt2[1] - centered_pt2[0]*centered_pt1[1]
                discriminant = self.robot_radius**2 * dr**2 - D**2
                # print('discriminant: ', discriminant)

                # need to check if intersection points are between two pts, algo treats as infinite line
                if discriminant == 0:
                    #print('one collision at ' + str(rad))
                    # this means that there is one collision
                    modified_laser_pt = [D * dy / dr**2, -D * dx / dr**2]
                    #print('modified laser pt: ', modified_laser_pt)
                    modified_dist = np.linalg.norm(modified_laser_pt - self.pt1)
                    new_ranges[i] = modified_dist
                    #print('dist: ', dist)
                    #print('modified dist: ', modified_dist)
                elif discriminant > 0:
                    # print('two collisions at ' + str(rad))
                    # this means that there is two collisions
                    intersection0 = np.array([(D * dy + self.sgn_star(dy) * dx * np.sqrt(discriminant)) / dr**2,
                                     (-D * dx + np.abs(dy) * np.sqrt(discriminant)) / dr**2], dtype=float)
                    intersection1 = np.array([(D * dy - self.sgn_star(dy) * dx * np.sqrt(discriminant)) / dr**2,
                                     (-D * dx - np.abs(dy) * np.sqrt(discriminant)) / dr**2], dtype=float)
                    # check to see which collision is closer
                    dist0 = np.linalg.norm(intersection0 - centered_pt1)
                    dist1 = np.linalg.norm(intersection1 - centered_pt1)
                    #print('dist0: ', dist0)
                    #print('dist1: ', dist1)
                    # print('new_ranges[i]: ', new_ranges[i])
                    if dist0 < dist1:
                        # print('intersection0: ', intersection0)
                        if dist0 < new_ranges[i] and dist0 < np.linalg.norm(centered_pt2 - centered_pt1) and (
                                                     np.linalg.norm(intersection0 - centered_pt2) < np.linalg.norm(centered_pt2 - centered_pt1)):
                            new_ranges[i] = dist0
                            break
                    else:
                        # print('intersection1: ', intersection1)
                        if dist1 < new_ranges[i] and np.linalg.norm(intersection1 - centered_pt1) < np.linalg.norm(centered_pt2 - centered_pt1) and (
                                                     np.linalg.norm(intersection1 - centered_pt2) < np.linalg.norm(centered_pt2 - centered_pt1)):
                            new_ranges[i] = dist1
                            break
                    # print('dist: ', dist)
                    # print('modified dist: ', new_ranges[i])

        modified_laser_scan.ranges = new_ranges
        modified_laser_scan.range_min = np.amin(new_ranges)
        modified_laser_scan.range_max = np.amax(new_ranges)
        modified_scan_pub.publish(modified_laser_scan)


def test():
    rospy.init_node('modified_stdr_scan', anonymous=True)
    ego_robot_pos = [8, 7, 1.57]
    robot_pos_list = [[10, 7]] # , [8, 6], [10, 10], [6, 7]]
    for pos in robot_pos_list:
        os.system(
            "rosrun stdr_robot robot_handler add /home/masselmeier/catkin_ws/src/PotentialGap/stdr_robots/robots/holonomic_robot_360_bumper.xml " + str(pos[0]) + " " + str(pos[1]) + " 0")

    os.system(
        "rosrun stdr_robot robot_handler add /home/masselmeier/catkin_ws/src/PotentialGap/stdr_robots/robots/holonomic_robot_360_bumper.xml " + str(
            ego_robot_pos[0]) + " " + str(ego_robot_pos[1]) + " 1.57")
    # note: seems that you can only hijack the last robot spawned?
    n = len(robot_pos_list)
    scan_modifier = StdrScanModifier(n)
    for i in range(0, n):
        topic = '/robot' + str(i) + '/odom'
        # print('topic: ', topic)
        rospy.Subscriber(topic, Odometry, scan_modifier.update_other_robot_odoms, queue_size=1, buff_size=2 ** 24)
        my_command = "rosrun stdr_samples stdr_obstacle_avoidance robot" + str(i) + " laser_0"
        subprocess.Popen(my_command.split())

    # TODO: can we use move_base with multiple robots? I think so

    rospy.Subscriber('/robot' + str(n) + '/laser_0', LaserScan, scan_modifier.modify_ego_scan, queue_size=1, buff_size=2 ** 24)

    rospy.Subscriber('robot'  + str(n) + '/odom', Odometry, scan_modifier.own_odom_callback, queue_size=1, buff_size=2 ** 24)

    '''
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) # potential_gap_egocircle_path_follower
    print("waiting for server")
    # client.wait_for_server()
    print("Done!")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/robot0"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    print("sending goal")
    client.send_goal(goal)
    print("waiting for result")
    '''


    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__=='__main__':
    test()

#rospy.Subscriber(ego_scan_topic, modify_ego_scan, queue_size=1, buff_size=2 ** 24)
