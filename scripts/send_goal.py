#!/usr/bin/env python

import rospy
from move_base_msgs.msg import *
import actionlib
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import tf
import os

# client_0 = actionlib.SimpleActionClient('/robot0/move_base', MoveBaseAction)
# client_1 = actionlib.SimpleActionClient('/robot1/move_base', MoveBaseAction)

def send_goal(pose, i):
    client = actionlib.SimpleActionClient('/robot' + str(i) + '/move_base', MoveBaseAction)
    print("waiting for server")
    client.wait_for_server()
    print("Done!")
    target_pose = PoseStamped()
    pose_msg = Pose()
    pose_msg.position.x = pose[0]
    pose_msg.position.y = pose[1]
    q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
    pose_msg.orientation = Quaternion(*q)
    target_pose.pose = pose_msg
    goal = MoveBaseGoal()
    goal.target_pose = target_pose
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map_static'
    client.send_goal(goal)

def test():
    rospy.init_node('send_goal', anonymous=True)

    robot_goal_list = [[10, 7, 3.14], [1, 2, 3.14], [10, 7, 3.14]]
    robot_pos_list = [[1, 2], [6, 7], [20, 20]]
    for i in range(0, len(robot_pos_list)):
        pos = robot_pos_list[i]
        goal = robot_goal_list[i]
        # replace with some sort of automated roslaunch?
        os.system(
            "rosrun stdr_robot robot_handler add /home/masselmeier/catkin_ws/src/PotentialGap/stdr_robots/robots/holonomic_robot_360_bumper.xml " + str(pos[0]) + " " + str(pos[1]) + " 0")
        # give a laserscan callback to give a new goal
        send_goal(goal, i)

    n = len(robot_pos_list)

    rate = rospy.Rate(10)  # 10hz

    keep_waiting = True
    counter = 0
    result = None
    start_time = rospy.Time.now()
    client = actionlib.SimpleActionClient('/robot0/move_base', MoveBaseAction)
    while keep_waiting:
        try:
            state = client.get_state()
            if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
                keep_waiting = False
            #elif bumper_checker.collided:
            #    keep_waiting = False
            #    result = "BUMPER_COLLISION"
            elif rospy.Time.now() - start_time > rospy.Duration(600):
                keep_waiting = False
                result = "TIMED_OUT"
            else:
                counter += 1
                # if blocks, sim time cannot be 0
                rate.sleep()
        except:
            keep_waiting = "False"

    print(result)

if __name__=='__main__':
    test()

#rospy.Subscriber(ego_scan_topic, modify_ego_scan, queue_size=1, buff_size=2 ** 24)
