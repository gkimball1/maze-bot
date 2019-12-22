#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb
import copy
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
import tf2_ros
from vision import vision
from pyquaternion import Quaternion


"""
modified from lab 7 file 
"""

def as_transformation_matrix(trans):
    """Convert geomety_msgs.msg.TransformStamped to Transformation matrix.
    Args:
        trans (geomety_msgs.msg.TransformStamped): transform to be converted.
    Returns:
        numpy.array: transformation matrix generated.
    """
    mat = Quaternion(
        trans.rotation.w,
        trans.rotation.x,
        trans.rotation.y,
        trans.rotation.z)
    mat = mat.transformation_matrix
    mat[0][3] = trans.translation.x
    mat[1][3] = trans.translation.y
    mat[2][3] = trans.translation.z
    return mat


def main():
    """
    Main Script


    """
    while not rospy.is_shutdown():

        planner = PathPlanner("right_arm")
        limb = Limb("right")
        joint_angles = limb.joint_angles()
        print(joint_angles)
        camera_angle = {'right_j6': 1.72249609375, 'right_j5': 0.31765625, 'right_j4': -0.069416015625, 'right_j3': 1.1111962890625, 'right_j2': 0.0664150390625, 'right_j1': -1.357775390625, 'right_j0': -0.0880478515625}
        limb.set_joint_positions( camera_angle)
        limb.move_to_joint_positions( camera_angle, timeout=15.0, threshold=0.008726646, test=None)
        #drawing_angles = {'right_j6': -2.00561328125, 'right_j5': -1.9730205078125, 'right_j4': 1.5130146484375, 'right_j3': -1.0272568359375, 'right_j2': 1.24968359375, 'right_j1': -0.239498046875, 'right_j0': 0.4667275390625}
        #print(joint_angles)
        #drawing_angles = {'right_j6': -1.0133310546875, 'right_j5': -1.5432158203125, 'right_j4': 1.4599892578125, 'right_j3': -0.04361328125, 'right_j2': 1.6773486328125, 'right_j1': 0.019876953125, 'right_j0': 0.4214736328125}
        drawing_angles = {'right_j6': 1.9668515625, 'right_j5': 1.07505859375, 'right_j4': -0.2511611328125, 'right_j3': 0.782650390625, 'right_j2': 0.25496875, 'right_j1': -0.3268349609375, 'right_j0': 0.201146484375}

        raw_input("Press <Enter> to take picture: ")
        waypoints_abstract = vision()
        print(waypoints_abstract)
        
        #ar coordinate : 
        x = 0.461067  
        y = -0.235305
        #first get the solution of the maze
        
        #solutionpoints = [(0,0),(-0.66,0.16), (-0.7, 0.4)]
        # Make sure that you've looked at and understand path_planner.py before starting
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        r = rospy.Rate(10)

        #find trans from
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('base','ar_marker_0', rospy.Time()).transform
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if tf2_ros.LookupException:
                    print("lookup")
                elif tf2_ros.ConnectivityException:
                    print("connect")
                elif tf2_ros.ExtrapolationException:
                    print("extra")
                # print("not found")
                pass
            r.sleep()

        mat = as_transformation_matrix(trans)

        point_spaces = []
        for point in waypoints_abstract:
        # for point in solutionpoints:
            point = np.array([point[0], point[1], 0, 1])
            point_space = np.dot(mat, point)
            point_spaces.append(point_space)

        print(point_spaces)
        length_of_points = len(point_spaces)

        
        raw_input("Press <Enter> to move the right arm to drawing position: ")
        limb.set_joint_positions( drawing_angles)
        limb.move_to_joint_positions( drawing_angles, timeout=15.0, threshold=0.008726646, test=None)


        ##
        ## Add the obstacle to the planning scene here
        ##
        #add obstacle
        size = [0.78, 1.0, 0.05]
        name = "box"
        pose = PoseStamped()
        pose.header.frame_id = "base"
        pose.pose.position.x = 0.77
        pose.pose.position.y = 0.0
        pose.pose.position.z = -0.18

        #Orientation as a quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        planner.add_box_obstacle(size, name, pose)
    

    
        #limb.set_joint_positions( drawing_angles)
        #limb.move_to_joint_positions( drawing_angles, timeout=15.0, threshold=0.008726646, test=None)
        
        #starting position
        x, y, z = 0.67,0.31, -0.107343
        goal_1 = PoseStamped()
        goal_1.header.frame_id = "base"

        #x, y, and z position
        goal_1.pose.position.x = x
        goal_1.pose.position.y = y
        goal_1.pose.position.z = z

        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0


        while not rospy.is_shutdown():
            try:
                waypoint = []
                for point in point_spaces:
                    
                    goal_1.pose.position.x = point[0]
                    goal_1.pose.position.y = point[1]
                    goal_1.pose.position.z = -0.113343  #set this value when put different marker
                    waypoint.append(copy.deepcopy(goal_1.pose))

                plan, fraction = planner.plan_straight(waypoint, [])
                print(fraction)

                raw_input("Press <Enter> to move the right arm to draw: ")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

        raw_input("Press <Enter> to start again: ")

        #keyboard, used for testing drawing workspace
        # while not rospy.is_shutdown():
        #     finish = False
        #     while not finish:

        #         try:
        #             waypoint = []
        #             print("enter direction")
        #             direction = raw_input()
        #             if direction == 'w':
        #                 goal_1.pose.position.x -= 0.1
        #                 waypoint = [goal_1.pose]
        #             elif direction == 's':
        #                 goal_1.pose.position.x += 0.1
        #                 waypoint = [goal_1.pose]
        #             elif direction == 'a':
        #                 goal_1.pose.position.y -= 0.1
        #                 waypoint = [goal_1.pose]
        #             elif direction == 'd':
        #                 goal_1.pose.position.y += 0.1
        #                 waypoint = [goal_1.pose]
        #             elif direction == 'q':
        #                 goal_1.pose.position.z -= 0.1
        #                 waypoint = [goal_1.pose]
        #             elif direction == 'e':
        #                 goal_1.pose.position.z += 0.1
        #                 waypoint = [goal_1.pose]
        #             else:
        #                 finish = True
        #                 waypoint = [goal_1.pose]
                    

        #             plan, fraction = planner.plan_straight(waypoint, [])
        #             print(fraction)
                    

        #             # raw_input("Press <Enter> to move the right arm to goal pose 1: ")
        #             if not planner.execute_plan(plan):
        #                 raise Exception("Execution failed")
        #         except Exception as e:
        #             print e
        #             traceback.print_exc()
        #         else:
        #             break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
