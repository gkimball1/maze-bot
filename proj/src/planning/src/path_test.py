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

    planner = PathPlanner("right_arm")
    limb = Limb("right")
    joint_angles = limb.joint_angles()
    print(joint_angles)
    camera_angle = {'right_j6': 1.72249609375, 'right_j5': 0.31765625, 'right_j4': -0.069416015625, 'right_j3': 1.1111962890625, 'right_j2': 0.0664150390625, 'right_j1': -1.357775390625, 'right_j0': -0.0880478515625}
    raw_input("Press <Enter> to move the right arm to take picture: ")
    limb.set_joint_positions( camera_angle)
    limb.move_to_joint_positions( camera_angle, timeout=15.0, threshold=0.008726646, test=None)
    drawing_angles = {'right_j6': -2.00561328125, 'right_j5': -1.9730205078125, 'right_j4': 1.5130146484375, 'right_j3': -1.0272568359375, 'right_j2': 1.24968359375, 'right_j1': -0.239498046875, 'right_j0': 0.4667275390625}
    #print(joint_angles)

    waypoints_abstract = vision()
    print(waypoints_abstract)
    
    #ar coordinate : 
    x = 0.461067  
    y = -0.235305
    #first get the solution of the maze
    # solutionpoints = [(0.439859 - x, 0.042909 - y), 
    #                     (0.583217 - x, 0.03325 - y),
    #                     (0.585826 - x, -0.166944 - y),
    #                     (0.688275 - x, -0.16808 - y),
    #                     (0.671065 - x, 0.353373 - y),
    #                     (0.786306 - x, 0.358615 - y),
    #                     (0.770597 - x, 0.405286 - y),
    #                     ]
    solutionpoints = [(0,0),(-0.66,0.16), (-0.7, 0.4)]
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
    for point in solutionpoints:
        point = np.array([point[0], point[1], 0, 1])
        point_space = np.dot(mat, point)
        point_spaces.append(point_space)

    print(point_spaces)

    raw_input("Press <Enter> to move the right arm to drawing position: ")
    limb.set_joint_positions( drawing_angles)
    limb.move_to_joint_positions( drawing_angles, timeout=15.0, threshold=0.008726646, test=None)

    
    

    

    # if ROBOT == "sawyer":
    #     Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    #     Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    #     Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    #     Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    # else:
    #     Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
    #     Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    #     Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    #     Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    # controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))
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
    #add a wall
    # size = [0.20, 0.09, 0.7]
    # name = "wall"
    # pose = PoseStamped()
    # pose.header.frame_id = "base"
    # pose.pose.position.x = 0.5
    # pose.pose.position.y = -0.6
    # pose.pose.position.z = -0.1

    # #Orientation as a quaternion
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0
    # planner.add_box_obstacle(size, name, pose)

    #Set up the right gripper
    

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;



    while not rospy.is_shutdown():
        #follow ar
        raw_input("Press <Enter> to move the right arm to start draw: ")
        while not rospy.is_shutdown():
            finish = False
            r = rospy.Rate(10)
            
            try:
                waypoint = []

                goal_ar = PoseStamped()
                goal_ar.header.frame_id = "base"

                #x, y, and z position
                goal_ar.pose.position.x = 0.45918716
                goal_ar.pose.position.y =  -0.4163326
                goal_ar.pose.position.z = 0

                #Orientation as a quaternion
                goal_ar.pose.orientation.x = 0
                goal_ar.pose.orientation.y = -1
                goal_ar.pose.orientation.z = 0
                goal_ar.pose.orientation.w = 0
                for point in point_spaces:
                    x = point[0]
                    y = point[1]
                    print([x,y])
                    goal_ar.pose.position.x = x
                    goal_ar.pose.position.y = y
                    
                    waypoint.append(copy.deepcopy(goal_ar.pose))
                
                

                plan, fraction = planner.plan_straight(waypoint, [])
                print(fraction)
                

                raw_input("Press <Enter> to move the right arm to next goal: ")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
            r.sleep()
        
        # while not rospy.is_shutdown():
        #     try:
        #         goal_3 = PoseStamped()
        #         goal_3.header.frame_id = "base"

        #         #x, y, and z position
        #         goal_3.pose.position.x = 0.66
        #         goal_3.pose.position.y = 0.26
        #         goal_3.pose.position.z = 0.01

        #         #Orientation as a quaternion
        #         goal_3.pose.orientation.x = 0.0
        #         goal_3.pose.orientation.y = -1.0
        #         goal_3.pose.orientation.z = 0.0
        #         goal_3.pose.orientation.w = 0.0
        #         point = [goal_3.pose]

        #         plan, fraction = planner.plan_straight(point, [orien_const])
        #         print(fraction)
                

        #         raw_input("Press <Enter> to move the right arm to start position: ")
        #         if not planner.execute_plan(plan):
        #             raise Exception("Execution failed")
        #         print("done")
        #     except Exception as e:
        #         print e
        #     else:
        #         break

        # while not rospy.is_shutdown():
        #     try:
        #         goal_2 = PoseStamped()
        #         goal_2.header.frame_id = "base"
                

        #         #x, y, and z position
        #         goal_2.pose.position.x = 0.61425
        #         goal_2.pose.position.y = 0.0304
        #         goal_2.pose.position.z = 0.14

        #         #Orientation as a quaternion
        #         goal_2.pose.orientation.x = 0.0
        #         goal_2.pose.orientation.y = -1.0
        #         goal_2.pose.orientation.z = 0.0
        #         goal_2.pose.orientation.w = 0.0



        #         plan = planner.plan_to_pose(goal_2, [])

                

        #         raw_input("Press <Enter> to move the right arm to goal pose 2: ")
        #         if not planner.execute_plan(plan):
        #             raise Exception("Execution failed")
        #     except Exception as e:
        #         print e
        #     else:
        #         break
        #limb.set_joint_positions( drawing_angles)
        #limb.move_to_joint_positions( drawing_angles, timeout=15.0, threshold=0.008726646, test=None)
        """set x,y,z be ar_marker"""
        x, y, z = 0.77,0.41, -0.14
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

        #keyboard
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

        # while not rospy.is_shutdown():
        #     try:
                
                
        #         waypoint = []
        #         waypoint.append(copy.deepcopy(goal_1.pose))

        #         goal_1.pose.position.x = 0.770597
        #         goal_1.pose.position.y = 0.405286
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.786306
        #         goal_1.pose.position.y = 0.358615
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.671065
        #         goal_1.pose.position.y = 0.353373
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.688275
        #         goal_1.pose.position.y = -0.16808
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.585826
        #         goal_1.pose.position.y = -0.166944
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.583217
        #         goal_1.pose.position.y = 0.03325
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.439859
        #         goal_1.pose.position.y = 0.042909
        #         waypoint.append(copy.deepcopy(goal_1.pose))
        #         goal_1.pose.position.x = 0.461067
        #         goal_1.pose.position.y = -0.235305
        #         waypoint.append(copy.deepcopy(goal_1.pose))
                
                
        #         # for i in range(7):
        #         #     goal_1.pose.position.y -= 0.1
        #         #     waypoint.append(copy.deepcopy(goal_1.pose))

        #         # for i in range(3):
        #         #     goal_1.pose.position.x -= 0.1
        #         #     waypoint.append(copy.deepcopy(goal_1.pose))
        #         # for i in range(7):
                    
        #         #     goal_1.pose.position.y += 0.1
                    

        #         #     waypoint.append(copy.deepcopy(goal_1.pose))
        #         # for i in range(3):
        #         #     goal_1.pose.position.z += 0.1
        #         #     waypoint.append(copy.deepcopy(goal_1.pose))
        #         # for i in range(5):
        #         #     goal_1.pose.position.x += 0.1
        #         #     goal_1.pose.position.y = y
        #         #     goal_1.pose.position.z = z

        #         #     waypoint.append(copy.deepcopy(goal_1.pose))



        #         # Might have to edit this . . . 
        #         plan, fraction = planner.plan_straight(waypoint, [orien_const])
        #         print(fraction)
                

        #         raw_input("Press <Enter> to move the right arm to draw: ")
        #         if not planner.execute_plan(plan):
        #             raise Exception("Execution failed")
        #     except Exception as e:
        #         print e
        #         traceback.print_exc()
        #     else:
        #         break

        

        

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
