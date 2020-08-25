# coding=utf-8

import numpy as np
import rospy
# import robot_controller
# import time
from scipy.spatial.transform import Rotation
# from std_msgs.msg import Float32MultiArray
# from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import moveit_commander
import moveit_msgs.msg
import tf
import moveit_ros_planning_interface as moveit
import geometry_msgs.msg
from math import cos, sin


def euler_to_quaternion(roll, pitch, yaw):
    qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    return [qx, qy, qz, qw]

def quaternion_to_euler(quat):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

    return [roll, pitch, yaw]


def move2(pose_g, min_z, listener, group, planning_frame, eef_link,rot_z=0):
    step = - 0.05
    xy_step = 0.05

    #MoveIt
    try:
        trans, rot = listener.lookupTransform(planning_frame, eef_link, rospy.Time(0))

        current_z_rot = quaternion_to_euler(rot)[2]

        cos_z = cos(current_z_rot)
        sin_z = sin(current_z_rot)

        x = pose_g[1]/ 1000
        y = pose_g[0]/ 1000
        z = -pose_g[2] / 1000 + 0.15

        print "X "+str(x)
        print "Y "+str(y)

        if z < step:
            z = step

        if x < -xy_step:
            x = -xy_step
        elif x > xy_step:
            x = xy_step
        if y < -xy_step:
            y = -xy_step
        elif y > xy_step:
            y = xy_step

        x += 0.07
        y += 0.02
        # x = 0.0
        # y = 0.0

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = rot[3]
        pose_goal.orientation.x = rot[0]
        pose_goal.orientation.y = rot[1]
        pose_goal.orientation.z = rot[2]
        pose_goal.position.x = trans[0] + x * sin_z + y * cos_z
        pose_goal.position.y = trans[1] + y * sin_z + x * cos_z
        pose_goal.position.z = trans[2] + z

        if pose_goal.position.z < min_z:
            pose_goal.position.z = min_z

        group.set_pose_target(pose_goal)

        print pose_goal

        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        group.clear_pose_targets()
        rospy.sleep(1.0)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Couldn't find " + planning_frame + "->" + eef_link + " transform"

def move2joint(joints, group):
    group.go(joints, wait=True)
    group.stop()

def moveLookingDown(group):
    #joints = [3.10, -3.30, -0.82, 2.53, 0.77, -0.03]
    joints = [3.2724923474893677, -3.070034154258026, -1.0691886997717261, -0.4429645641561608, -0.8159414253073491, 2.9478611066184226]
    move2joint(joints, group)

def get_current_z(listener, planning_frame, eef_link):
    trans, rot = listener.lookupTransform(planning_frame, eef_link, rospy.Time(0))
    return trans[2]


    # spanko = rospy.Duration(2)
    #
    # print pose
    # rospy.sleep(spanko)
    # publisher.publish(pose)
    # rospy.sleep(spanko)

    # pose.pose.orientation.x = pose_g.data[3]
    # pose.pose.position.x = pose_g.data[2]/1000 - 0.2
    # pose.pose.position.y = 0.0
    # pose.pose.position.z = 0.0

    # print pose
    # rospy.sleep(spanko)
    # publisher.publish(pose)
    # rospy.sleep(spanko)


    # new_matrix = np.matmul(newish_matrix, orientation_ggcnn)

    # rot_x_limit = [[1, 0, 0, 0],
    #          [0, 0.707, 0.707, 0],
    #          [0, -0.707, 0.707, 0],
    #          [0, 0, 0, 1]]
    # inv_mat = np.linalg.inv(rot_x_limit)
    # limit = np.matmul(inv_mat, new_matrix)
    # if limit[2][3] > min_z:
    #     limit [2][3] = min_z
    #     new_matrix = np.matmul(rot_x_limit, limit)

    # should_do = np.matmul(inv_mat, akt_matrix)
    # if should_do[2][3] > min_z-0.15:
    #     return -1
    # mat_to_calc = [[new_matrix[0][0], new_matrix[0][1], new_matrix[0][2]],
    #                [new_matrix[1][0], new_matrix[1][1], new_matrix[1][2]],
    #                [new_matrix[2][0], new_matrix[2][1], new_matrix[2][2]]]
    #
    # fi = np.arccos((np.trace(mat_to_calc) - 1) / 2)
    #
    # ux = 1 / (2 * np.sin(fi)) * (mat_to_calc[2][1] - mat_to_calc[1][2])
    # uy = 1 / (2 * np.sin(fi)) * (mat_to_calc[0][2] - mat_to_calc[2][0])
    # uz = 1 / (2 * np.sin(fi)) * (mat_to_calc[1][0] - mat_to_calc[0][1])
    #
    # rx = ux * fi
    # ry = uy * fi
    # rz = uz * fi
    #
    # pose_goal = [new_matrix[0][3], new_matrix[1][3], new_matrix[2][3], rx, ry, rz]
    # pose_goal = np.transpose(pose_goal)
    # trajectory = list()
    # trajectory.append(pose_goal)
    # manipulator.move(trajectory, False, a=a, v=v)

    # akt_to_round = manipulator.get_pose()
    # pos_akt = np.round(akt_to_round, 3)
    # pos_cel = np.round(pose_goal, 3)
    # time_before = time.time()

    # while pos_akt[0] != pos_cel[0] or pos_akt[1] != pos_cel[1] or pos_akt[2] != pos_cel[2] or pos_akt[3] != pos_cel[3] or pos_akt[4] != pos_cel[4] or pos_akt[5] != pos_cel[5]:
    #     akt_to_round = manipulator.get_pose()
    #     pos_akt = np.round(akt_to_round, 3)
    #     time_now = time.time()
    #     passed = int(time_now - time_before )
    #     if passed > 0.2:
    #         return -1

#
# def get_pose(manipulator):
#     # manipulator = robot_controller.Ur3(ip, port_write, port_read)
#     pose = manipulator.get_pose()
#     return pose
#
# def get_z_orienation(manipulator):
#
#     pose = manipulator.get_pose()
#     matrix = [[1, 0, 0],
#               [0, 0.707, 0.707],
#               [0, -0.707, 0.707]]
#
#     inv_matrix = np.linalg.inv(matrix)
#
#     rotation = [pose[3], pose[4], pose[5]]
#     r = Rotation.from_rotvec(rotation)
#     matrix_from_axis = r.as_dcm()
#
#     tmp_matrix = [[matrix_from_axis[0][0], matrix_from_axis[0][1], matrix_from_axis[0][2]],
#                   [matrix_from_axis[1][0], matrix_from_axis[1][1], matrix_from_axis[1][2]],
#                   [matrix_from_axis[2][0], matrix_from_axis[2][1], matrix_from_axis[2][2]]]
#
#     tmp_matrix = np.matmul(tmp_matrix, inv_matrix)
#
#     new_r = Rotation.from_dcm(tmp_matrix)
#
#     euler = new_r.as_euler("XYZ", degrees=False)
#     z_orientation = euler[2]
#     return z_orientation
#
#
# def get_joints(manipulator):
#     joints = manipulator.get_joints()
#     return joints
#
#
# def set_box(manipulator):
#     print("Move arm to box position")
#     raw_input("Press enter...")
#     pose = manipulator.get_pose()
#     return pose
#
#
# def set_start(manipulator):
#     print("Move arm to start position")
#     raw_input("Press enter...")
#     pose = manipulator.get_pose()
#     manipulator.grip(150)
#     return pose
#
#
# def goto(pose, manipulator):
#     trajectory = list()
#     trajectory.append(pose)
#     manipulator.move(trajectory, False, a=0.1, v=0.8)
#
#     akt_to_round = manipulator.get_pose()
#     pos_akt = np.round(akt_to_round, 3)
#     pos_cel = np.round(pose, 3)
#     time_before = time.time()
#     while pos_akt[0] != pos_cel[0] or pos_akt[1] != pos_cel[1] or pos_akt[2] != pos_cel[2] or pos_akt[3] != pos_cel[3] or pos_akt[4] != pos_cel[4] or pos_akt[5] != pos_cel[5]:
#         akt_to_round = manipulator.get_pose()
#         pos_akt = np.round(akt_to_round, 3)
#         time_now = time.time()
#         passed = int(time_now - time_before )
#         if passed > 15:
#             return -1
#
#     manipulator.grip(150)
#     time.sleep(2)
#     return 0
#
#
# def set_table_z(manipulator):
#     manipulator.grip(0)
#     time.sleep(3)
#     print("Touch the table with arm")
#     raw_input("Press enter...")
#     akt_pose = get_pose(manipulator)
#     rot_x = [[1, 0, 0, 0],
#              [0, 0.707, 0.707, 0],
#              [0, -0.707, 0.707, 0],
#              [0, 0, 0, 1]]
#     inv_mat = np.linalg.inv(rot_x)
#     akt_pose = [[1, 0, 0, akt_pose[0]],
#                 [0, 1, 0, akt_pose[1]],
#                 [0, 0, 1, akt_pose[2]],
#                 [0, 0, 0, 1]]
#     calc_pose = np.matmul(inv_mat, akt_pose)
#     return calc_pose[2][3]
#
#
# def check_joints(manipulator):
#     joints = manipulator.get_joints()
#     if joints[5] > np.pi or joints[5] < -np.pi:
#         joints = np.delete(joints, 5)
#         joints = np.append(joints, 0)
#         trajectory = list()
#         trajectory.append(joints)
#         manipulator.move(trajectory, is_movej=True, is_pose=False, a=0.2, v=0.6)
#         pos_akt = manipulator.get_joints()
#         pos_akt = np.round(pos_akt, 2)
#         joints = np.round(joints, 2)
#         time_before = time.time()
#         while pos_akt[5] != joints[5]:
#             akt_to_round = manipulator.get_joints()
#             pos_akt = np.round(akt_to_round, 2)
#             time_now = time.time()
#             passed = int(time_now - time_before)
#             if passed > 10:
#                 return -1
