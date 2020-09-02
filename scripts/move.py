#! /usr/bin/env python

import rospy
import tools
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
import tf
from rg6_service.srv import RG6_grip, RG6_gripResponse
from std_msgs.msg import Float64
from moveit_msgs.msg import ExecuteTrajectoryActionResult
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest
import sys
import copy
import moveit_commander
import tf
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list




def starter():
    global MIN_Z
    global time_last
    global group
    global last_z
    global mistakes
    global first

    mistakes = 0
    last_z = 1000
    first = True

    tools.moveLookingDown(group)
    tools.grip(gripper, run_program, 160)
    time_last = time.time()

    return True


def move(pose_g):
    global start
    global done
    global once
    global last_z
    global time_last, time_now
    global listener
    global min_z
    global group
    global planning_frame, eef_link


    time_now = time.time()
    if done:
        if not (pose_g[0] == 0 or pose_g[1] == 0 or (pose_g[2] > last_z+30 and
                                                               tools.get_current_z(listener, planning_frame, eef_link)
                                                               < min_z + 0.2)):
            print('Publishing new pose')
            if tools.move2(pose_g, min_z, listener, group, planning_frame, eef_link, gripper, run_program):
                print("Grasping...")
                tools.grip(gripper, run_program, 0)
                rospy.sleep(3)
                done = starter()
            else:
                time_last = time.time()
                last_z = pose_g[2]
        else:
            print pose_g[0] == 0
            print pose_g[1] == 0
            print pose_g[2] > last_z
            print pose_g[2], last_z

            print("TIME: "+str(int(time_now-time_last)))
            if int(time_now-time_last) >= 15:
                try:
                    done = False
                    print("Quiting")
                    # tools.grip(gripper, run_program, 0)
                    # rospy.sleep(3)
                    done = starter()
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

last_z = 1000
mistakes = 0
ok = False
start = False
first = True
done = False
time_last = time.time()
time_now = time.time()

rospy.init_node('move')

pub = rospy.Publisher('/controller_ur/move_to_pose', PoseStamped, queue_size=1)
pub_image = rospy.Publisher('/camera/color/image_raw_GG', Image, queue_size=1)
pub_depth = rospy.Publisher('object_detection/depth_GG', Image, queue_size=1)
pub_order = rospy.Publisher('/controller_ur/order', String, queue_size=1)
#pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)

print("Waiting for gripper service...")
rospy.wait_for_service('/rg2_gripper/control_width')
gripper = rospy.ServiceProxy('/rg2_gripper/control_width', RG6_grip)
run_program = rospy.ServiceProxy('/arm_controller/ur_hardware_interface/dashboard/play', Trigger)
print("Got it\n")

MIN_Z = 1.08446212623
# MIN_Z = None

if MIN_Z == 0 or MIN_Z is None:
    tools.grip(gripper, run_program, 0)

# MoveIt init
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)

# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#                                                moveit_msgs.msg.DisplayTrajectory,
#                                                queue_size=20)

planning_frame = group.get_planning_frame()
eef_link = group.get_end_effector_link()

listener = tf.TransformListener()
listener.waitForTransform(planning_frame, eef_link, rospy.Time(0), rospy.Duration(1))

# print("Moving to 'moveLookingDown'")
# tools.moveLookingDown(group)
# rospy.sleep(1)

if MIN_Z == 0 or MIN_Z is None:
    print("Move arm to touch the table")
    raw_input("Press enter...")

    listener.waitForTransform(planning_frame, eef_link, rospy.Time(0), rospy.Duration(1))
    trans, _ = listener.lookupTransform(planning_frame, eef_link, rospy.Time(0))
    min_z = trans[2]
else:
    min_z = MIN_Z

print("Table Z is at " + str(min_z) +"m")


# min_z = input('Enter minimum z value (default for "coffee_table" is 1.2)')
if (not start) and (not done):
    done = starter()

while not rospy.is_shutdown():
    rate = rospy.Rate(2)
    poses = []
    cur_z = tools.get_current_z(listener, planning_frame, eef_link)
    if cur_z - min_z < 0.2:
        rang = range(15)
        cut = 8
    else:
        rang = range(8)
        cut = 3


    for i in rang:
        try:
            print "Iteration nr " + str(i)
            image = rospy.wait_for_message('/camera/color/image_raw', Image, rospy.Duration(2))

            pub_image.publish(image)

            depth = rospy.wait_for_message('object_detection/depth', Image, rospy.Duration(2))

            pub_depth.publish(depth)

            pose_gg = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray, rospy.Duration(2))
            if i > cut:
                poses.append(pose_gg)
            mistakes = 0
        except:
            poses = None
            mistakes += 1
            print "Exception nr " + str(mistakes)
            break
    pose_gg = []
    maxes = []
    minis = []


    if not poses is None:
        for gg_index in range(5):
            value = 0
            maxi = 0
            mini = 100000

            for poses_index in range(len(poses)):
                value += poses[poses_index].data[gg_index]

                if poses[poses_index].data[gg_index] > maxi:
                    maxi = poses[poses_index].data[gg_index]

                elif poses[poses_index].data[gg_index] < mini:
                    mini = poses[poses_index].data[gg_index]

            #value = value - maxi - mini
            value = value / (len(poses))#-2)
            pose_gg.append(value)
        # print pose_gg

        if not pose_gg is None:
            if move(pose_gg) == -1:
                print("ERROR")
            # rate.sleep()
            first = False
    if mistakes >= 5:
        if first:
            tools.move2([0, 0, 200, 0, 0], min_z, listener, group, planning_frame, eef_link, gripper, run_program)
            first = False
            mistakes = 0
        else:
            done = starter()
# rospy.spin()


    #-5.65995602607727, -23.948228645324708, 464.4, -0.5571989595890046

    #[54.38340740203857, -18.848447799682617, 468.4, -0.17701317369937897]
    #[62.10660629272461, -16.417512321472167, 473.4, -0.05969421975314617]




