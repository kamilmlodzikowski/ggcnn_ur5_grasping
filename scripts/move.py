#! /usr/bin/env python

import rospy
import tools
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time

def move(pose_g):
    global start
    global done
    # global min_z
    global once
    if (not start) and (not done):
        order = 'moveLookingDown'
        # print order
        rospy.sleep(0.2)
        pub_order.publish(order)
        done = True

    elif (not start) and done:
        # rospy.sleep(5.)
        msg = rospy.wait_for_message('/controller_ur/currentState', String)
        # print msg.data
        if msg.data == 'STATE_MOVING':
            once = True
        if once and msg.data == 'STATE_NORMAL':
            start = True

    elif start:
        global last_z
        #print('Got new data')
        rate = rospy.Rate(1)
        if not (pose_g.data[0] == 0 or pose_g.data[1] == 0 or pose_g.data[2] <= 0.15 or pose_g.data[2] > last_z):
            #print('Publishing new pose')
            tools.move2(pose_g, pub, 1.45)
            rate.sleep()
	
    # else:
    #     time_now = time.time()
    #     if ok == True:
    #         passed = int(time_now - time_before)
    #         print passed
    #         if passed >= 5:
    #             time.sleep(0.5)
    #             manipulator.grip(0)
    #             time.sleep(5)
    #             teraz = tools.get_pose(manipulator)
    #             teraz[0] -= 0.08
    #             teraz[1] -= 0.10
    #             teraz[2] -= 0.10
    #             trajectory = list()
    #             trajectory.append(teraz)
    #             manipulator.move(trajectory, False, a=0.1, v=0.8)
    #             time.sleep(4)
    #             tools.goto(box, manipulator)
    #             tools.goto(start, manipulator)
    #             last_z = 1000
    #             ok = False
    #             tools.check_joints(manipulator)


rospy.init_node('move')
pub = rospy.Publisher('/controller_ur/move_to_pose', PoseStamped, queue_size=1)
pub_order = rospy.Publisher('/controller_ur/order', String, queue_size=1)
pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)

last_z = 1000
ok = False
start = False
done = False
once = False
# min_z = input('Enter minimum z value (default for "coffee_table" is 1.5)')

while not rospy.is_shutdown():
    rospy.spin()


