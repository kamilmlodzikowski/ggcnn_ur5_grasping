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


def starter():
    global MIN_Z

    if MIN_Z == 0 or MIN_Z is None:
        order = 'moveLookingDown'
        # print order
        print("publishing order ", order)
        rospy.sleep(1)

        pub_order.publish(order)

        _ = rospy.wait_for_message('/execute_trajectory/result', ExecuteTrajectoryActionResult)
    rospy.sleep(1)
    cmd = Float64(160)
    gripper(cmd)

    return True


def move(pose_g):
    global start
    global done
    global once
    global last_z
    global time_last, time_now
    global listener
    global min_z

    rate = rospy.Rate(0.5)
    time_now = time.time()
    if done:
        try:
            # listener.waitForTransform('/base_link', '/right_arm_ee_link', rospy.Time(0), rospy.Duration(1))
            trans, _ = listener.lookupTransform('/base_link', '/right_arm_ee_link', rospy.Time(0))
            cur_z = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Couldn't find '/base_link' -> '/ee_link' transform"
            return -1
        if not (pose_g.data[0] == 0 or pose_g.data[1] == 0 or pose_g.data[2] > last_z):
            #print('Publishing new pose')
            tools.move2(pose_g, pub, cur_z, min_z)
            time_last = time.time()
            last_z = pose_g.data[2]
            rate.sleep()
        else:
            done = False
            print int(time_now-time_last)
            if int(time_now-time_last) >= 8:
                try:
                    print("Grasping...")
                    cmd = Float64(0)
                    gripper(cmd)
                    rospy.sleep(3)
                    done = starter()
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

last_z = 1000
ok = False
start = False
done = False
time_last = time.time()
time_now = time.time()

rospy.init_node('move')

pub = rospy.Publisher('/controller_ur/move_to_pose', PoseStamped, queue_size=1)
pub_depth = rospy.Publisher('object_detection/depth_GG', Image, queue_size=1)
pub_order = rospy.Publisher('/controller_ur/order', String, queue_size=1)
#pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)

print("Waiting for gripper service...")
rospy.wait_for_service('/rg2_gripper/control_width')
gripper = rospy.ServiceProxy('/rg2_gripper/control_width', RG6_grip)
print("Got it\n")

MIN_Z = 1.08446212623
# MIN_Z = None

order = 'moveLookingDown'
print("publishing order ", order)
rospy.sleep(1)
pub_order.publish(order)
rospy.sleep(5)

if MIN_Z == 0 or MIN_Z is None:
    cmd = Float64(0)
    gripper(cmd)

listener = tf.TransformListener()

if MIN_Z == 0 or MIN_Z is None:
    print("Move arm to touch the table")
    raw_input("Press enter...")

    listener.waitForTransform('/base_link', '/right_arm_ee_link', rospy.Time(0), rospy.Duration(1))
    trans, _ = listener.lookupTransform('/base_link', '/right_arm_ee_link', rospy.Time(0))
    min_z = trans[2]
else:
    min_z = MIN_Z

print("Table Z is at " + str(min_z) +"m")


# min_z = input('Enter minimum z value (default for "coffee_table" is 1.2)')
if (not start) and (not done):
    done = starter()

while not rospy.is_shutdown():
    depth = rospy.wait_for_message('object_detection/depth', Image)
    pub_depth.publish(depth)
    pose_gg = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
    move(pose_gg)
    rospy.spin()


