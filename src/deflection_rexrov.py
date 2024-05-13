#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
# from gazebo_msgs.msg import LinkState, LinkStates
# from gazebo_msgs.srv import GetLinkState
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy import stats
import time

threshold = 1e-6

pose_x = 0.0
pose_y = 0.0
pose_z = 0.0
ang_x = 0.0
ang_y = 0.0
ang_z = 0.0

N = 500
values_x = np.zeros(N)
values_y = np.zeros(N)
values_z = np.zeros(N)
values_ax = np.zeros(N)
values_ay = np.zeros(N)
values_az = np.zeros(N)

t = np.zeros(N)

pub1 = rospy.Publisher('perch_stable', Bool, queue_size=1)

qL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def sub1_cb(msg):
	global pose_x, pose_y, pose_z, values_x, values_y, values_z
	global ang_x, ang_y, ang_z, values_ax, values_ay, values_az
	global t

	pos = msg.pose.pose.position
	ang = msg.pose.pose.orientation
	pose_x = pos.x
	pose_y = pos.y
	pose_z = pos.z

	x = ang.x
	y = ang.y
	z = ang.z
	w = ang.w
	rotvec = R.as_rotvec(R.from_quat([x, y, z, w]))

	ang_x = rotvec[0]
	ang_y = rotvec[1]
	ang_z = rotvec[2]

	values_x = np.append(values_x[1:N], pose_x)
	values_y = np.append(values_y[1:N], pose_y)
	values_z = np.append(values_z[1:N], pose_z)

	values_ax = np.append(values_ax[1:N], ang_x)
	values_ay = np.append(values_ay[1:N], ang_y)
	values_az = np.append(values_az[1:N], ang_z)

	now = rospy.get_rostime()
	t = np.append(t[1:N], now.secs + now.nsecs*1e-9)
	# print("x: " + str(x))
	# print("y: " + str(y))
	# print("z: " + str(z))
	# print("w: " + str(w))
	# print("")



def sub2_cb(msg):
	global qL, qR
	qL[0] = msg.position[1]
	qL[1] = msg.position[2]
	qL[2] = msg.position[3]
	qL[3] = msg.position[4]
	qL[4] = msg.position[5]
	qL[5] = msg.position[6]

	qR[0] = msg.position[12]
	qR[1] = msg.position[13]
	qR[2] = msg.position[14]
	qR[3] = msg.position[15]
	qR[4] = msg.position[16]
	qR[5] = msg.position[17]



def avg_window():
	global values_x, values_y, values_z
	global values_ax, values_ay, values_az
	global t
	global qL, qR

	avg_x = np.average(values_x)
	avg_y = np.average(values_y)
	avg_z = np.average(values_z)
	avg_ax = np.average(values_ax)
	avg_ay = np.average(values_ay)
	avg_az = np.average(values_az)

	print("Avg X: " + str(avg_x))
	print("Avg Y: " + str(avg_y))
	print("Avg Z: " + str(avg_z))
	print("")

	print("Avg Ang X: " + str(avg_ax))
	print("Avg Ang Y: " + str(avg_ay))
	print("Avg Ang Z: " + str(avg_az))
	print("")

	slope_x, i, r, p, ste = stats.linregress(t, values_x)
	slope_y, i, r, p, ste = stats.linregress(t, values_y)
	slope_z, i, r, p, ste = stats.linregress(t, values_z)
	slope_ax, i, r, p, ste = stats.linregress(t, values_ax)
	slope_ay, i, r, p, ste = stats.linregress(t, values_ay)
	slope_az, i, r, p, ste = stats.linregress(t, values_az)

	print("Slope X: " + str(slope_x))
	print("Slope Y: " + str(slope_y))
	print("Slope Z: " + str(slope_z))
	print("")
	
	print("Slope Ang X: " + str(slope_ax))
	print("Slope Ang Y: " + str(slope_ay))
	print("Slope Ang Z: " + str(slope_az))
	print("")

	# S = [np.linalg.norm(np.array([slope_x, slope_y, slope_z, slope_ax, slope_ay, slope_az]))]
	S = [slope_x, slope_y, slope_z, slope_ax, slope_ay, slope_az]

	stable = Bool()
	if all(np.absolute(x) < threshold for x in S):
		print("Steady state reached")
		print("")
		stable = True
		pub1.publish(stable)
	else:
		stable = False
		pub1.publish(stable)

	print("================================")
	print("")

	print("Joint Values: ")
	print("QL: " + str(qL))
	print("QR: " + str(qR))
	print("")

	print("================================")
	print("")

	# time.sleep(0.1)



if __name__ == '__main__':

	# Initialize the node
	rospy.init_node("rexrov_poses")

	# Initialize the subscribers
	sub1 = rospy.Subscriber("/rexrov/pose_gt", Odometry, callback=sub1_cb)
	sub2 = rospy.Subscriber("rexrov/joint_states", JointState, callback=sub2_cb)

	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		avg_window()
		rate.sleep()