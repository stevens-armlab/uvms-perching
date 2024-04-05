#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkState, LinkStates
from gazebo_msgs.srv import GetLinkState
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy import stats
import time

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

def sub1_cb(msg):
	global pose_x, pose_y, pose_z, values_x, values_y, values_z
	global ang_x, ang_y, ang_z, values_ax, values_ay, values_az
	global t

	poses = msg.pose
	pose_x = poses[0].position.x
	pose_y = poses[0].position.y
	pose_z = poses[0].position.z

	x = poses[0].orientation.x
	y = poses[0].orientation.y
	z = poses[0].orientation.z
	w = poses[0].orientation.w

	# rotmat = R.as_dcm(R.from_quat([x, y, z, w]))
	# theta = np.arccos((np.trace(rotmat)-1)/2)
	# m = 1/(2*np.sin(theta))*np.array([\
	# 	rotmat[2,1] - rotmat[1,2],\
	# 	rotmat[0,2] - rotmat[2,0],\
	# 	rotmat[1,0] - rotmat[0,1]])
	# rotvec = theta*m
	# print(rotvec)
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


def avg_window():
	global values_x, values_y, values_z
	global values_ax, values_ay, values_az
	global t

	avg_x = np.average(values_x)
	avg_y = np.average(values_y)
	avg_z = np.average(values_z)
	avg_ax = np.average(values_ax)
	avg_ay = np.average(values_ay)
	avg_az = np.average(values_az)

	# x = list(range(0, N))
	# print(x)
	# print(x[1:N])
	# print(np.append(x[1:N], 100))
	# print("")

	print("Avg X: " + str(avg_x))
	print("Avg Y: " + str(avg_y))
	print("Avg Z: " + str(avg_z))
	print("")

	print("Avg Ang X: " + str(avg_ax))
	print("Avg Ang Y: " + str(avg_ay))
	print("Avg Ang Z: " + str(avg_az))
	print("")

	# print("T: " + str(t))
	# print("")

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
	if all(np.absolute(x) < 1e-8 for x in S):
		print("Steady state reached")
		print("")

	print("================================")
	print("")

	time.sleep(1)



if __name__ == '__main__':

	# Initialize the node
	rospy.init_node("link_poses")

	# Initialize the subscribers
	sub1 = rospy.Subscriber("/gazebo/link_states", LinkStates, callback=sub1_cb)

	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		avg_window()
		rate.sleep()