#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkState, LinkStates
from gazebo_msgs.srv import GetLinkState, ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench
from scipy.spatial.transform import Rotation as R
import numpy as np
import os



pose_x = 0.0
pose_y = 0.0
pose_z = 0.0
ang_x = 0.0
ang_y = 0.0
ang_z = 0.0



folderpath = os.path.dirname(os.path.abspath(__file__)) + "/displacement_files/"
config = input("Enter config number:")
file = open(folderpath + "pose_q" + str(config) + ".txt", "w")


forces = [[1,0,0,0,0,0],\
	[0,1,0,0,0,0],\
	[0,0,1,0,0,0],\
	[-1,0,0,0,0,0],\
	[0,-1,0,0,0,0],\
	[0,0,-1,0,0,0]]



def sub1_cb(msg):
	global pose_x, pose_y, pose_z
	global ang_x, ang_y, ang_z

	poses = msg.pose
	pose_x = poses[0].position.x
	pose_y = poses[0].position.y
	pose_z = poses[0].position.z

	x = poses[0].orientation.x
	y = poses[0].orientation.y
	z = poses[0].orientation.z
	w = poses[0].orientation.w

	rotvec = R.as_rotvec(R.from_quat([x, y, z, w]))

	ang_x = rotvec[0]
	ang_y = rotvec[1]
	ang_z = rotvec[2]



def apply_force(x,y,z,ax,ay,az):
	rospy.wait_for_service('/gazebo/apply_body_wrench')
	apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

	w = Wrench()
	w.force.x = x
	w.force.y = y
	w.force.z = z
	w.torque.x = ax
	w.torque.y = ay
	w.torque.z = az

	pt = Point()
	pt.x = 0
	pt.y = 0
	pt.z = 0

	try:
		print apply_body_wrench(body_name = "robot::linkROV",
			reference_frame = "",
			reference_point = pt,
			wrench = w,
			start_time = rospy.Time(),
			duration = rospy.Duration(-1))

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)



def wait_for_stable(F):
	print("Applying F" + str(F+1) + "...")
	force = forces[F]
	apply_force(force[0],force[1],force[2],force[3],force[4],force[5])

	try:
		input("Press enter when F" + str(F+1) + " is stable...")
	except SyntaxError:
		pass



def write_pose(F, x, y, z, ax, ay, az):
	X = [str(x + 0.2286) + "\n",\
		str(y) + "\n",\
		str(z) + "\n",\
		str(ax) + "\n",\
		str(ay) + "\n",\
		str(az) + "\n"]

	L = ["F" + str(F+1) + ":\n",\
		X[0],\
		X[1],\
		X[2],\
		X[3],\
		X[4],\
		X[5],\
		"\n"]

	file.writelines(L)



def wrench_loop():
	global pose_x, pose_y, pose_z
	global ang_x, ang_y, ang_z



	print("Zero wrench...")
	apply_force(0,0,0,0,0,0)
	
	try:
		input("Press enter...")
	except SyntaxError:
		pass
	
	for i in range(6):
		wait_for_stable(i)
		rospy.sleep(1)
		write_pose(i, pose_x, pose_y, pose_z, ang_x, ang_y, ang_z)

	print("Zero wrench...")
	apply_force(0,0,0,0,0,0)

	file.close()
	print("DONE")



if __name__ == '__main__':

	# Initialize the node
	rospy.init_node("wrench_node", anonymous=True)
	sub1 = rospy.Subscriber("/gazebo/link_states", LinkStates, callback=sub1_cb)

	rate = rospy.Rate(100)


	while not rospy.is_shutdown():
		wrench_loop()
		break
		rate.sleep()


	

	# rate = rospy.Rate(100)

	# while not rospy.is_shutdown():
	# 	avg_window()
	# 	rate.sleep()