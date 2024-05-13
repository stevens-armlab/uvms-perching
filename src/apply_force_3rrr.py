#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
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
start_pose = [pose_x, pose_y, pose_z, ang_x, ang_y, ang_z]
perch_stable = False



folderpath = os.path.dirname(os.path.abspath(__file__)) + "/displacement_files/"
config = input("Enter config number:")
file = open(folderpath + "pose_3rrr_x" + str(config) + "_eig.txt", "w")



# forces = [\
# 	[0.0001, 0,    0, 0, 0, 0],\
# 	[0,    0.0001, 0, 0, 0, 0],\
# 	[0,    0,    0, 0, 0, 0.0001],\

# 	[0.001, 0,    0, 0, 0, 0],\
# 	[0,    0.001, 0, 0, 0, 0],\
# 	[0,    0,    0, 0, 0, 0.001],\

# 	[0.01, 0,    0, 0, 0, 0],\
# 	[0,    0.01, 0, 0, 0, 0],\
# 	[0,    0,    0, 0, 0, 0.01],\

# 	[0.1, 0,   0, 0, 0, 0],\
# 	[0,   0.1, 0, 0, 0, 0],\
# 	[0,   0,   0, 0, 0, 0.1],\

# 	[1, 0, 0, 0, 0, 0],\
# 	[0, 1, 0, 0, 0, 0],\
# 	[0, 0, 0, 0, 0, 1]]

forces = [\
    [-3.3546e-06, -2.0008e-06, 0, 0, 0, 1.2527e-05],\
    [-4.3294e-07, -5.5412e-07, 0, 0, 0, -2.0444e-07],\
    [8.2097e-08, -6.8234e-08, 0, 0, 0, 1.1086e-08],\
    [-3.3546e-05, -2.0008e-05, 0, 0, 0, 0.00012527],\
    [-4.3294e-06, -5.5412e-06, 0, 0, 0, -2.0444e-06],\
    [8.2097e-07, -6.8234e-07, 0, 0, 0, 1.1086e-07],\
    [-0.00033546, -0.00020008, 0, 0, 0, 0.0012527],\
    [-4.3294e-05, -5.5412e-05, 0, 0, 0, -2.0444e-05],\
    [8.2097e-06, -6.8234e-06, 0, 0, 0, 1.1086e-06]]

N = np.size(forces,0)



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



def sub2_cb(msg):
	global perch_stable
	perch_stable = msg.data



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
		print apply_body_wrench(body_name = "robot::base_link",
			reference_frame = "",
			reference_point = pt,
			wrench = w,
			start_time = rospy.Time(),
			duration = rospy.Duration(-1))

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)



def apply_force_F(F):
	print("Applying F" + str(F+1) + "...")
	force = forces[F]
	apply_force(force[0],force[1],force[2],force[3],force[4],force[5])

	# try:
	# 	input("Press enter when F" + str(F+1) + " is stable...")
	# except SyntaxError:
	# 	pass



def write_pose(F, x, y, z, ax, ay, az):
	global pose_start
	X = [str(x - pose_start[0]) + "\n",\
		str(y - pose_start[1]) + "\n",\
		str(z - pose_start[2]) + "\n",\
		str(ax - pose_start[3]) + "\n",\
		str(ay - pose_start[4]) + "\n",\
		str(az - pose_start[5]) + "\n"]

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
	global perch_stable
	global pose_start


	print("Zero wrench...")
	apply_force(0, 0, 0, 0, 0, 0)

	try:
		input("Press enter...")
	except SyntaxError:
		pass
	
	rospy.sleep(0.1)

	pose_start = [pose_x, pose_y, pose_z, ang_x, ang_y, ang_z]
	print("Pose start: " + str(pose_start))
	for i in range(N):
		apply_force_F(i)
		rospy.sleep(1)
		while 1:
			if perch_stable == True:
				break
				
		print("Perch stability reached, writing pose to file...")
		write_pose(i, pose_x, pose_y, pose_z, ang_x, ang_y, ang_z)
		rospy.sleep(1)

	print("Zero wrench...")
	apply_force(0, 0, 0, 0, 0, 0)

	file.close()
	print("DONE")



if __name__ == '__main__':

	# Initialize the node
	rospy.init_node("wrench_node", anonymous=True)
	sub1 = rospy.Subscriber("/gazebo/link_states", LinkStates, callback=sub1_cb)
	sub2 = rospy.Subscriber("/perch_stable", Bool, callback=sub2_cb)

	rate = rospy.Rate(100)


	while not rospy.is_shutdown():
		wrench_loop()
		break
		rate.sleep()


	

	# rate = rospy.Rate(100)

	# while not rospy.is_shutdown():
	# 	avg_window()
	# 	rate.sleep()