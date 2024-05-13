#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
# from gazebo_msgs.msg import LinkState, LinkStates
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
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
joint_received = False



folderpath = os.path.dirname(os.path.abspath(__file__)) + "/displacement_files/"
config = input("Enter config number:")
file = open(folderpath + "pose_rexrov_q" + str(config) + ".txt", "w")



forces = np.zeros((12,6))

forces[0] = np.array([1,0,0,0,0,0])*0.1
forces[1] = np.array([0,1,0,0,0,0])*0.1
forces[2] = np.array([0,0,1,0,0,0])*0.1

forces[3] = np.array([1,0,0,0,0,0])*1
forces[4] = np.array([0,1,0,0,0,0])*1
forces[5] = np.array([0,0,1,0,0,0])*1

forces[6] = np.array([1,0,0,0,0,0])*5
forces[7] = np.array([0,1,0,0,0,0])*5
forces[8] = np.array([0,0,1,0,0,0])*5

forces[9] = np.array([1,0,0,0,0,0])*10
forces[10] = np.array([0,1,0,0,0,0])*10
forces[11] = np.array([0,0,1,0,0,0])*10

N = np.size(forces, 0)

joint_pid_string = [\
	"J1: 80\n",\
	"J2: 600\n",\
	"J3: 100\n",\
	"J4: 20\n",\
	"J5: 30\n",\
	"J6: 10\n",\
	"\n"]



qL_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qR_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



def sub1_cb(msg):
	global pose_x, pose_y, pose_z
	global ang_x, ang_y, ang_z

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



def sub2_cb(msg):
	global perch_stable
	perch_stable = msg.data



def sub3_cb(msg):
	global qL_start, qR_start, joint_received
	if joint_received == False:
		q1_start[0] = msg.position[1]
		q1_start[1] = msg.position[2]
		q1_start[2] = msg.position[3]
		q1_start[3] = msg.position[4]
		q1_start[4] = msg.position[5]
		q1_start[5] = msg.position[6]

		q2_start[0] = msg.position[12]
		q2_start[1] = msg.position[13]
		q2_start[2] = msg.position[14]
		q2_start[3] = msg.position[15]
		q2_start[4] = msg.position[16]
		q2_start[5] = msg.position[17]
		joint_received = True



def apply_force(x, y, z, ax, ay, az):
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
		print apply_body_wrench(body_name = "rexrov::rexrov/base_link",
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
	global qL_start, qR_start


	print("Zero wrench...")
	apply_force(0, 0, 0, 0, 0, 0)

	try:
		input("Press enter...")
	except SyntaxError:
		pass
	
	rospy.sleep(0.1)

	pose_start = [pose_x, pose_y, pose_z, ang_x, ang_y, ang_z]

	print("Pose start: " + str(pose_start))
	print("Arm L joint start" + str(qL_start))
	print("Arm R joint start" + str(qR_start))

	L = ["QL: \n",\
		str(qL_start[0]) + "\n",\
		str(qL_start[1]) + "\n",\
		str(qL_start[2]) + "\n",\
		str(qL_start[3]) + "\n",\
		str(qL_start[4]) + "\n",\
		str(qL_start[5]) + "\n",\
		"\n"]
	file.writelines(L)

	L = ["QR: \n",\
		str(qR_start[0]) + "\n",\
		str(qR_start[1]) + "\n",\
		str(qR_start[2]) + "\n",\
		str(qR_start[3]) + "\n",\
		str(qR_start[4]) + "\n",\
		str(qR_start[5]) + "\n",\
		"\n"]
	file.writelines(L)

	file.writelines(joint_pid_string)

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
	sub1 = rospy.Subscriber("/rexrov/pose_gt", Odometry, callback=sub1_cb)
	sub2 = rospy.Subscriber("/perch_stable", Bool, callback=sub2_cb)
	sub3 = rospy.Subscriber("rexrov/joint_states", JointState, callback=sub3_cb)

	rate = rospy.Rate(100)


	while not rospy.is_shutdown():
		wrench_loop()
		break
		rate.sleep()


	

	# rate = rospy.Rate(100)

	# while not rospy.is_shutdown():
	# 	avg_window()
	# 	rate.sleep()