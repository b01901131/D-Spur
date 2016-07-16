import numpy as np
#import theano
#import theano.Tensor as T
import time


th = [0,0,0,0,90,180]
L  = [10.0,20.0,30.0,40.0,50.0,60.0]




def FK(th):

	time_st = time.time()
	local_th = th[:]
	for i in range(len(local_th)):
		local_th[i] = local_th[i]/180.0*np.pi
	#print "\n-------Forward Kinematics-------"
	#print "Theta\t", th
	#print "L\t",L

	T0 = [  [np.cos(th[0]), 0.0,  np.sin(th[0]),  L[0]*np.cos(th[0])],
			[np.sin(th[0]), 0.0, -np.cos(th[0]),  L[0]*np.sin(th[0])],
			[0.0,           1.0,            0.0,                 0.0],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T1 = [  [np.cos(th[1]), -np.sin(th[1]),  0.0,    L[1]*np.cos(th[1])],
			[np.sin(th[1]),  np.cos(th[1]),  0.0,    L[1]*np.sin(th[1])],
			[0.0,                      0.0,  1.0,                   0.0],
			[0.0,                      0.0,  0.0,                   1.0]
		 ]

	T2 = [  [np.cos(th[2]), 0.0, -np.sin(th[2]),  L[2]*np.cos(th[2])],
			[np.sin(th[2]), 0.0,  np.cos(th[2]),  L[2]*np.sin(th[2])],
			[0.0,           1.0,            0.0,                 0.0],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T3 = [  [np.cos(th[3]), 0.0,  np.sin(th[3]),  				 0.0],
			[np.sin(th[3]), 0.0, -np.cos(th[3]),  				 0.0],
			[0.0,           1.0,            0.0,                L[3]],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T4 = [  [np.cos(th[4]), 0.0, -np.sin(th[4]),  L[4]*np.cos(th[4])],
			[np.sin(th[4]), 0.0,  np.cos(th[4]),  L[4]*np.sin(th[4])],
			[0.0,          -1.0,            0.0,                 0.0],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T5 = [  [np.cos(th[5]), -np.sin(th[5]),  0.0,  				 0.0],
			[np.sin(th[5]),  np.cos(th[5]),  0.0,  				 0.0],
			[0.0,           0.0,             1.0,               L[5]],
			[0.0,           0.0,             0.0,                1.0]
		 ]


	T = np.dot(np.dot(np.dot(T0,T1),np.dot(T2,T3)),np.dot(T4,T5))
	pitch = np.arcsin(-1*T[2][0])
	roll  = np.arccos(T[2][2]/np.cos(pitch))
	yaw   = np.arccos(T[0][0]/np.cos(pitch))
	pose  = np.dot(T,[0,0,0,1])[:-1]

	print "Pose\t",np.dot(T,[0,0,0,1])[:-1]
	print "Roll : %f\tPitch : %f\tYaw : %f" % (rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw))
	#print "Time:\t %.4f s" % (time.time()-time_st)
	return pose
	
def rad_to_deg(rad):
	return rad/np.pi*180.0

def deg_to_rad(deg):
	return deg/180.0*np.pi

def get_limit():

	delta = 0.001
	current = FK(th)
	for i in range(len(th)):
		new_th = th[:]
		new_th[i] += delta

		#print new_th, th
		print FK(new_th),FK(th)
		print np.divide(np.subtract(FK(new_th),current),delta)
		

		raw_input()

def IK(dest, cur_th):
	pass

FK(th)
#get_limit()


