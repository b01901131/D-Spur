import numpy as np
#import theano
#import theano.Tensor as T
import time


th_zero = [0, 90, -90, 0, 180, -180]
L  = [10.0, 105.0, 25.0, 110.0, 25.0, 150.0]

#th = [-10, -27, 47, 63, -20, 5]
#th = [0,-20,0,30,0,90]
#th = [ -96,  -75,  -34,  -27,   21,  206]
#th = [ -96.07351766,  -74.89197776,  -34.13323947,  -27.17642705,   21.20637644, 205.58019537]
th = [0,0,0,0,0,0]
#th = [  79, -90,  -41,  -11,   90,  -90]
#dest = [10, -355, 60, 10, -365, 60]
#dest = [ 200, 0, 200, 210, 0, 200, 200, 10, 200]
dest = [  130,200,200, 130,200,210, 140,200,200]

def FK(th):

	#time_st = time.time()
	local_th = list(th)
	for i in range(len(local_th)):
		local_th[i] = deg_to_rad(local_th[i]+th_zero[i])
	#print "\n-------Forward Kinematics-------"
	#print "Theta\t", th
	#print "L\t",L

	T0 = [  [np.cos(local_th[0]), 0.0,  np.sin(local_th[0]),  L[0]*np.cos(local_th[0])],
			[np.sin(local_th[0]), 0.0, -np.cos(local_th[0]),  L[0]*np.sin(local_th[0])],
			[0.0,           1.0,            0.0,                 0.0],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T1 = [  [np.cos(local_th[1]), -np.sin(local_th[1]),  0.0,    L[1]*np.cos(local_th[1])],
			[np.sin(local_th[1]),  np.cos(local_th[1]),  0.0,    L[1]*np.sin(local_th[1])],
			[0.0,                      0.0,  1.0,                   0.0],
			[0.0,                      0.0,  0.0,                   1.0]
		 ]

	T2 = [  [np.cos(local_th[2]), 0.0, -np.sin(local_th[2]),  L[2]*np.cos(local_th[2])],
			[np.sin(local_th[2]), 0.0,  np.cos(local_th[2]),  L[2]*np.sin(local_th[2])],
			[0.0,          -1.0,            0.0,                 0.0],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T3 = [  [np.cos(local_th[3]), 0.0,  np.sin(local_th[3]),  				 0.0],
			[np.sin(local_th[3]), 0.0, -np.cos(local_th[3]),  				 0.0],
			[0.0,           1.0,            0.0,                L[3]],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T4 = [  [np.cos(local_th[4]), 0.0,  np.sin(local_th[4]),  L[4]*np.cos(local_th[4])],
			[np.sin(local_th[4]), 0.0, -np.cos(local_th[4]),  L[4]*np.sin(local_th[4])],
			[0.0,           1.0,            0.0,                 0.0],
			[0.0,           0.0,            0.0,                 1.0]
		 ]

	T5 = [  [np.cos(local_th[5]), -np.sin(local_th[5]),  0.0,  				 0.0],
			[np.sin(local_th[5]),  np.cos(local_th[5]),  0.0,  				 0.0],
			[0.0,           0.0,             1.0,               L[5]],
			[0.0,           0.0,             0.0,                1.0]
		 ]
	Tb = [  [    			1.0,           0.0,             0.0,  				-10.0],
			[    			0.0,           1.0,             0.0,  				  0.0],
			[    			0.0,           0.0,             1.0,  				 65.0],
			[               0.0,           0.0,             0.0,                  1.0]
		 ]


	T = np.dot( np.dot( np.dot(Tb,T0),np.dot(T1,T2) ),np.dot( np.dot(T3,T4), T5) )
	front = np.dot(T,[[0],[0],[10],[1]])
	right = np.dot(T,[[10],[0],[0],[1]])

	pose = np.array([ T[0][3], T[1][3], T[2][3], front[0], front[1], front[2], right[0], right[1], right[2] ]) # x,y,z,roll,pitch,yaw
	#for i in range(len(pose)):
	#	pose[i] = round(pose[i],5)
	return pose



def rad_to_deg(rad):
	return rad/np.pi*180.0

def deg_to_rad(deg):
	return deg/180.0*np.pi

def IK(curr_th, dest):
	time_st = time.time()
	curr = FK(curr_th)
	d = dist(dest,curr)
	mini = d
	th_mini = curr_th
	#print curr
	while d > 0.05 and time.time() - time_st < 3:
		d_x = dest - curr
	
		d_x = np.divide(d_x,dist(d_x))

		d_q = np.dot(inverse_jac(curr_th),d_x)
		
		curr_th += d_q

		for i in range(len(curr_th)):
			if curr_th[i]>90 or curr_th[i]<-90:
				curr_th[i] -= d_q[i]

		'''
		for i in range(6):
			curr_th[i] = round(curr_th[i],0)
			while not curr_th[i] in range(-180,180):
				curr_th[i] -= np.sign(curr_th[i])*360
		'''
		curr = FK(curr_th)

		
		#print curr_th#,dist(dest,curr)
		#print curr
		d = dist(dest,curr)
		if d < mini:
			mini = d
			th_mini = list(curr_th)
			#print th_mini
		print d
		#print "////////////////////////////"
	

	for i in range(6):
		th_mini[i] = round(th_mini[i],0)
		while not th_mini[i] in range(-180,180):
			th_mini[i] -= np.sign(th_mini[i])*360
	curr = FK(th_mini)
	print th_mini#,dist(dest,curr)
	print curr
<<<<<<< HEAD
	print dist(dest,curr)
=======
	print dist(dest,curr) 
	return curr_th
>>>>>>> 91522897adf4e24b7a45a7b6dd1a49ea77a2b4d4

def dist(curr,dest = [0,0,0,0,0,0,0,0,0]):
	summary = 0.0
	for i in range(9):
		summary += (curr[i]-dest[i])**2
	return np.sqrt(summary)
	
def inverse_jac(th_in):

	#print '\n'
	delta = 0.001
	current = FK(th_in)
	jac = []
	
	for i in range(len(th_in)):
		new_th = list(th_in)
		new_th[i] += delta

		#print new_th, th
		#print FK(new_th)
		jac.append(np.divide(np.subtract(FK(new_th),current),delta))	



		#raw_input()

	jac = np.transpose(jac)
	#print jac	
	#print jac
	in_jac = np.dot( np.linalg.inv( np.dot(np.transpose(jac),jac) ),np.transpose(jac) )
	#print in_jac
	#return np.linalg.inv(jac)
	return in_jac

#IK(th,dest)
#print FK(th)

