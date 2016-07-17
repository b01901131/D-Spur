import numpy as np
#import theano
#import theano.Tensor as T
import time


th_zero = [0, 90, -90, 0, 180, -180]
L  = [10.0, 105.0, 25.0, 110.0, 30.0, 150.0]

#th = [-10, -27, 47, 63, -20, 5]
th = [0,20,0,90,0,90]
#dest = [10, -355, 60, 0, 0, 0]
dest = [  15.23108,      19.98846,     416.15625 ,    -17.1167058,    11.61764334,
   53.11618079]
#dest = [15,      19,     416,     -17,    11,  53] 


def FK(th):

	time_st = time.time()
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


	T = np.dot(np.dot(np.dot(T0,T1),np.dot(T2,T3)),np.dot(T4,T5))
	for i in range(4):
		for j in range(4):
			T[i][j] = round(T[i][j],5)

	roll = np.arctan2(T[2][1],T[2][2]) #* 180/np.pi
	pitch = np.arctan2(-T[2][0],np.sqrt(T[2][1]**2+T[2][2]**2)) #* 180/np.pi
	yaw = np.arctan2(T[1][0],T[0][0]) #* 180/np.pi


	if round(np.cos(pitch),5) == 0:

		yaw = deg_to_rad(th[5] + th[4] + th[0]) #* 180/np.pi

		if round(np.sin(yaw),5) == 0:
			pitch = np.arctan2(-T[2][0],T[2][2]/np.cos(yaw))
		else:
			pitch = np.arctan2(-T[2][0],T[2][1]/np.sin(yaw)) #* 180/np.pi

		if not round(np.cos(pitch),5) == 0:
			roll = np.arctan2(T[2][1]/np.cos(pitch),T[2][2]/np.cos(pitch)) #* 180/np.pi
		elif np.sin(pitch)>0:
			roll = yaw - np.arctan2(T[1][2],T[1][1])
		else:
			roll = np.arctan2(-T[1][2],T[1][1]) - yaw




	#print pose

	Tr =  	[  	[            1.0,            0.0,            0.0,            0.0],
				[            0.0,   np.cos(roll),  -np.sin(roll),            0.0],
				[            0.0,   np.sin(roll),   np.cos(roll),            0.0],
				[            0.0,            0.0,            0.0,            1.0],
			]
	Tp =  	[  	[  np.cos(pitch),            0.0,  np.sin(pitch),            0.0],
				[            0.0,            1.0,            0.0,            0.0],
				[ -np.sin(pitch),            0.0,  np.cos(pitch),            0.0],
				[            0.0,            0.0,            0.0,            1.0],
			]			
	Ty =  	[  	[    np.cos(yaw),   -np.sin(yaw),            0.0,            0.0],
				[    np.sin(yaw),    np.cos(yaw),            0.0,            0.0],
				[            0.0,            0.0,            1.0,            0.0],
				[            0.0,            0.0,            0.0,            1.0],
			]
	Tt =  	[  	[            1.0,            0.0,            0.0,        T[0][3]],
				[            0.0,            1.0,            0.0,        T[1][3]],
				[            0.0,            0.0,            1.0,        T[2][3]],
				[            0.0,            0.0,            0.0,            1.0],
			]
	#print "Pose\t",np.dot(T,[0,0,0,1])[:-1]
	#print "Time:\t %.4f s" % (time.time()-time_st)

	#print T , '\n'
	#print pose[0],pose[1],pose[2]
	Ttest = np.dot(np.dot(Tt,Ty),np.dot(Tp,Tr))
	for i in range(4):
		for j in range(4):
			Ttest[i][j] = round(Ttest[i][j],5)
	#print abs(Ttest - T)<0.001
	pose = np.array([ T[0][3]+10, T[1][3], T[2][3]+65, rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw) ]) # x,y,z,roll,pitch,yaw
	return pose



def rad_to_deg(rad):
	return rad/np.pi*180.0

def deg_to_rad(deg):
	return deg/180.0*np.pi

def IK(curr_th, dest):
	curr = FK(curr_th)
	d = dist(dest,curr)
	#print curr
	while d > 10:
		d_x = dest - curr
	
		d_x = np.divide(d_x,dist(d_x))*1.6

		d_q = np.dot(inverse_jac(curr_th),d_x)
		
		curr_th += d_q

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
		print d
		#print "////////////////////////////"
	

	for i in range(6):
		curr_th[i] = round(curr_th[i],0)
		while not curr_th[i] in range(-180,180):
			curr_th[i] -= np.sign(curr_th[i])*360
	curr = FK(curr_th)
	print curr_th#,dist(dest,curr)
	print curr
	print dist(dest,curr) 

def dist(curr,dest = [0,0,0,0,0,0]):
	summary = 0.0
	for i in range(6):
		summary += (curr[i]-dest[i])**2
	return np.sqrt(summary)
	
def inverse_jac(th_in):

	#print '\n'
	delta = 0.01
	current = FK(th_in)
	jac = []
	Ti =  	[  	[            1.0,            0.0,            0.0,       	 0.0,       	 0.0,       	 0.0],
				[            0.0,            1.0,            0.0,        	 0.0,       	 0.0,       	 0.0],
				[            0.0,            0.0,            1.0,        	 0.0,       	 0.0,       	 0.0],
				[            0.0,            0.0,            0.0,            1.0,       	 0.0,       	 0.0],
				[            0.0,            0.0,            0.0,            0.0,       	 1.0,       	 0.0],
				[            0.0,            0.0,            0.0,            0.0,       	 0.0,       	 1.0],
			]
	for i in range(len(th_in)):
		new_th = list(th_in)
		new_th[i] += delta

		#print new_th, th
		#print FK(new_th)
		jac.append(np.divide(np.subtract(FK(new_th),current),delta))	



		#raw_input()

	jac = np.dot(jac,Ti)
	jac = np.transpose(jac)
	
	#print jac
	in_jac = np.dot( np.linalg.inv( np.dot(np.transpose(jac),jac) ),np.transpose(jac) )
	#print in_jac
	#return np.linalg.inv(jac)
	return in_jac

IK(th,dest)
#print FK(th)

