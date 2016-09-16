import numpy as np
#import theano
#import theano.Tensor as T
import time


th_zero = [0, 90, -90, 0, 180, -180]
L  = [10.0, 105.0, 20.0]


th = [0,90]

dest = [-310,630]

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
			[0.0,                      0.0,  1.0,                  L[2]],
			[0.0,                      0.0,  0.0,                   1.0]
		 ]

		 
	Tb = [  [    			1.0,           0.0,             0.0,  				30.0],
			[    			0.0,           1.0,             0.0,  				  0.0],
			[    			0.0,           0.0,             1.0,  				 95.0],
			[               0.0,           0.0,             0.0,                  1.0]
		 ]


	T = np.dot( np.dot(Tb,T0),T1 )
	tangent = np.dot(T,[[1],[0],[0],[0]])
	

	pose = np.array([ T[0][3], T[1][3], T[2][3] ])

	#print pose
	# screen at x = -500 mm

	pose += ( (-500-pose[0]) / tangent[0]) * np.transpose(tangent[:3])[0]

	return pose[1:]



def rad_to_deg(rad):
	return rad/np.pi*180.0

def deg_to_rad(deg):
	return deg/180.0*np.pi

def IK(curr_th, dest, time_span):
	time_st = time.time()
	curr = FK(curr_th)
	d = dist(dest,curr)
	mini = d
	th_mini = curr_th
	#print curr
	while d > 0.5 and time.time() - time_st < time_span:
		d_x = dest - curr
	
		d_x = np.divide(d_x,dist(d_x))

		d_q = np.dot(inverse_jac(curr_th),d_x)
		
		curr_th += d_q

		for i in range(len(curr_th)):
			if curr_th[i]>90 :
				curr_th[i] = 90
			if curr_th[i]<-180:
				curr_th[i] = -180

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
		# print d
		#print "////////////////////////////"
	

	for i in range(2):
		th_mini[i] = round(th_mini[i],2)
		
	curr = FK(th_mini)
	print th_mini#,dist(dest,curr)
	print curr
	print dest
	print dist(dest,curr)
	return th_mini

def dist(curr,dest = [0,0]):
	summary = 0.0
	for i in range(2):
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

IK(th,dest,0.5)
#print FK(th)