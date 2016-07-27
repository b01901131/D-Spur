import cv2
import numpy as np
import copy

SCALE = 2

def get_color():
	return np.random.randint(0,255,3)

def get_max_contour(contours):
	idx = -1
	max_val = 0
	for i in range(len(contours)):
		if len(contours[i]) > max_val:
			max_val = len(contours[i])
			idx = i
	return idx

def locate_puzzle():

	# Load image
	img = cv2.imread('../img/cifar2.jpg', cv2.CV_LOAD_IMAGE_COLOR)
	img = cv2.resize(img, (0,0), fx=1.0/SCALE, fy=1.0/SCALE )
	# GreyScaling & Thresholding
	img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret , img_gray = cv2.threshold(img_gray, 0, 255, (cv2.THRESH_BINARY + cv2.THRESH_OTSU));
	
	# Morphological operations
	morph = cv2.dilate(img_gray, cv2.getStructuringElement(cv2.MORPH_RECT,(60/SCALE,60/SCALE)))
	morph = cv2.erode(morph, cv2.getStructuringElement(cv2.MORPH_RECT,(60/SCALE,60/SCALE)))

	# Edge detection
	canny = cv2.Canny(morph, 100, 200);
	# cv2.imshow('canny',canny)
	# cv2.waitKey(0)
	
	# Contours detection
	contours, i = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Get & Draw Largest Contour
	idx = get_max_contour(contours)
	drawing = np.zeros(np.array(img).shape, dtype=np.uint8)
	cv2.drawContours(drawing, contours, idx, get_color())
	# cv2.imshow('draw',drawing)
	# cv2.waitKey(0)

	# Approximate Poly DP
	square = []
	approx = cv2.approxPolyDP(contours[idx], cv2.arcLength(contours[idx], True)*0.02, True)
	print 'Approx size: ', np.array(approx).shape
	for n in range(len(approx)):
		cont = False
		for m in range(n):
			dist = cv2.norm(approx[n]-approx[m])
			#print approx[n], apporx[m], dist
			#raw_input()
			if dist < (40/SCALE):
				cont = True
				break
		if cont == True:
			continue
		else:
			center = (approx[n][0][0],approx[n][0][1])
			cv2.circle(drawing, center, 2, get_color(), 2)
			square.append( [float(i) for i in center])
			# cv2.imshow('draw',drawing)
			# cv2.waitKey(0)

	# Find Homography for Appoximated PolyDP
	transformed = np.zeros((512,512), dtype=np.uint8)
	dst = [(0.0,0.0),(0.0,511.0),(511.0,511.0),(511.0,0.0)]
	h,_ = cv2.findHomography(np.array(square), np.array(dst), 0)
	#print h

	# Warp Perspective Transformation	
	warp = cv2.warpPerspective(img, h, transformed.shape)
	# cv2.imshow('warp',warp)
	# cv2.waitKey(0)
	return warp
	

def crop_cifar_image(img):

	img2 = copy.copy(img)
	cv2.imshow('img2',img2)
	cv2.waitKey(0)
	crops = []

	for i in range(16):
		for j in range(16):
			cv2.rectangle(img2, (32*i,32*j), (32*(i+1),32*(j+1)), (0,0,255), 2)
			crops.append(img[32*i:32*(i+1),32*j:32*(j+1)])
	
	cv2.imshow('img2',img2)
	cv2.waitKey(0)

	return crops


puzzle = locate_puzzle()
imgs = crop_cifar_image(puzzle)
