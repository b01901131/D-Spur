import cv2
import numpy as np
import copy

SCALE = 1
H_SIZE = 512.0

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

def sortSquare(square):

	# print square
	sorted_square = square[:]
	avg_x, avg_y = np.average(square, axis = 0)

	for s in square:
		if s[0] < avg_x and s[1] < avg_y:
			sorted_square[0] = s
		if s[0] < avg_x and s[1] > avg_y:
			sorted_square[1] = s
		if s[0] > avg_x and s[1] < avg_y:
			sorted_square[2] = s
		if s[0] > avg_x and s[1] > avg_y:
			sorted_square[3] = s

	return sorted_square


	




def locate_puzzle(img):

	# Load image
	#img = cv2.imread('../img/cifar6.jpg', cv2.CV_LOAD_IMAGE_COLOR)
	img = cv2.resize(img, (0,0), fx=1.0/SCALE, fy=1.0/SCALE )
	img_2 = copy.deepcopy(img)
	# cv2.imshow('img',img)
	# cv2.waitKey(0)
	# GreyScaling & Thresholding
	img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret , img_gray = cv2.threshold(img_gray, 0, 255, (cv2.THRESH_BINARY + cv2.THRESH_OTSU));
	

	# Morphological operations
	morph = cv2.dilate(img_gray, cv2.getStructuringElement(cv2.MORPH_RECT,(11/SCALE,11/SCALE)))
	morph = cv2.erode(morph, cv2.getStructuringElement(cv2.MORPH_RECT,(11/SCALE,11/SCALE)))
	# cv2.imshow('morph',morph)
	# cv2.waitKey(0)

	# Edge detection
	canny = cv2.Canny(morph, 100, 200);
	# cv2.imshow('canny',canny)
	# cv2.waitKey(0)
	
	# Contours detection
	contours, i = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Get & Draw Largest Contour
	idx = get_max_contour(contours)
	drawing = np.zeros(np.array(img).shape, dtype=np.uint8)
	cv2.drawContours(img, contours, idx, (0,0,255), 1)
	# cv2.imshow('draw',drawing)
	# cv2.waitKey(0)

	# Approximate Poly DP
	square = []
	approx = cv2.approxPolyDP(contours[idx], cv2.arcLength(contours[idx], True)*0.02, True)
	# print 'Approx size: ', np.array(approx).shape
	point_cnt = 0
	for n in range(len(approx)):
			
		cont = False
		for m in range(n):
			dist = cv2.norm(approx[n]-approx[m])
			#print approx[n], apporx[m], dist
			#raw_input()
			if dist < (30/SCALE):
				cont = True
				break
		if cont == True:
			continue
		else:
			point_cnt += 1
			# if point_cnt > 4:
				# continue
			center = (approx[n][0][0],approx[n][0][1])
			cv2.circle(img, center, 2, (0,0,255), 2)
			square.append( [float(i) for i in center])
			# cv2.imshow('draw',drawing)
			# cv2.waitKey(0)
	if len(square) < 4:
		return None, False

	# print "Before: ", square
	square = sortSquare(square[:4])
	# print "Sort: ", square

	cv2.imshow('img',img)
	# cv2.waitKey(0)
	# Find Homography for Appoximated PolyDP
	transformed = np.zeros((H_SIZE, H_SIZE), dtype=np.uint8)
	dst = [(0.0,0.0),(0.0,H_SIZE-1),(H_SIZE-1,0.0),(H_SIZE-1,H_SIZE-1)]
	h,_ = cv2.findHomography(np.array(square), np.array(dst), 0)
	#print h

	# Warp Perspective Transformation	
	warp = cv2.warpPerspective(img_2, h, transformed.shape)
	cv2.imshow('warp',warp)
	# cv2.waitKey(0)
	return warp, True
	

def crop_cifar_image(img):

	img2 = copy.copy(img)
	# cv2.imshow('img2',img2)
	# cv2.waitKey(0)
	crops = []

	for i in range(16):
		for j in range(16):
			cv2.rectangle(img2, (32*i,32*j), (32*(i+1),32*(j+1)), (0,0,255), 2)
			crops.append(img[32*i:32*(i+1),32*j:32*(j+1)])
	
	cv2.imshow('warp',img2)
	cv2.waitKey(0)

	return crops

if __name__ == '__main__':
	
	cap = cv2.VideoCapture('../img/cifar.mp4')
	cv2.namedWindow('img', cv2.CV_WINDOW_AUTOSIZE)
	cv2.namedWindow('warp', cv2.CV_WINDOW_AUTOSIZE)
	cv2.moveWindow("img", 10, 100);
	cv2.moveWindow("warp", 330, 100);

	while True:
		valid, img = cap.read()

		if not valid:
			break

		cv2.imshow('img', img)
		puzzle, valid = locate_puzzle(img)
		imgs = crop_cifar_image(puzzle)
		cv2.waitKey(0)
		# key = cv2.waitKey(50)
		# if key == 27:
			# break



	# img = cv2.imread('../img/cifar5.jpg', cv2.CV_LOAD_IMAGE_COLOR)
	# puzzle = locate_puzzle(img)
	# imgs = crop_cifar_image(puzzle)
