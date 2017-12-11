# Shape Detector
# Author Wattanakorn Intanon
# Maintainer Wattanakorn Intanon
# License GPL
# Email inta495@gmail.com
# 


import cv2
import numpy as np
class ShapeDetector:
	"""docstring for ClassName"""
	def __init__(self):
		pass
	def detect(sefl, c,width,height):
		shape = "unidentified"
		peri = cv2.arcLength(c , True)
		approx = cv2.approxPolyDP( c , 0.03 * peri , True)
		#print approx		
		if len(approx) == 4:
			for j in range(0,4):
				#print approx[j][0][0]
				if approx[j][0][0] >= width -2 or approx[j][0][0] <= 2:
					return 2
					pass
				if approx[j][0][1] >= height -2 or approx[j][0][1] <= 2:
					return 2
					pass 
				pass
			#print "#"
			return 1
		else:
			# other shape return 2
			return 2

class BlueDetector:
	def __init__(self):
		pass
	def detect(self , img , lower_bound , upper_bound):
		hsv = cv2.cvtColor( img , cv2.COLOR_BGR2HSV)
		# Range setup
		lower_blue = np.array(lower_bound)
		upper_blue = np.array(upper_bound)

		#mark
		#return binary image
		return cv2.inRange( hsv , lower_blue , upper_blue)




		

		
