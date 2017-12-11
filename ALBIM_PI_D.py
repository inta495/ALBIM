# ALBIM
# Author Wattanakorn Intanon
# Maintainer Wattanakorn Intanon
# License GPL
# Email inta495@gmail.com
# 
#
# Program Thread
# 1. Main
# |-------	2. img_cap
# |-------	3. img_process
# |-------	4. multiwii_io

from CapCAM import CapCAM
from shapeDetector import ShapeDetector
from shapeDetector import BlueDetector
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import multiprocessing
import numpy as np  #parallelize mathematics Algorithm library 
from pyMultiWii import MultiWii # MultiWii Serial Protocal Created by Altex
import sys # i/o command
import os 
import socket # for UDP protocal for sending data to Ground control station
import logging
import csv
import io
import math
#define global
UDP_IP = ""
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
#sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(.02)

width = 320
height = 240
half_width = width/2
half_height = height/2
#define MuliWii Object
board = MultiWii("/dev/ttyUSB0")
#RC AUX condition
HIGH = 1750
LOW  = 1250
# log
logger = multiprocessing.log_to_stderr()
logger.setLevel(multiprocessing.SUBDEBUG)
#start time
start_time = time.time()
loop_time = 0.01

## function ##
def limit(val , minn , maxn):
	return max(min(maxn , val) , minn)
	pass

## Thread ##
def img_cap(pipe_1):
	camera = PiCamera()
	camera.iso = 100
	camera.resolution = ( width , height)
	camera.framerate = 15
	rawCapture = PiRGBArray(camera, size=( width , height))
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		img = frame.array
		if img is not None :
			pipe_1.send(img)
			pass
		rawCapture.truncate(0)	
	pass

def img_process(pipe_1,pipe_2):
	bl = BlueDetector()
	sd = ShapeDetector()
	count = 1
	while True:
		img = pipe_1.recv()
		mask = bl.detect(img,[100,60,50],[130,255,255])
		_ , cnts , hierarchy = cv2.findContours( mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
		shape = filter(lambda contour : sd.detect(contour , width , height) == 1 , cnts)
		M = map(lambda contour : cv2.moments(contour) if (cv2.contourArea(contour) > 0.0001*width*height) else None  , shape)
		sys.stdout.write("\033[K") # Clear to the end of line
		M = filter(lambda x : x is not None , M)
		cX = map(lambda moment : int((moment["m10"] / moment["m00"])) , M)
		sys.stdout.write("\033[K") # Clear to the end of line
		cY = map(lambda moment : int((moment["m01"] / moment["m00"])) , M)
		sys.stdout.write("\033[K") # Clear to the end of line
		if len(cX) is 0 :
			cXX = -1
			cYY = -1
			pass
		else :
			cXX = cX[0]
			cYY = cY[0]
			pass		
		pipe_2.send([cXX , cYY , 0])		
		pass
	pass

def multiwii_io(pipe_1,pipe_2):
	global loop_time
	global board
	last_time = 0
	current = 0
	timestr = time.strftime("%Y%m%d-%H%M%S")	
	logging.basicConfig(level = logging.DEBUG)
	multiwii_io_log = logging.getLogger(__name__)
	multiwii_io_log.info('Start reading data')
	multiwii_io_handler = logging.FileHandler('MW_log'+timestr+'.log',mode = 'w')
	multiwii_io_handler.setLevel(logging.DEBUG)
	multiwii_io_log.addHandler(multiwii_io_handler)
	# PID gain for position control
	dt = 0
	P_roll = 0.074 #0.034
	I_roll = 0.01 #0.015
	D_roll = 0.361
	P_pitch = 0.0607 #0.05 #0.054
	I_pitch = 0.01 #0.015
	D_pitch = 0.296 #0.3 
	p_x = 0
	p_y = 0
	i_x = 0
	i_y = 0
	d_x = 0
	d_y = 0
	###############################
	multiwii_io_log.debug(" Roll PID :"+str(P_roll)+" , "+str(I_roll)+" , "+str(D_roll))
	multiwii_io_log.debug("Pitch PID :"+str(P_pitch)+" , "+str(I_pitch)+" , "+str(D_pitch))
	multiwii_io_log.debug("		count : Time : Message :")
	counter = 0
	alt_command = 0	
	target_point = [-1,-1,-1]
	target_X = -1
	target_Y = -1
	dist_x = 0
	dist_y = 0
	true_X = 0
	true_Y = 0
	#check Alt Hold mode
	Alt_Hold_Mode = 0
	check_Alt_Hold_Mode = 0
	#check Forward mode
	Forward_Mode = 0
	Forward_start_time = 0
	Forward_time = 0
	#check Pos Hold mode
	Pos_Hold_Mode = 0
	#check Landing mode
	Landing_counter = 0	
	Landing_Mode = 0
	command_throttle = 0
	#Disarm
	Disarm_start_time = 0
	finish_time = 0
	#######
	#######

	Mode = 0
	# PI-D controller
	check_D = 0
	last_x = 0
	last_y = 0

	time_log = "None"
	message = "None"
	RC_COMMAND_log = "None"
	mode_log = "None"
	TARGET_log = "None"
	MODE_NOW = "None"
	POS_COM = "None"
	DEBUG_PID = "None"
	com_roll = 0
	com_pitch = 0
	trim_roll = 1500
	trim_pitch = 1500
	board.getData(MultiWii.RC)
	trim_roll = limit(board.rcChannels['roll'],1470,1530)
	trim_pitch = limit(board.rcChannels['pitch'],1470,1530)
	command_roll = trim_roll 
	command_pitch = trim_pitch
	command_yaw = 0
	command_Altitude_hold = 0
	multiwii_io_log.debug("TRIM :"+str(trim_roll)+" , "+str(trim_pitch) )

	client_message = ""
	address = ""
	if True:
		while True:
			if True:
				last_time = current
				current = time.time()
				com_roll = 0
				com_pitch = 0
				elapsed = 0
				dist_x = 0
				dist_y = 0				
				#TARGET
				if pipe_1.poll():
					target_point = pipe_1.recv()
					pass
				target_X = target_point[0] 
				target_Y = target_point[1]
				target_Area = target_point[2]
		 		#Reading Data from Multiwii		 		
				board.getData(MultiWii.MOTOR)						
				board.getData(MultiWii.RC)        # [roll , pitch , yaw , throttle , AUX1 , AUX2 , AUX3 , AUX4 , elapsed , timestamp] , 8*2 Byte
				board.getData(MultiWii.ATTITUDE)
				board.getData(MultiWii.ALTITUDE)  # [altitude , elapsed]
				#board.getData(MultiWii.RAW_IMU)
				board.getData(MultiWii.STATUS)
				board.getData(MultiWii.RC_COMMAND)
				board.getData(MultiWii.DEBUG)
				## SWITCH ##
				############
				############
				if Mode < 5:												
					if board.rcChannels['AUX2'] >= HIGH :
						Alt_Hold_Mode = 1
						if board.rcChannels['AUX3'] >= HIGH :
							Forward_Mode = 1
							pass
						else:
							Forward_Mode = 0
							Pos_Hold_Mode = 0
							Landing_Mode = 0
							pass
						pass
					else:
						Alt_Hold_Mode = 0
						Forward_Mode = 0
						Pos_Hold_Mode = 0
						Landing_Mode = 0
						pass
					pass

				## Controller ##
				################
				################
				if Pos_Hold_Mode == 1:
					# Landing Mode
					if Landing_Mode == 1:
						if Mode == 6:
							#Disarm
							command_throttle = 1000
							command_Altitude_hold = 1000
							command_yaw = 1000
							if current - finish_time > 5:
								print "Finish Mission"
								print "Reset ALL RC stick for safety..."
								while True:									
									pass
								pass
							pass
						elif Mode == 5:							
							command_throttle = 1000
							command_Altitude_hold = 1000
							if current - Disarm_start_time > 5: # wait
								finish_time = current
								Mode = 6
								pass
							pass		
						else:
							Mode = 4
							landing_open = 1
							if landing_open == 1 :
								if float(board.altitude['altitude']) < 0.05 :
									command_throttle = 1160
									command_Altitude_hold = 1000
									Mode = 5 #landed
									Disarm_start_time = current																		
									pass
								elif float(board.altitude['altitude']) <= 1.50:
									command_throttle = 1360
									pass
								else:
									data = [30]
									board.sendCMD(2,MultiWii.SET_ALTITUDE,data)								
									pass
								pass
							pass
						pass
					else:
						Mode = 3
						pass
					##############
					##############
					if target_X > -1 and target_Y > -1 :						
						dist_x = half_width  - target_X #set point = 160
						dist_y = (half_height-10) - target_Y #set point = 120

						correction_gain = 0.7
						dist_x = dist_x - correction_gain*(160*math.tan(float(board.attitude['angx'])*math.pi/180) / 0.477)
						dist_y = dist_y + correction_gain*(120*math.tan(float(board.attitude['angy'])*math.pi/180) / 0.378)
						
						#time						
						dt = current - last_time

						# P
						p_x = dist_x
						p_y = dist_y						
						# I
						i_x = limit(i_x + dt*dist_x,-800,800)
						i_y = limit(i_y + dt*dist_y,-800,800)						
						# D
						if check_D == 0:
							check_D = 1
							d_x = 0
							d_y = 0
							last_x = dist_x
							last_y = dist_y
							pass
						else:
							alpha = 0.17
							d_x = (1-alpha)*d_x + (alpha)*((dist_x - last_x)/dt)
							d_y = (1-alpha)*d_y + (alpha)*((dist_y - last_y)/dt)
							last_x = dist_x
							last_y = dist_y
							pass

						com_roll = P_roll*p_x + I_roll*i_x + D_roll*d_x
						com_pitch = P_pitch*p_y + I_pitch*i_y + D_pitch*d_y

						#effect of altitude
						if Mode >= 4:
							com_roll  = limit(float(board.altitude['altitude']), 0.3 , 1.5)*com_roll
							com_pitch = limit(float(board.altitude['altitude']), 0.3 , 1.5)*com_pitch
							pass

						command_roll = int(limit(trim_roll+com_roll , trim_roll-40 , trim_roll+40))
						#reversed -
						command_pitch = int(limit(trim_pitch-com_pitch , trim_pitch-40 , trim_pitch+40))

						#check for landing
						sq_radius = dist_x*dist_x + dist_y*dist_y
						radius = 50
						desired_sq_radius = radius*radius

						if Landing_counter > 10 :
							Landing_Mode = 1
							pass
						elif sq_radius < desired_sq_radius :
							Landing_counter = Landing_counter + dt							
							pass
						else:
							Landing_counter = 0
							command_throttle = 0
							pass
						pass

						if Mode == 5:
							command_roll = 0
							command_pitch = 0
							pass
						data = [ command_roll , command_pitch , command_yaw , command_throttle ,0,command_Altitude_hold,0,0]
						board.sendCMD(16,MultiWii.SET_RAW_RC,data)
						pass
					else:
						if Mode >= 5:
							command_roll = 0
							command_pitch = 0
							data = [ command_roll , command_pitch , command_yaw , command_throttle ,0,command_Altitude_hold,0,0]
							board.sendCMD(16,MultiWii.SET_RAW_RC,data)
							pass
						else:
							check_D = 0
							command_roll = int(limit((0.99*command_roll + 0.01*trim_roll) , trim_roll-40 , trim_roll+40))
							command_pitch = int(limit((0.99*command_pitch + 0.01*trim_pitch) , trim_pitch-40 , trim_pitch+40))
							data = [ command_roll , command_pitch , command_yaw , command_throttle ,0,command_Altitude_hold,0,0] 
							board.sendCMD(16,MultiWii.SET_RAW_RC,data)
							pass
						p_x = 0
						p_y = 0
						i_x = 0.99*i_x
						i_y = 0.99*i_y
						d_x = 0
						d_y = 0
						dist_x = 0
						dist_y = 0
						pass
					pass
				elif Forward_Mode == 1:
					#trim_roll = limit(board.rcChannels['roll'],1490,1510)
					#trim_pitch = limit(board.rcChannels['pitch'],1490,1510)					
					Mode = 2
					#check_Alt_Hold_Mode = 0
					if target_X > -1 and target_Y > -1:
						Pos_Hold_Mode = 1
						pass
					else:
						if Forward_start_time == 0:
							Forward_start_time = current
							pass
						Forward_time = current
						if Forward_time - Forward_start_time > 4 :
							data = [ 0 , trim_pitch + 3 ,0,0 ,0,0,0,0]
							pass
						elif Forward_time - Forward_start_time > 1:
							data = [ 0 , trim_pitch + 6 ,0,0 ,0,0,0,0]
							pass
						else:
						 	data = [ 0 , trim_pitch + 9 ,0,0 ,0,0,0,0]
						 	pass
						board.sendCMD(16,MultiWii.SET_RAW_RC,data)
						pass
					pass
				elif Alt_Hold_Mode == 1:
					Mode = 1
					if check_Alt_Hold_Mode == 0:
						data = [100]
						board.sendCMD(2,MultiWii.SET_ALTITUDE,data)
						check_Alt_Hold_Mode = 1						
						pass															
					p_x = 0
					p_y = 0
					i_x = 0.99*i_x
					i_y = 0.99*i_y
					d_x = 0
					d_y = 0
					Landing_counter = 0
					command_throttle = 0
					pass
				else:
					Mode = 0
					check_Alt_Hold_Mode = 0
					Landing_counter = 0
					command_throttle = 0
					pass
				#message for logging

				#time stamp log				
				time_log = "{:+8.4f}".format(current - start_time)				
				#data log
				message = "{:+4.2f}, {:+4.2f}, {:+4.2f}, {:+4.2f},".format(float(board.attitude['angx']) , float(board.attitude['angy']) , float(board.attitude['heading']) , float(board.altitude['altitude']))
				#RC_log = "{:+4d}, {:+4d}, {:+4d}, {:+4d}, {:+4d}, {:+4d}, {:+4d}, {:+4d}".format(board.rcChannels['roll'],board.rcChannels['pitch'],board.rcChannels['yaw'],board.rcChannels['throttle'],board.rcChannels['AUX1'],board.rcChannels['AUX2'],board.rcChannels['AUX3'],board.rcChannels['AUX4'])	
				RC_log = "{:+4d}, {:+4d}, {:+4d}".format(board.rcChannels['AUX2'],board.rcChannels['AUX3'],board.rcChannels['throttle'])
				RC_COMMAND_log = "{:+4d}, {:+4d}, {:+4d}, {:+4d}".format(board.rcCommand['roll'],board.rcCommand['pitch'],board.rcCommand['yaw'],board.rcCommand['throttle'])
				mode_log = "{:+4d}".format(board.status['flag'])
				MOTOR_log = "{:+4d}, {:+4d}, {:+4d}, {:+4d}".format(board.motor['m1'],board.motor['m2'],board.motor['m3'],board.motor['m4'])
				DEBUG_log = "{:+4d}, {:+4d}, {:+4d}, {:+4d}".format(board.debug['ch1'],board.debug['ch2'],board.debug['ch3'],board.debug['ch4'])
				TARGET_log = "tar: {:4.2f},{:4.2f},{:4.2f},{:4.2f},{:4.2f}".format(target_X,target_Y,dist_x,dist_y,target_Area)
				#RAW_IMU_log = "{:+4.2f} , {:+4.2f}".format(board.rawIMU['ax'],board.rawIMU['ay'])
				MODE_NOW = "mode: {:4d} {:4.2f}".format(Mode,Landing_counter)
				POS_COM = "{:+4.2f}, {:+4.2f}".format(com_roll,com_pitch)				
				DEBUG_PID = "{:+4.2f}, {:+4.2f}, {:+4.2f}, {:+4.2f}, {:+4.2f}, {:+4.2f}".format(p_x,i_x,d_x,p_y,i_y,d_y)				
				#Add to log file
				# SEND DATA via UDP ###############################################
				MESSAGE = "{:4.2f} {:4.2f} {:4.2f} {:4.2f} {:4.2f} {:4.2f} {:4d}".format( board.attitude['angx'] , board.attitude['angy'] , board.attitude['heading'] , board.altitude['altitude'] , dist_x , dist_y , Mode)
				pipe_2.send(MESSAGE)
				###################################################################
				
				#sock.settimeout(0.005)
				#client_message , address =  sock.recvfrom(2)
				#if client_message =="A":
				#	sock.sendto(MESSAGE , address)
				#	pass					
				multiwii_io_log.debug(" %8d , %s , %s , %s , %s , %s , %s , %s , %s , %s , %s",counter,time_log,message,RC_log,RC_COMMAND_log,TARGET_log,POS_COM,MODE_NOW,DEBUG_log,mode_log,DEBUG_PID)
				#multiwii_io_log.debug("%s",TARGET_log)
				multiwii_io_log.addHandler(multiwii_io_handler)
				#loop counter
				counter = counter+1
				#loop time
				while elapsed < loop_time:
					elapsed = time.time() - current
					pass
				pass
			pass
		pass
	#except Exception, e:
	#	sys.stdout("Error in MultiWii_io: %s", str(e))
	#	time.sleep(30)  
	pass

def udp_func(pipe_1):
	global sock
	MESSAGE = ""
	client_message = "B"
	address = ""
	while True:
		if pipe_1.poll():
			MESSAGE = pipe_1.recv()
			pass
		#print "ping"
		time.sleep(0.01)

		try:
			client_message , address =  sock.recvfrom(2)
			pass
		except Exception, e:
			client_message = "B"
			pass
		if client_message == "A":
			sock.sendto(MESSAGE , address)
			pass
		pass
	pass
###############################################################################################
###############################################################################################
###############################################################################################
if __name__ == '__main__' :
	#initialize pipe line 
	pipe_11 , pipe_12 = multiprocessing.Pipe(False) 
	pipe_21 , pipe_22 = multiprocessing.Pipe(False) # for image_processing -> image_show
	pipe_31 , pipe_32 = multiprocessing.Pipe(False)
 	#initialize thread
	img_cap_thread = multiprocessing.Process(target = img_cap , args = (pipe_12,))
	img_process_thread = multiprocessing.Process(target = img_process , args = (pipe_11,pipe_22))
	multiwii_io_thread = multiprocessing.Process(target = multiwii_io , args = (pipe_21,pipe_32))
	udp_func_thread = multiprocessing.Process(target = udp_func , args = (pipe_31,))
	#wait
	time.sleep(1)
	try:
		#start thread
		img_cap_thread.start()
		img_process_thread.start()
		multiwii_io_thread.start()
		udp_func_thread.start()
		time.sleep(10)
		#infinite loop
		while multiwii_io_thread.is_alive():
			pass
		#terminate thread
		img_cap_thread.terminate()
		img_process_thread.terminate()
		multiwii_io_thread.terminate()
		udp_func_thread.terminate()

		img_cap_thread.join()
		img_process_thread.join()
		multiwii_io_thread.join()
		udp_func_thread.join()
	except KeyboardInterrupt:
		img_cap_thread.terminate()
		img_process_thread.terminate()
		multiwii_io_thread.terminate()
		udp_func_thread.terminate()

		img_cap_thread.join()
		img_process_thread.join()
		multiwii_io_thread.join()
		udp_func_thread.join()
		print "ctrl + C three times"
		try:
			while True:
				pass
			pass
		except KeyboardInterrupt:
			print "ctrl + C two times"
			try:
				while True:
					pass
				pass
			except KeyboardInterrupt:
				print "ctrl + C one time"
				try:
					while True:
						pass
					pass
				except KeyboardInterrupt:
					pass
			pass
	pass
