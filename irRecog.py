import cv2
import numpy as np
import socket
import json

#preset the camera parameters for recognition. (Helpful for uniform scaling)
frameWidth = 640
frameHeight = 480
#start the camera
cap = cv2.VideoCapture(1)
#cap = cv2.VideoCapture('onboard.mp4')
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(15,150)

#setup socet to send coordinates:  irRecog.py -->  ROS Script
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# to save the video from onboard camera before and after processing.
v = cv2.VideoWriter('onboard.mp4',cv2.VideoWriter_fourcc('F','M','P','4'), 30,(640,480))
v2 = cv2.VideoWriter('mask.mp4',cv2.VideoWriter_fourcc('F','M','P','4'), 30,(640,480))
#function to send data to desired IP and PORT (for ROS to intercept)
def publishString(msg):
	sock.sendto(msg, ("127.0.0.1", 4041))
#Function to get countours in the image
def getContours(img):
	#cv2.imshow('x1',img)
	img = np.uint8(img*255)
	#print(img)
	#imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(img, 127, 255, 0)
	#print(cv2.CAP_PROP_FPS)
	contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	#initialize x,y,h,w
	x,y,w,h = 0,0,0,0
	#print(len(contours))
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area>1:
			cv2.drawContours(img, cnt, -1, (255, 0, 0), 3)
			peri = cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,0.02*peri,True)
			x, y, w, h = cv2.boundingRect(approx)
	cv2.imshow('x',img)
	#return center of the detection box x,y
	return x+w/2,y+h/2
#function to morph image to detect Infrared
def imageMorph(img):
	#HSV
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#YCbCr
	imgYCC = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
	v = imgHSV[:,:,2]
	y = imgYCC[:,:,0]
	#cv2.imshow("v",v)
	v = v/255.0
	y = y/255.0
	#print(v)
	#cv2.imshow("v",v)
	imx = v * y
	imx = imx * v
	imx = imx * v
	imx = imx * v
	#imx = cv2.adaptiveThreshold(imx ,1,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
	imx = cv2.threshold(imx,0.5,1,cv2.THRESH_BINARY)
	#cv2.imshow("imx",imx[1])
	imx = imx[1]
	disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (19,19))
	dilation = cv2.dilate(imx,disk,iterations = 1)
	#print(dilation)
	#cv2.imshow("imx",dilation)
	return dilation

def getLocation(img):
	x,y=getContours(img)
	#cv2.imshow('getLoaction',img)
	msg = {
	"x" : float(x)/float(640),
	"y" : float(y)/float(480)
	}
	# x,y = center of the detection box
	print(msg)
	# publish x,y.
	publishString(str.encode(json.dumps(msg)))#"'x':{} , 'y':{}".format(x/640,y/480)))
	return x,y
	


while True:
    success, img = cap.read()
    if success:
		#imgResult = img.copy()
		cv2.imshow("Original", img)
		v.write(img)
		imx = imageMorph(img)
		x,y = getLocation(imx)
		cv2.circle(img,(x,y),15,[0,0,255],cv2.FILLED)
		cv2.imshow("Result", img)
		v2.write(img)
    if cv2.waitKey(1) and 0xFF == ord('q'):
    	v.release()
    	v2.release()
    	break
