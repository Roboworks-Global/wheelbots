#!/usr/bin/env python3
from __future__ import division
import message_filters
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt
import cv_bridge

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

np.seterr(all='raise')  
displayImage=False
plt.close('all')

class VisualTracker(Node):
	def __init__(self):
		super().__init__('visualtracker')
		qos = QoSProfile(depth=10)
		self.bridge = cv_bridge.CvBridge()
		#self.tmp_list = self.get_parameter('~targetred/upper').value
		self.targetUpper = np.array([0, 50, 50])
		self.targetLower = np.array([180, 255, 255])
		
		#self.pictureHeight= self.get_parameter('~pictureDimensions/pictureHeight')
		#self.pictureWidth = self.get_parameter('~pictureDimensions/pictureWidth')
		#vertAngle =self.get_parameter('~pictureDimensions/verticalAngle')
		#horizontalAngle =  self.get_parameter('~pictureDimensions/horizontalAngle')
		
		self.pictureHeight= 480
		self.pictureWidth = 640
		vertAngle =0.43196898986859655
		horizontalAngle =  0.5235987755982988
		
		# precompute tangens since thats all we need anyways:
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)	
		self.lastPoCsition =None
		# one callback that deals with depth and rgb at the same time
		im_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
		dep_sub = message_filters.Subscriber(self,Image,'/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
		queue_size = 30
 
		self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub],queue_size,0.05)
		self.timeSynchronizer.registerCallback(self.trackObject)
		self.positionPublisher = self.create_publisher( PositionMsg,'/object_tracker/current_position', qos)
		self.posMsg=PositionMsg()

	
	def trackObject(self, image_data, depth_data):
		if(image_data.encoding != 'rgb8'):
			raise ValueError('image is not rgb8 as expected')
		#convert both images to numpy arrays
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
		depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1")	
		if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))
		# blure a little and convert to HSV color space
		#blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)	
		# select all the pixels that are in the range specified by the target
		org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)	

		# clean that up a little, the iterations are pretty much arbitrary
		mask = cv2.erode(org_mask, None, iterations=4)		
		#self.get_logger().warn('no position found')
		#mask = cv2.dilate(mask,None, iterations=3)
		# find contours of the object
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

		newPos = None #if no contour at all was found the last position will again be set to none
		# lets you display the image for debuging. Not in realtime though
		if displayImage:
			backConverted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
			#cv2.imshow('frame', backConverted)
			#cv2.waitKey(0)
			#print(backConverted)			
			plt.figure()
			plt.subplot(2,2,1)
			plt.imshow(frame)
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,2)
			plt.imshow(org_mask, cmap='gray', interpolation = 'bicubic')			
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,3)			
			plt.imshow(mask, cmap='gray', interpolation = 'bicubic')
			plt.xticks([]),plt.yticks([])
			plt.show()
			rclpy.sleep(0.2)
		# go threw all the contours. starting with the bigest one
		for contour in sorted(contours, key=cv2.contourArea, reverse=True):
			# get position of object for this contour

			pos = self.analyseContour(contour, depthFrame)

			# if it's the first one we found it will be the fall back for the next scan if we don't find a plausible one
			if newPos is None:
				newPos = pos
			# check if the position is plausible
			#if self.checkPosPlausible(pos):
			self.lastPosition = pos
			self.publishPosition(pos)
			return
		
		self.lastPosition = newPos #we didn't find a plossible last position, so we just save the biggest contour 
		# and publish warnings
		#self.get_logger().warn('no position found')
		#self.infoPublisher.publish(StringMsg('visual:nothing found'))
		
	def publishPosition(self, pos):
		# calculate the angles from the raw position
		self.posMsg.angle_x = self.calculateAngleX(pos)
		print(self.posMsg.angle_x)
		self.posMsg.angle_y = self.calculateAngleY(pos)
		self.posMsg.distance=float(pos[1])
		# publish the position (angleX, angleY, distance)

		self.positionPublisher.publish(self.posMsg)

	def checkPosPlausible(self, pos):
		'''Checks if a position is plausible. i.e. close enough to the last one.'''

		# for the first scan we cant tell
		if self.lastPosition is None:
			return False

		# unpack positions
		((centerX, centerY), dist)=pos	
		((PcenterX, PcenterY), Pdist)=self.lastPosition
		
		if np.isnan(dist):
			return False

		# distance changed to much
		if abs(dist-Pdist)>0.5:
			return False

		# location changed to much (5 is arbitrary)
		if abs(centerX-PcenterX)>(self.pictureWidth /5):
			return False

		if abs(centerY-PcenterY)>(self.pictureHeight/5):
			return False
		
		return True
			
		
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''
		centerX = pos[0][0]
		displacement = 2*centerX/self.pictureWidth-1
		angle = -1*np.arctan(displacement*self.tanHorizontal)
		return angle

	def calculateAngleY(self, pos):
		centerY = pos[0][1]
		displacement = 2*centerY/self.pictureHeight-1
		angle = -1*np.arctan(displacement*self.tanVertical)
		return angle
	
	def analyseContour(self, contour, depthFrame):
		'''Calculates the centers coordinates and distance for a given contour

		Args:
			contour (opencv contour): contour of the object
			depthFrame (numpy array): the depth image
		
		Returns:
			centerX, centerY (doubles): center coordinates
			averageDistance : distance of the object
		'''
		# get a rectangle that completely contains the object
		centerRaw, size, rotation = cv2.minAreaRect(contour)

		# get the center of that rounded to ints (so we can index the image)
		center = np.round(centerRaw).astype(int)

		# find out how far we can go in x/y direction without leaving the object (min of the extension of the bounding rectangle/2 (here 3 for safety)) 
		minSize = int(min(size)/3)

		# get all the depth points within this area (that is within the object)
		depthObject = depthFrame[(center[1]-minSize):(center[1]+minSize), (center[0]-minSize):(center[0]+minSize)]

		# get the average of all valid points (average to have a more reliable distance measure)
		depthArray = depthObject[~np.isnan(depthObject)]
		#averageDistance = np.mean(depthArray)
		
		if len(depthArray) == 0:
			self.get_logger().warn('empty depth array. all depth values are nan')
			averageDistance=0
		else:
			averageDistance = np.mean(depthArray)
			
		if(averageDistance>400 or averageDistance<3000):
			pass
		else:
			averageDistance=400


		return (centerRaw, averageDistance)
	
def main(args=None):
    print('visual_tracker')
    rclpy.init(args=args)
    visualTracker = VisualTracker()
    
    print('visualTracker init done')
    try:
        rclpy.spin(visualTracker)
    finally:
        visualTracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    


