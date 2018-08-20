#!/usr/bin/env python

from datetime import datetime

# ros packages
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Image

# minio clinet
# https://docs.minio.io/docs/python-client-api-reference
from minio import Minio
from minio.error import (ResponseError, BucketAlreadyOwnedByYou,
                         BucketAlreadyExists)

class ImgUploader(object):
	def __init__(self):
		'''
		Initialize
		- ros node initialization and parameter loading
		- create minio client instance and check bucket
		- start timer to save & upload image to the minio server
		'''
		rospy.loginfo('ros node initialize')
		rospy.init_node('img_uploader', anonymous=True)
		self._img_upload_period = rospy.get_param('~img_upload_period', 600)
		self._img_name = rospy.get_param('~img_name', '/home/ubuntu/img_to_be_upload.png')
		self._minio_ep = rospy.get_param('~server_ep', 'localhost:443')

		# minio python client
		rospy.loginfo('setup minio client')
		# remove scheme
		self._minio_ep = self._minio_ep.split('//')[1]
		rospy.loginfo(self._minio_ep)

		self.minioClient = Minio(self._minio_ep,
		              access_key='rapyuta123',
		              secret_key='rapyuta123',
		              secure=True)

		try:
		       self.minioClient.make_bucket("img")
		except BucketAlreadyOwnedByYou as err:
		       pass
		except BucketAlreadyExists as err:
		       pass
		except ResponseError as err:
		       raise

		rospy.loginfo('start timer')
		rospy.Timer(rospy.Duration(self._img_upload_period), self.callback)


	def callback(self, data):
		'''
		Timer callback function
		- call ros service to save image
		- upload saved image to the minio server
		'''
		rospy.loginfo('callback function')
		file_name = str(datetime.now())+'.png'

		# ros service call
		# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
		rospy.wait_for_service('/image_saver/save')
		try:
			save_img = rospy.ServiceProxy('/image_saver/save',  Empty)
			save_img()
			print 'Service_called'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		# Put an object to the cloud file server.
		try:
			rospy.loginfo('upload image'+file_name)
			import os, io
			rospy.loginfo(os.getcwd())
			rospy.loginfo(open(self._img_name, mode='rb'))
			self.minioClient.fput_object('img', file_name, self._img_name)
		except ResponseError as err:
			print(err)


if __name__ == '__main__':
	img = ImgUploader()
	rospy.spin()
