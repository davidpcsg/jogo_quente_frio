#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
# rospy for the subscriber

import rospy
from std_msgs.msg import String
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2


#wisard
from wisard import *
from string import *
from pyppm import *
import sys
import os.path
#------


   

#*********************************************************************
#wizzard aplication class 
#*********************************************************************
# the application class
class wiz:
    # init: receives the root widget
    def __init__(self):

        # general widget creation & variables
        self.n_classes = 4
        self.n_bits = 10
	self.n_samples = 22
        self.types = [("PPM", ".ppm")]
        self.net = self.x = self.y = None
	#
	self.imgTitClass1 = cv2.imread('/home/labvad/Imagens/imgClass1.jpg',flags=cv2.IMREAD_COLOR)
	self.imgTitClass2 = cv2.imread('/home/labvad/Imagens/imgClass2.jpg',flags=cv2.IMREAD_COLOR)
	self.imgTitClass3 = cv2.imread('/home/labvad/Imagens/imgClass3.jpg',flags=cv2.IMREAD_COLOR)
	self.imgTitClass4 = cv2.imread('/home/labvad/Imagens/imgClass4.jpg',flags=cv2.IMREAD_COLOR)
        self.imgBlack = cv2.imread('/home/labvad/Imagens/black.jpg',flags=cv2.IMREAD_COLOR)
		
    def switch_class_name(self,class_identifier):
        switcher = {
            1: "Estreito",
            2: "Normal",
            3: "Largo",
            4: "Parede",
        }
        return switcher.get(class_identifier)
	
    def switch_class_image(self,class_identifier):
        switcher = {
            1: self.imgTitClass1,
            2: self.imgTitClass2,
            3: self.imgTitClass3,
            4: self.imgTitClass4,
        }
        return switcher.get(class_identifier)

    def train_sample(self):

        for i in range(1,(self.n_classes+1)):
            self.train_class = i
            for j in range(1, self.n_samples+1):
                file = '/home/labvad/Imagens/classe'+str(i)+'_treino/picture' + str(j)+'_.ppm'
                image = load_image(file)
                x = len(image)
                y = len(image[0])

                #  check if the image has the same size of the last one
                if not self.x:
                    self.x = x
                    self.y = y
                elif self.x != x or self.y != y:
                    print("Image size inconsistent with the WISARD!\n")
                    return None

                # checks if the wisard was already instantiated
                if not self.net:
                    self.net = wisard(self.n_classes, self.n_bits, x, y)
                    print("Created WISARD net, input size=%dx%d.\n" % (x, y))
                    print("Each discriminator has %d RAM neurons.\n" % (self.net.n_ram))

                self.net.train(self.train_class - 1, image, 0, 0)

                # print feedback to user
                print("Class %d <- \"%s\" ." % (self.train_class, file))


    def rec_sample(self, file):
        image = load_image(file)
        x = len(image)
        y = len(image[0])

        #  check if the image has the same size of the last one
        if not self.x:
            self.x = x
            self.y = y
        elif self.x != x or self.y != y:
            print("Image size inconsistent with the WISARD!\n")
            return None

        # recognise the image and print a report
        output = self.net.stats(image, 0, 0)
        print("\nWISARD answer for \"%s\":" % file)
        print("Class   Sum  Recog.")
        for i in xrange(self.n_classes):
            print("%5d %5d %6.1f%%" % (i + 1, output[i][0], output[i][1]))
        
	storeClassNumber = 1
        storePercent= 0
        for i in xrange(self.n_classes):
            if storePercent < output[i][1]:
                storePercent = output[i][1]
                storeClassNumber = i+1
 	cv2.namedWindow('image',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', 320,240)   
	class_name = self.switch_class_name (storeClassNumber)	
	print(class_name)
        print("%5d" % storePercent)		
        cv2.imshow('image',self.switch_class_image(storeClassNumber))
	cv2.waitKey(3)


# wiz class end
#*********************************************************************
#wizzard aplication class end
#*********************************************************************


# Instantiate CvBridge
bridge = CvBridge()



#def image_callback(msg):
#    print("Received an image!")
#    try:
#        # Convert your ROS Image message to OpenCV2
#        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
#    except CvBridgeError, e:
#        print(e)
#    else:
#        # Save your OpenCV2 image as a jpeg 
#        cv2.imwrite('camera_image.jpeg', cv2_img)

def msg_img_to_rec_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    image_source = data.data
    file = image_source
    wiz.rec_sample(a,file)
#    print(image_source)
		
def main():
	
    rospy.init_node('pic_to_recognize')
    # Define your image topic
#    image_topic = "/cameras/left_hand_camera/image"
    # Set up your subscriber and define its callback
#    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber("pic_to_rec", String, msg_img_to_rec_callback)
    # Spin until ctrl + c
    rospy.spin()
    

if __name__ == '__main__':
    a = wiz()
    wiz.train_sample(a)	
    main()
