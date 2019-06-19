#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
# rospy for the subscriber

import os
import rospy
from std_msgs.msg import String
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2


#wisard
import wisardpkg as wp
from string import *
from pyppm import *
import sys
import os.path
import vlc
#------

import time
   

#*********************************************************************
#wizzard aplication class 
#*********************************************************************
# the application class
class wiz:
    
    # init: receives the root widget
    def __init__(self):

        # general widget creation & variables
        self.n_classes = 3
        self.raiz = os.environ['RESOURCES_FOLDER'] + '/wisard'
        self.n_samples = 10
        self.types = [("PPM", ".ppm")]
        self.wsd = self.x = self.y = None
        
        #
        self.imgTitClass1 = cv2.imread(self.raiz + '/imgClass1.jpg',flags=cv2.IMREAD_COLOR)
        self.imgTitClass2 = cv2.imread(self.raiz +'/imgClass2.jpg',flags=cv2.IMREAD_COLOR)
        self.imgTitClass3 = cv2.imread(self.raiz +'/imgClass3.jpg',flags=cv2.IMREAD_COLOR)
        self.imgTitClass4 = cv2.imread(self.raiz +'/imgClass4.jpg',flags=cv2.IMREAD_COLOR)
        self.imgBlack = cv2.imread(self.raiz +'/black.jpg',flags=cv2.IMREAD_COLOR)
    
    def switch_class_image(self,class_identifier):
        switcher = {
            "quadrado": self.imgTitClass1,
            "triangulo": self.imgTitClass2,
            "circulo": self.imgTitClass3,
            "indefinido": self.imgTitClass4,
        }
        return switcher.get(class_identifier)

    def train_sample(self):

        classes = list()
        examples = list()
        addressSize = 10
        ignoreZero  = False
        verbose = True
        self.wsd = wp.Wisard(addressSize, ignoreZero=ignoreZero, verbose=verbose)
        
        for i in range(1,(self.n_classes+1)):
            self.train_class = i
            for j in range(1, self.n_samples+1):
                file = self.raiz +'/classe'+str(i)+'_treino/picture_' + str(j)+'.jpg'
                image_src = cv2.imread(file, 0)
                image_src = cv2.resize(image_src,(100, 100))
                image_dst = [(1 if e==255 else 0) for e in image_src.flatten()]
                            
                # print feedback to user
                print("Class %d <- \"%s\" ." % (self.train_class, file))
                
                train_class_label = ""
                if self.train_class == 1:
                    train_class_label = "quadrado"
                elif self.train_class == 2:
                    train_class_label = "triangulo"
                elif self.train_class == 3:
                    train_class_label = "circulo"
                else:
                    train_class_label = "indefinido"
                    
                classes.append(train_class_label)
                examples.append(image_dst)
                
        self.wsd.train(examples,classes)


    def rec_sample(self, file):
        image = cv2.imread(file)
        image = self.process_image(image)
        image = cv2.resize(image,(100, 100))
        cv2.imwrite(self.raiz + "/resized_image.bmp", image)
        image_bin = [(1 if e==255 else 0) for e in image.flatten()]
        
        # recognise the image and print a report
        out = self.wsd.classify([image_bin])
        
        print("wisard answer:")
        print(out[0])
        self.play_audio(out[0])
        storeClassNumber = 1
        storePercent= 0
        cv2.namedWindow('image',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', 320,240)   
        class_name = out[0]  
        print(class_name)     
        cv2.imshow('image', self.switch_class_image(class_name))
        cv2.waitKey(3)
    
    def play_audio(self, train_class_label):
        audio_to_play = ""
        if train_class_label == "quadrado":
            audio_to_play = os.environ['RESOURCES_FOLDER'] + '/audio/robot_quadrado.mp3'
        elif train_class_label == "triangulo":
            audio_to_play = os.environ['RESOURCES_FOLDER'] + '/audio/robot_triangulo.mp3'
        elif train_class_label == "circulo":
            audio_to_play = os.environ['RESOURCES_FOLDER'] + '/audio/robot_circulo.mp3'
        else:
            audio_to_play = os.environ['RESOURCES_FOLDER'] + '/audio/robot_indefinido.mp3'
        p = vlc.MediaPlayer("file://" + audio_to_play)
        p.play()
    
    def process_image(self, image):

       image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       _,image_thresh = cv2.threshold(image_gray,50,255,cv2.THRESH_BINARY)
       cv2.imwrite('image.bmp', image_thresh)
       inverted_image = cv2.imread('image.bmp')
       inverted_image[inverted_image == 255] = 1
       inverted_image[inverted_image == 0] = 255
       inverted_image[inverted_image == 1] = 0
       inverted_image_gray = cv2.cvtColor(inverted_image,cv2.COLOR_BGR2GRAY)
       ret,inverted_image_tresh = cv2.threshold(inverted_image_gray,127,255,0)
       cv2.imwrite('inverted_image.bmp', inverted_image_tresh)
       im2,contours, hierarchy = cv2.findContours(inverted_image_tresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
       cnt = contours[0]
       x,y,w,h = cv2.boundingRect(cnt)
       ret,inverted_image = cv2.threshold(inverted_image,0,255,cv2.THRESH_BINARY_INV)
       detected_image = inverted_image[y:y+h,x:x+w]
       detected_image = cv2.cvtColor(detected_image,cv2.COLOR_BGR2GRAY)
       _,detected_image = cv2.threshold(detected_image,50,255,cv2.THRESH_BINARY)
       return detected_image
        
# wiz class end
#*********************************************************************
#wizzard aplication class end
#*********************************************************************




# Instantiate CvBridge
bridge = CvBridge()

def msg_img_to_rec_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    wiz.rec_sample(a, data.data)
        
def main():
    
    rospy.init_node('pic_to_recognize')
    rospy.Subscriber("pic_to_rec", String, msg_img_to_rec_callback)
    rospy.spin()
    

if __name__ == '__main__':
    a = wiz()
    wiz.train_sample(a) 
    main()
