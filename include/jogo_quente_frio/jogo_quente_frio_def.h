/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD. 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

/*Modificado por Rubens Lacerda Queriroz */

#ifndef JOGO_QUENTE_FRIO_DEF_H_
#define JOGO_QUENTE_FRIO_DEF_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <math.h>
#include <limits.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <iostream>
using namespace cv;
using namespace std;

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2
#define LEFT10   3
#define RIGHT10  4

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

class JogoQuenteFrioDefinitions
{
 public:
  JogoQuenteFrioDefinitions();
  ~JogoQuenteFrioDefinitions();
  bool init();
  bool controlLoop(); 
 
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
 
  // imagem
   Mat imagem;
   std_msgs::String msg_pic_to_rec_;
 
  // ROS Parameters
  bool is_debug_;

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  //--
  
  ros::Publisher image_pub_;
  ros::Publisher msg_pic_to_rec_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber joint_state_sub_;
  //----
  ros::Subscriber cold_hot_sub_; 
  ros::Subscriber image_sub_; 
  //----
  //cv_brigde
  //cv::janela_imagem_;
  //-----
  char temperatura;
  char direcao_giro;
  bool got_picture;
  int  pic_count;
  //-----	
  
  double turning_radius_;
  double rotate_angle_;
  double front_distance_limit_;
  double side_distance_limit_;
  
  double direction_vector_[3] = {0.0, 0.0, 0.0};
  //double direction_vector_[5] = {0.0, 0.0, 0.0,0.0,0.0};

  double right_joint_encoder_;
  double priv_right_joint_encoder_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void jointStateMsgCallBack(const sensor_msgs::JointState::ConstPtr &msg); 
  //-----------
  void temperaturaCallback(const std_msgs::String::ConstPtr& msg);  
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);  
 
};
#endif // JOGO_QUENTE_FRIO_DEF_H_
