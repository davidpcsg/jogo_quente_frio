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

/*Modificado por Rubens Lacerda Queiroz */

#include "jogo_quente_frio/jogo_quente_frio_def.h"


JogoQuenteFrioDefinitions::JogoQuenteFrioDefinitions()
  : nh_priv_("~")
{ 
  //Init gazebo ros turtlebot3 node
  got_picture = false;	
  pic_count = 1;	
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
  
}

JogoQuenteFrioDefinitions::~JogoQuenteFrioDefinitions()
{

  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
  
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool JogoQuenteFrioDefinitions::init()
{
 		
  // initialize ROS parameter
  nh_.param("is_debug", is_debug_, is_debug_);
  std::string robot_model = nh_.param<std::string>("tb3_model", "");
	
  if (!robot_model.compare("burger"))
  {
    turning_radius_ = 0.08;
    rotate_angle_ = 50.0 * DEG2RAD;
    front_distance_limit_ = 0.3;
    side_distance_limit_  = 0.3;
  }
  else if (!robot_model.compare("waffle"))
  {
	printf("  waffle  ");
    turning_radius_ = 0.1435;
    rotate_angle_ = 40.0 * DEG2RAD;
    front_distance_limit_ = 0.3;
    side_distance_limit_  = 0.3;
  }
  ROS_INFO("robot_model : %s", robot_model.c_str());
  ROS_INFO("turning_radius_ : %lf", turning_radius_);
  ROS_INFO("front_distance_limit_ = %lf", front_distance_limit_);
  ROS_INFO("side_distance_limit_ = %lf", side_distance_limit_);

  // initialize variables
  right_joint_encoder_ = 0.0;
  priv_right_joint_encoder_ = 0.0;
  //-------	
  direcao_giro = 'D';	
  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  image_pub_ = nh_.advertise<sensor_msgs::Image>("foto", 1);	
  msg_pic_to_rec_pub_ = nh_.advertise<std_msgs::String>("pic_to_rec", 1);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &JogoQuenteFrioDefinitions::laserScanMsgCallBack, this);
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &JogoQuenteFrioDefinitions::jointStateMsgCallBack, this);	
  // ----
  cold_hot_sub_ = nh_.subscribe("temperatura", 10, &JogoQuenteFrioDefinitions::temperaturaCallback,this);
  image_sub_ = nh_.subscribe("/camera/rgb/image_raw/compressed", 1, &JogoQuenteFrioDefinitions::imageCallback,this);		
  // ---	 	
	
  return true;
}



void JogoQuenteFrioDefinitions::jointStateMsgCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{
  right_joint_encoder_ = msg->position.at(0); 
}

void JogoQuenteFrioDefinitions::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
 uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      direction_vector_[num] = msg->range_max;
	  //ROS_INFO("range_max: %f", msg->range_max);
    }
    else
    {
      direction_vector_[num] = msg->ranges.at(scan_angle[num]);
	  //ROS_INFO("(scan_angle[num] %f", msg->ranges.at(scan_angle[num]));	
    }
  } 
}

//---
void JogoQuenteFrioDefinitions::temperaturaCallback(const std_msgs::String::ConstPtr& msg)
	
{
 
    // ROS_INFO ("I heard: [%s]", msg->data.c_str());
	 if (("[%s]",msg->data) == "Q") {
		 temperatura = 'Q';
		// ROS_INFO ("Temperatura Quente");
	 } else if(("[%s]",msg->data) == "F") {
		 temperatura = 'F';
		 if (direcao_giro == 'D'){
			 direcao_giro = 'E';
		 } else if (direcao_giro == 'E'){
			 direcao_giro = 'D';
		 }	 
		 //ROS_INFO ("Temperatura Fria");
	 } else if(("[%s]",msg->data) == "S") {
		 temperatura = 'S';
		 //ROS_INFO ("Para");
	 } else if(("[%s]",msg->data) == "C") {
		 temperatura = 'C';
		 //ROS_INFO ("foto");		 		 
	 } else {
		 temperatura = 'Z';
	 }	 
	 
}

void  JogoQuenteFrioDefinitions::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
 try
  {
	  
	imagem = cv::imdecode(cv::Mat(msg->data),1);
	cv::imshow("view", imagem);  
    cv::waitKey(10);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert to image!");
   }   
}


//-----
	

void JogoQuenteFrioDefinitions::updatecommandVelocity(double linear, double angular)
{ 
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
  
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool JogoQuenteFrioDefinitions::controlLoop()
{
 static uint8_t turtlebot3_state_num = 0;

stringstream ss;
string str;
	
  
	if ( temperatura == 'Q') {
		// ROS_INFO ("Temperatura Quente");
 		if (((direction_vector_[CENTER] < front_distance_limit_) && (direction_vector_[CENTER] != 0.0 ))||
		   (direction_vector_[LEFT] < side_distance_limit_)	||
	       (direction_vector_[RIGHT] < side_distance_limit_)) {
            updatecommandVelocity(0.0, 0.0);
			got_picture = false;
		} else {	 	
			updatecommandVelocity(LINEAR_VELOCITY, 0.0);
			got_picture = false;
		}		
	 } else if (temperatura == 'F') {
             got_picture = false;
	         // ROS_INFO ("Temperatura Fria");	              
		      if (direcao_giro == 'D') { 
					  updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
				      turtlebot3_state_num = GET_TB3_DIRECTION;
			  } else if (direcao_giro == 'E'){
					  updatecommandVelocity(0.0, ANGULAR_VELOCITY);
		              turtlebot3_state_num = GET_TB3_DIRECTION;
			  }
		
	 } else if (temperatura == 'S'){
		      got_picture = false;
		     //  ROS_INFO ("Para");
		       updatecommandVelocity(0,0);
		
	 } else if (temperatura == 'C'){	  
		     if (got_picture == false ){
			   ss<<pic_count;
			   str = ss.str(); 
			   pic_count = pic_count + 1;	 
			   got_picture = true;	 
		       ROS_INFO ("Foto");	
							  
				 
			  Size size(320,266); 	
	          resize(imagem,imagem,size);	  
	          threshold( imagem, imagem, 100,255,THRESH_BINARY);	 
              cv::imwrite( "/home/labvad/Imagens/picture"+ str + "_.ppm", imagem );
			   //cv::imwrite( "picutre"+ str + "_.ppm", imagem ); 
	         }			 
	 }		
 

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  	
  ros::init(argc, argv, "salva_fotos");
	
  cv::namedWindow("view",CV_WINDOW_NORMAL);	
  cv::startWindowThread();		
	  	
 JogoQuenteFrioDefinitions JogoQuenteFrioDefinitions;
	
  ros::Rate loop_rate(125);

  while (ros::ok())
  {	     
    JogoQuenteFrioDefinitions.controlLoop();
    ros::spinOnce();  
    loop_rate.sleep(); 
  }
	
  cv::destroyWindow("view");		

  return 0;
}
