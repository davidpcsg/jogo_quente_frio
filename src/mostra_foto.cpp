#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


void mostra_foto(sensor_msgs::ImageConstPtr imagem_pub)
{
  try
  {
	cv::namedWindow("view3",CV_WINDOW_NORMAL);	
    cv::startWindowThread();  
    cv::imshow("view3", cv_bridge::toCvShare(imagem_pub, "bgr8")->image);  
    cv::waitKey(10);
	sleep(3);
	cv::destroyWindow("view3");  
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imagem_pub->encoding.c_str());
   } 
  
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  mostra_foto(msg);
}


int main(int argc, char **argv)
{  
   ros::init(argc, argv, "mostra_foto");
   ros::NodeHandle nh;
	
   ros::Subscriber sub = nh.subscribe("foto", 1, imageCallback);
   ros::spin();

}
