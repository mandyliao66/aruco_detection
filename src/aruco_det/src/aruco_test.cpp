/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
 

#include <iostream>
#include "aruco.h"
#include <cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/String.h"
#include <std_msgs/Int32.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>

cv::Mat srcImg, dstImg;
aruco::MarkerDetector MDetector;
std::vector<aruco::Marker> Markers;

std::string image_encoding="bgr8"; //options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16"
ros::Publisher pose_pub; 
geometry_msgs::Pose _Pose;
ros::Publisher marker_id_pub;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
	//ARUCO START	
	srcImg = cv_bridge::toCvShare(msg, image_encoding)->image;
    dstImg = srcImg;


	Eigen::Quaterniond quat2;
	_Pose.position.x = 0;
	_Pose.position.y = 0;
	_Pose.position.z = 0;
	 _Pose.orientation.w = 1.0;
	 _Pose.orientation.x = 0.0;
	 _Pose.orientation.y = 0.0;
	 _Pose.orientation.z = 0.0;
        
	//for each marker, draw info and its boundaries in the image
    for(auto m:MDetector.detect(srcImg)){

        m.draw(dstImg);
	    cv::Mat cam_mat=(cv::Mat_<float>(3, 3) << 2391.588100, 0.000000, 992.390258, 0.000000, 2419.429502, 752.570843, 0.000000, 0.000000, 1.000000);
	    cv::Mat dist_mat=(cv::Mat_<float>(1,5) << 0.049529, -0.468856, 0.002365, -0.008591, 0.000000);
	    m.calculateExtrinsics(0.05, cam_mat, dist_mat, 2);
		
		//projection 2371.783936 0.000000 978.791824 0.000000, 0.000000 2423.029297 754.196973 0.000000, 0.000000 0.000000 1.000000 0.000000

		std_msgs::Int32 marker_id_msg;
		marker_id_msg.data = m.id;
		marker_id_pub.publish(marker_id_msg);
		//geometry_msgs::PoseArray pose_publishing;
		//geometry_msgs::Pose _Pose;
		
		cv::Mat R2(3, 3, CV_64FC1);
		cv::Rodrigues(m.Rvec, R2);
		Eigen::Matrix3d eigMat2;
		cv::cv2eigen(R2, eigMat2);
		quat2 = Eigen::Quaterniond(eigMat2); // Use the existing variable and assign a new value
    	quat2.normalize();
		tf::quaternionEigenToMsg(quat2 , _Pose.orientation);
		
		_Pose.position.x = m.Tvec.at<float>(0);
		_Pose.position.y = m.Tvec.at<float>(1);
		_Pose.position.z = m.Tvec.at<float>(2);
	}
		
            pose_pub.publish(_Pose);
			ros::spinOnce();

        


        cv::imshow("out",dstImg);
        cv::waitKey(1);//wait for key to be pressed
	}

		catch (cv_bridge::Exception& e){
	  ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "aruco_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("/flir_adk/image_raw", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe("/camera/image_color", 1, imageCallback);
    	MDetector.setDictionary("ARUCO_MIP_36h12");
	
	marker_id_pub = nh.advertise<std_msgs::Int32>("aruco_marker_id", 100);
	pose_pub = nh.advertise<geometry_msgs::Pose>("aruco_detection", 100);

	ros::Rate loop_rate(10);

	

    //MDetector.setDetectionMode(aruco::DetectionMode::DM_FAST,0.02);
    	ros::spin();
}