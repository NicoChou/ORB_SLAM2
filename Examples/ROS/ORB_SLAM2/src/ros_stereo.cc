/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void convertPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose);     //Nico
    void matrixToQuaternion(const cv::Mat& mat, Eigen::Quaternion<double>& quat);//Nico
    void grabTandpub(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    cv::Mat T_pub;
    cv::Mat Rwc_pub;
    cv::Mat twc_pub;
    Eigen::Quaternion<double> Q;
    geometry_msgs::Pose pose_pub;
    //nav_msgs::Path orbPath;
    nav_msgs::Odometry orbOdom;
    ros::Publisher orbPath_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();
    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    if(argv[3])
    {
        igb.do_rectify = true;

        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
    igb.orbPath_pub = nh.advertise<nav_msgs::Odometry>("orbslam_path",100);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        T_pub=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        //std::cout<<"T_pub : "<<T_pub.rowRange(0,3).col(3)<<std::endl;
        grabTandpub(msgLeft,msgRight);
    }
    else
    {
        T_pub=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        grabTandpub(msgLeft,msgRight);
    }

}

void ImageGrabber::grabTandpub(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    //T_pub=mpSLAM->GetT();
    Rwc_pub = T_pub.rowRange(0,3).colRange(0,3).t();
    //std::cout<<"R_pub : "<<R_pub<<std::endl;
    twc_pub = -Rwc_pub*T_pub.rowRange(0,3).col(3);
    std::cout<<"twc_pub : "<<twc_pub<<std::endl;
    //std::cout<<"t.at<double>(0):"<<t_pub.at<float>(0,0)<<" t.at<double>(1):"<<t_pub.at<float>(1,0)<<" t.at<double>(2):"<<t_pub.at<float>(2,0)<<std::endl;
    matrixToQuaternion(Rwc_pub,Q);
    //std::out<<"Q : "<<Q<<std::endl;
    convertPoseFormat(twc_pub,Q,pose_pub);
    orbOdom.header.seq = msgLeft->header.seq;
    orbOdom.header.stamp = msgLeft->header.stamp;
    orbOdom.header.frame_id = "camera_link";
    orbOdom.child_frame_id = "camera_odom";
    orbOdom.pose.pose = pose_pub;
   // std::cout<<"orbOdom.time:"<<orbOdom.header.stamp<<"  pose_pub.position :"<< pose_pub.position<<std::endl;
    orbPath_pub.publish(orbOdom);
}

void ImageGrabber::convertPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose) {

    pose.position.x = t.at<float>(0,0);
    pose.position.y = t.at<float>(1,0);
    pose.position.z = t.at<float>(2,0);
    pose.orientation.w = Q.w();
    pose.orientation.x = Q.x();
    pose.orientation.y = Q.y();
    pose.orientation.z = Q.z();

}

void ImageGrabber::matrixToQuaternion(const cv::Mat& mat, Eigen::Quaternion<double>& quat) {
    double m00 = mat.at<float>(0,0);
    double m01 = mat.at<float>(0,1);
    double m02 = mat.at<float>(0,2);
    double m10 = mat.at<float>(1,0);
    double m11 = mat.at<float>(1,1);
    double m12 = mat.at<float>(1,2);
    double m20 = mat.at<float>(2,0);
    double m21 = mat.at<float>(2,1);
    double m22 = mat.at<float>(2,2);
    double qw, qx, qy, qz;
    double tr1 = 1.0 + m00 - m11 - m22;
    double tr2 = 1.0 - m00 + m11 - m22;
    double tr3 = 1.0 - m00 - m11 + m22;
    if ((tr1 > tr2) && (tr1 > tr3)) {
    double S = sqrt(tr1) * 2.0; // S=4*qx
     qw = (m21 - m12) / S;
     qx = 0.25 * S;
     qy = (m01 + m10) / S;
     qz = (m02 + m20) / S;
     } else if ((tr2 > tr1) && (tr2 > tr3)) {
    double S = sqrt(tr2) * 2.0; // S=4*qy
     qw = (m02 - m20) / S;
     qx = (m01 + m10) / S;
     qy = 0.25 * S;
     qz = (m12 + m21) / S;
     } else if ((tr3 > tr1) && (tr3 > tr2)) {
     double S = sqrt(tr3) * 2.0; // S=4*qz
     qw = (m10 - m01) / S;
     qx = (m02 + m20) / S;
     qy = (m12 + m21) / S;
     qz = 0.25 * S;
     } else {
    qw = 1.0;
    qx = 0.0;
    qy = 0.0;
    qz = 0.0;
     }
    quat.x() = qx;
    quat.y() = qy;
    quat.z() = qz;
    quat.w() = qw;
    quat.normalize();
 }
