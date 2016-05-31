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
#include <ugv_msgs/NCOM.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/core.hpp>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "../../../include/System.h"
#include "../../../include/Converter.h"
#define DegToRad M_PI/180
#define RadToDeg 180/M_PI
#define GRAVITY 9.8035
#define EARTH_R 6367560
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM)
    {
        init_gps_.flag      = 0;
        init_gps_.altitude  = 0.0;
        init_gps_.latitude  = 0.0;
        init_gps_.longitude = 0.0;
        initial_inertial_flag_ = 0;
        f.open("ros_stereo_pose.txt");
        f<<fixed;
//        tf::Matrix3x3 c2w(0,0,1,
//                          -1,0,0,
//                          0,-1,0);

        c2w.setRPY(-M_PI/2,0,-M_PI/2);
        mTcw_imu = cv::Mat::eye(4,4,CV_32F);
        mRcw_imu = cv::Mat::eye(3,3,CV_32F);
        mtcw_imu = cv::Mat::zeros(3,1,CV_32F);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
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
    nav_msgs::Odometry inertialOdom_;
    ros::Publisher orbPath_pub;
    ros::Publisher tf_orb;
    ros::Publisher inertialPath_pub_;
    //subscribe inertial
    ros::Subscriber sub_inertial_;
    float roll,pitch,yaw;
    void dataCallback(const ugv_msgs::NCOMConstPtr &msg);
    //geometry_msgs::PoseStamped inertial_;
    geometry_msgs::Quaternion qFromeuler;
    tf::Matrix3x3 tfInertialRotation_;
    cv::Mat Initial_Pose_;
    //publish orb odometry tf
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform tf_t_;
    tf::Quaternion tf_q_;
    //! parameters for initial GPS
    struct initial_gps{
        int flag;
        float latitude;
        float longitude;
        float altitude;
    }init_gps_;
    int initial_inertial_flag_;
    ofstream f;
    tf::Matrix3x3 c2w;
    // add imu motion model to vo Nico
    cv::Mat mTcw_imu;
    cv::Mat mRcw_imu;
    cv::Mat mtcw_imu;
    tf::Matrix3x3 mtfR;
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
    igb.orbPath_pub = nh.advertise<nav_msgs::Odometry>("orbslam_path",1);
    igb.inertialPath_pub_ = nh.advertise<nav_msgs::Odometry>("inertial_odom",1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/stereo/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/stereo/right/image_raw", 1);
   //todo inertial msg
    igb.sub_inertial_ = nh.subscribe("/inertial",1,&ImageGrabber::dataCallback,&igb);
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
    igb.f.close();
    ros::shutdown();

    return 0;
}

void ImageGrabber::dataCallback(const ugv_msgs::NCOMConstPtr &msg)
{

    roll = msg->roll*DegToRad;
    pitch = -msg->pitch*DegToRad;
    yaw = -msg->heading*DegToRad;

    qFromeuler = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    inertialOdom_.header.frame_id = "world";
    inertialOdom_.header.stamp = msg->header.stamp;
    inertialOdom_.header.seq = msg->header.seq;
    inertialOdom_.child_frame_id = "vehicle";
    inertialOdom_.pose.pose.orientation.x = qFromeuler.x;
    inertialOdom_.pose.pose.orientation.y = qFromeuler.y;
    inertialOdom_.pose.pose.orientation.z = qFromeuler.z;
    inertialOdom_.pose.pose.orientation.w = qFromeuler.w;
    if(init_gps_.flag ==0)
    {
        init_gps_.flag = 1;
        init_gps_.latitude  = msg->latitude;
        init_gps_.longitude = msg->longitude;
        init_gps_.altitude  = msg->altitude;
        inertialOdom_.pose.pose.position.x = 0;
        inertialOdom_.pose.pose.position.y = 0;
        inertialOdom_.pose.pose.position.z = 0;
        tfInertialRotation_.setRPY(pitch,yaw,-roll);
        Initial_Pose_ = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                Initial_Pose_.at<float>(j,i) = tfInertialRotation_[j][i];
    }
    else
    {
        inertialOdom_.pose.pose.position.x =  (msg->latitude  - init_gps_.latitude ) * DegToRad * EARTH_R;
        inertialOdom_.pose.pose.position.y =  -(msg->longitude - init_gps_.longitude) * DegToRad * EARTH_R * cos(msg->latitude * DegToRad);
        inertialOdom_.pose.pose.position.z =  (msg->altitude  - init_gps_.altitude );
    }
    mtfR.setRPY(roll,pitch,yaw);
    mtfR = c2w.transpose()*mtfR*c2w;
    mRcw_imu = ORB_SLAM2::Converter::toCvMat(mtfR);
    mRcw_imu = mRcw_imu.t();
    mRcw_imu.copyTo(mTcw_imu.rowRange(0,3).colRange(0,3));
    mtcw_imu.at<float>(0,0) = (float)(-inertialOdom_.pose.pose.position.y);
    mtcw_imu.at<float>(1,0) = (float)(-inertialOdom_.pose.pose.position.z);
    mtcw_imu.at<float>(2,0) = (float)(inertialOdom_.pose.pose.position.x);
    mtcw_imu = -mRcw_imu*mtcw_imu;
    mtcw_imu.copyTo(mTcw_imu.rowRange(0,3).col(3));
    mpSLAM->mTcw_imu = mTcw_imu;
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

    static int flag = 0;
    if (flag==0&&init_gps_.flag ==1)
    {
        flag = 1;
        mpSLAM->Initial_Pose_ = Initial_Pose_;
    }

    if(do_rectify&&init_gps_.flag ==1)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        T_pub=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
//        std::cout<<"T_pub : "<<T_pub<<std::endl;
        grabTandpub(msgLeft,msgRight);
//        if(abs(twc_pub.at<float>(1))>=0.2)
//        initial_inertial_flag_=1;
    }
//    else
//    {
//        T_pub=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
//        grabTandpub(msgLeft,msgRight);
//        //std::cout<<"T_pub : "<<T_pub<<std::endl;
//    }

}

void ImageGrabber::grabTandpub(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{

    static tfScalar vroll,vpitch,vyaw;
    Rwc_pub = T_pub.rowRange(0,3).colRange(0,3).t();
    twc_pub = -Rwc_pub*T_pub.rowRange(0,3).col(3);
    //std::cout <<"twc_pub: "<< twc_pub << std::endl;
    std::cout <<"R_vo: "<<  Rwc_pub << std::endl;
    std::cout <<"R_ins: "<< mRcw_imu << std::endl;
    std::cout <<"t_vo: "<< T_pub.rowRange(0,3).col(3) << std::endl;
    std::cout <<"t_ins: "<< mtcw_imu<< std::endl;


    //std::cout <<"inertialOdom_.pose.pose.position: "<<inertialOdom_.pose.pose.position.x<<' '<<inertialOdom_.pose.pose.position.y<< ' '<<inertialOdom_.pose.pose.position.z<<endl;
    tf::Matrix3x3 M(Rwc_pub.at<float>(0,0),Rwc_pub.at<float>(0,1),Rwc_pub.at<float>(0,2),
                    Rwc_pub.at<float>(1,0),Rwc_pub.at<float>(1,1),Rwc_pub.at<float>(1,2),
                    Rwc_pub.at<float>(2,0),Rwc_pub.at<float>(2,1),Rwc_pub.at<float>(2,2));
    M=c2w*M*c2w.transpose();
    M.getRPY(vroll,vpitch,vyaw);

//    std::cout<<"roll: "    << vroll*RadToDeg       <<
//               "  pitch: "<< vpitch*RadToDeg     <<
//               "  yaw: " << vyaw*RadToDeg       <<  std::endl;
    f << setprecision(9) << vroll*RadToDeg << " "<<vpitch*RadToDeg<<" "<<vyaw*RadToDeg <<" "<<twc_pub.at<float>(2)<<" "<<-twc_pub.at<float>(0)<<" "<<-twc_pub.at<float>(1)<<endl;
    tf_t_.setOrigin(tf::Vector3(inertialOdom_.pose.pose.position.x,inertialOdom_.pose.pose.position.y,0.0));
    tf_q_.setRPY(roll,pitch,yaw);
    tf_t_.setRotation(tf_q_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_,inertialOdom_.header.stamp,"/world","/vehicle"));

    tf_t_.setOrigin(tf::Vector3( twc_pub.at<float>(2), -twc_pub.at<float>(0),-twc_pub.at<float>(1)));
    //tf_q_.setRPY(-vyaw, vroll, vpitch);
    //tf_q_.setRPY( vroll,vpitch, vyaw);
    //tf_t_.setRotation(tf_q_);
    tf_t_.setBasis(M);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_, msgLeft->header.stamp, "/world", "/visual_odometry"));


    orbOdom.header.seq = msgLeft->header.seq;
    orbOdom.header.stamp = msgLeft->header.stamp;
    orbOdom.header.frame_id = "world";
    orbOdom.child_frame_id = "visual_odometry";
    tf::poseTFToMsg(tf_t_,orbOdom.pose.pose);
    orbPath_pub.publish(orbOdom);
    inertialPath_pub_.publish(inertialOdom_);
}


