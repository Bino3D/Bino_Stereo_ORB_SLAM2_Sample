#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
};

std::string vocabulary;
std::string settings;
std::string intrinsuc_file;
std::string extrinsic_file;

int main(int argc, char **argv)
{
 ros::init(argc, argv, "Bino");
    ros::start();
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
    nh_private.param<std::string>("vocabulary",vocabulary, "ORBvoc.bin"); 
    nh_private.param<std::string>("settings",settings, "BINO3.6.yaml");
    nh_private.param<std::string>("intrinsuc_file",intrinsuc_file, "intrinsics.yml");
    nh_private.param<std::string>("extrinsic_file",extrinsic_file, "extrinsics.yml");
    ORB_SLAM2::System SLAM(vocabulary,settings,ORB_SLAM2::System::STEREO,true);
    ImageGrabber igb(&SLAM);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "image_rect_l", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "image_rect_r", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

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
    cv::Mat imLeft, imRight;
	imLeft=cv_ptrLeft->image.clone();
	imRight=cv_ptrRight->image.clone();
    mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec()); //Tcw
}


  
