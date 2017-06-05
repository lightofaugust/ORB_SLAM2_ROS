#ifndef TF2INTERFACE
#define TF2INTERFACE

//these are for tf2 publishing
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//these are for looking up the tf between camera_link and rgb_optical at the beginning
#include <tf2_ros/transform_listener.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Converter.h"

namespace ORB_SLAM2
{
    class TF2Interface
    {
    public:
        TF2Interface()
        {
            //mTfClinkToRgb initialization?

        }
        ~TF2Interface(){}
        void PublishCurrentFrameTcw(cv::Mat &Tcw);
        void LookupCameralinkToRgbOptical();

    private:

        geometry_msgs::TransformStamped mTfClinkToRgb;

        //geometry_msgs::TransformStamped mTfWtoClink;
        tf2_ros::TransformBroadcaster mTfBr;

    };

    void TF2Interface::PublishCurrentFrameTcw(cv::Mat &Tcw)
    {
        if (Tcw.empty())
            return;

        //transform between left-hand ORB to right-hand ros
        cv::Mat Tcw_left = Tcw;
        cv::Mat Rcw_left = Tcw_left.rowRange(0,3).colRange(0,3);
        cv::Mat tcw_left = Tcw_left.rowRange(0,3).col(3);

        //flip X to get right handness
        tcw_left.at<float>(0) = -tcw_left.at<float>(0);
        static const cv::Mat Rdirection = (cv::Mat_<float>(3,3) << -1,0,0,0,1,0,0,0,1);
        cv::Mat RflipX = Rdirection*Rcw_left*Rdirection;

        //left and right handness have different clockwise direction
        cv::transpose(RflipX,RflipX);

        //convert from current right-hand coordinates to ros convention
        static const cv::Mat RtoROS = (cv::Mat_<float>(3,3) << 0,0,-1,-1,0,0,0,1,0);
        tcw_left = RtoROS*tcw_left; //ros convention world to camera_rgb --t
        RflipX = RtoROS*RflipX;     //ros convention world to camera_rgb --R

        //convert from W_to_rgb to rgb_to_W, because tf in ros can only be send in this direction
        cv::Mat Rwc; 
        cv::transpose(RflipX, Rwc);
        cv::Mat twc = -Rwc*tcw_left; 

        geometry_msgs::TransformStamped tfROSWtoORB; //it's ros world relative to ORB coordinate
        tfROSWtoORB.header.frame_id = "camera_rgb_optical_frame";
        tfROSWtoORB.child_frame_id = "world";
        tfROSWtoORB.header.stamp = ros::Time::now();
        tfROSWtoORB.transform.translation.x = twc.at<float>(0);
        tfROSWtoORB.transform.translation.y = twc.at<float>(1);
        tfROSWtoORB.transform.translation.z = twc.at<float>(2);
        vector<float> q = Converter::toQuaternion(Rwc);
        tfROSWtoORB.transform.rotation.x = q[0];
        tfROSWtoORB.transform.rotation.y = q[1];
        tfROSWtoORB.transform.rotation.z = q[2];
        tfROSWtoORB.transform.rotation.w = q[3];

        //transform from rgb_optical(ORB) to camera_link
        // tf2::doTransform(tfROSWtoORB, mTfWtoClink, mTfClinkToRgb);

        mTfBr.sendTransform(tfROSWtoORB);
    }

    //look up the tf between camera_link and rgb_optical at the beginning, for 'rgb_optical -> world' publishing
    void TF2Interface::LookupCameralinkToRgbOptical()
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        //                                          frame                 child_frame          
        mTfClinkToRgb = tfBuffer.lookupTransform("camera_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
        std::cout << mTfClinkToRgb << std::endl;
    }
}






#endif
