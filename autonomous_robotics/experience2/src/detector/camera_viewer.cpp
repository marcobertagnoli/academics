#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

using namespace cv;

cv::Mat left, right;

void leftCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    left = cv_ptr -> image;

    //std::cout << "left.size: ( " << left.rows << "; " << left.cols << " )" << std::endl;
}

void rightCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    right = cv_ptr -> image;

    //std::cout << "right.size: ( " << right.rows<< "; " << right.cols << " )" << std::endl;

}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "camera_viewer" );
    ros::NodeHandle nh;

    ros::Subscriber leftimg_sub = nh.subscribe( "/multisense_sl/camera/left/image_raw", 1, leftCallback);
    ros::Subscriber rightimg_sub = nh.subscribe( "/multisense_sl/camera/right/image_raw", 1, rightCallback);

    namedWindow("Left", WINDOW_NORMAL);
    namedWindow("Right", WINDOW_NORMAL);

    ros::spinOnce();

    while( ros::ok() )
    {
        if(left.rows > 0 && left.cols > 0)
        {
            imshow( "Left", left );
            waitKey( 10 );
        }
        if(right.rows > 0 && right.cols > 0)
        {
            imshow( "Right", right );
            waitKey( 10 );
        }

        ros::spinOnce();
    }

    return 0;
}
