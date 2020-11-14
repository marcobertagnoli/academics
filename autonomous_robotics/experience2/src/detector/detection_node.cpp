#include <opencv2/core/core.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <stdio.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv_apps/Point2DArray.h>
#include <opencv_apps/Point2D.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <esperienza2/computePose.h>

class Detector
{
    private:

        //Handle ros
        ros::NodeHandle nh;
        image_transport::Subscriber left_img_sub, right_img_sub;
        std::string package_path;
        cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
        ros::ServiceServer computePose_srv;
        void leftCallback(const sensor_msgs::Image::ConstPtr& msg);
        void rightCallback(const sensor_msgs::Image::ConstPtr& msg);
        bool computePose(esperienza2::computePose::Request &req, esperienza2::computePose::Response &res);

        //Image computing
        cv::Mat left, right, template_img, comp_left, comp_right, comp_lr, tmp_left, tmp_right;
        std::vector<cv::KeyPoint> keypoints_left, keypoints_right, keypoints_template, keypoints_in_left, keypoints_in_right;
        cv::Mat descriptors_left, descriptors_right, descriptors_template, desc_in_right, desc_in_left;
        cv::SurfFeatureDetector detector;
        cv::SurfDescriptorExtractor extractor;
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches_lr, matches_tl, matches_tr;
        std::vector< cv::DMatch > good_matches_lr, good_matches_tl, good_matches_tr;
        int minHessian;
        std::vector<cv::Mat> descriptors, masks;
        double max_dist, min_dist;

        //Detector
        void DetectMatches(std::string caller);
        bool call_left, call_right;

    public:
        Detector();
        ~Detector();
};

Detector::Detector()
{
    package_path = ros::package::getPath( "esperienza2" );
    template_img = cv::imread( package_path + "/data/template_lattina.png", CV_LOAD_IMAGE_GRAYSCALE );

    image_transport::ImageTransport im_tr(nh);
    left_img_sub = im_tr.subscribe("/multisense_sl/camera/left/image_raw", 1, &Detector::leftCallback, this);
    right_img_sub = im_tr.subscribe("/multisense_sl/camera/right/image_raw", 1, &Detector::rightCallback, this);

    computePose_srv = nh.advertiseService( "computePose", &Detector::computePose, this );

    minHessian = 400;
    detector = cv::SurfFeatureDetector( minHessian );

//    cv::namedWindow("Left Camera", cv::WINDOW_NORMAL);
//    cv::namedWindow("Right Camera", cv::WINDOW_NORMAL);
//    cv::namedWindow("Matched Right", cv::WINDOW_NORMAL);
//    cv::namedWindow("Matched Left", cv::WINDOW_NORMAL);
      cv::namedWindow("Matches", cv::WINDOW_NORMAL);
//      cv::namedWindow("Matches left", cv::WINDOW_NORMAL);
//      cv::namedWindow("Matches right", cv::WINDOW_NORMAL);


    call_left = false;
    call_right = false;

    //Computing template image features
    detector.detect(template_img, keypoints_template);
    extractor.compute(template_img, keypoints_template, descriptors_template);
}

Detector::~Detector()
{
    cv::destroyAllWindows();
}

void Detector::DetectMatches(std::string caller)
{
    if (caller == "l")
    {
        call_left = true;
    }
    else if (caller == "r")
    {
        call_right = true;
    }

    if(call_left & call_right)
    {
        good_matches_lr.clear();
        int counter = 0;

        while(good_matches_lr.size() < 3 && counter < 5) //Finchè non trova almeno 2 match left and right continua a cercare dopo 10 tentativi si ferma
        {
            tmp_left = left.clone();
            tmp_right = right.clone();
            counter ++;
            //Key-points detection and features extraction
            keypoints_right.clear();
            keypoints_left.clear();
            std::cout << "Compute and detect keypoints and features" << std::endl;
            detector.detect(tmp_right, keypoints_right);
            extractor.compute(tmp_right, keypoints_right, descriptors_right);
            detector.detect(tmp_left, keypoints_left);
            extractor.compute(tmp_left, keypoints_left, descriptors_left);

            //Compute matches and distances between matched keypoints
            std::cout << "Compute matches and distances between matched keypoints" << std::endl;
            matches_tr.clear();
            matches_tl.clear();
            matcher.match( descriptors_right, descriptors_template, matches_tr );
            matcher.match( descriptors_left, descriptors_template, matches_tl );
            std::cout << "left image: " << matches_tl.size() << "matches" << std::endl;
            std::cout << "right image: " << matches_tr.size() << "matches" << std::endl;

            //Extraction of good tr matches
            max_dist = 0;
            min_dist = 100;

            for( int i = 0; i < matches_tr.size(); i++ )
            {
                double dist = matches_tr[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            good_matches_tr.clear();
            for( int i = 0; i < matches_tr.size(); i++ )
            { if( matches_tr[i].distance <= cv::max(4*min_dist, 0.05) ) //cv::max(2*min_dist, 0.01)
                { good_matches_tr.push_back( matches_tr[i]); }
            }

            //Extraction of good tl matches
            max_dist = 0;
            min_dist = 100;

            for( int i = 0; i < matches_tl.size(); i++ )
            {
                double dist = matches_tl[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            good_matches_tl.clear();
            for( int i = 0; i < matches_tl.size(); i++ )
            { if( matches_tl[i].distance <= cv::max(4*min_dist, 0.05) ) //cv::max(2*min_dist, 0.01)
                { good_matches_tl.push_back( matches_tl[i]); }
            }

            //Print left and right matches wrt template
//            cv::drawMatches( tmp_left, keypoints_left, template_img, keypoints_template, good_matches_tl, comp_left, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


//            cv::drawMatches( tmp_right, keypoints_right, template_img, keypoints_template, good_matches_tr, comp_right, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//            cv::imshow("Matches left", comp_left);
//            cv::imshow("Matches right", comp_right);
//            cv::waitKey(0);

            //Extraxtion of the descriptors and keypoints inside the coke

            //Right descriptors
            keypoints_in_right.clear();
            desc_in_right = descriptors_right(cv::Rect(0,0,descriptors_right.cols, good_matches_tr.size()));
            std::cout << "right image: " << good_matches_tr.size() << "good matches" << std::endl;
            for(int i = 0; i<good_matches_tr.size(); i++)
            {
                descriptors_right.row(good_matches_tr[i].queryIdx).copyTo(desc_in_right.row(i));
                keypoints_in_right.push_back(keypoints_right[good_matches_tr[i].queryIdx]);
            }

            std::cout << "Desc in right dimension: " << desc_in_right.cols << " , " << desc_in_right.rows << std::endl;

            //Left descriptors
            keypoints_in_left.clear();
            desc_in_left = descriptors_left(cv::Rect(0,0,descriptors_left.cols, good_matches_tl.size()));
            std::cout << "left image: " << good_matches_tl.size() << "good matches" << std::endl;

            for(int i = 0; i<good_matches_tl.size(); i++)
            {
                descriptors_left.row(good_matches_tl[i].queryIdx).copyTo(desc_in_left.row(i));
                keypoints_in_left.push_back(keypoints_left[good_matches_tl[i].queryIdx]);
            }

            std::cout << "Desc in left dimension: " << desc_in_left.cols << " , " << desc_in_left.rows << std::endl;

            std::cout << "Descriptors copied" << std::endl;
            ;
            //Compute left-right matches
            matches_lr.clear();
            matcher.match( desc_in_right, desc_in_left, matches_lr );

            std::cout << "Left right matches computed" << std::endl;

            //Extraction of good lr matches
            max_dist = 0;
            min_dist = 100;

            for( int i = 0; i < matches_tl.size(); i++ )
            {
                double dist = matches_lr[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            good_matches_lr.clear();
            for( int i = 0; i < matches_lr.size(); i++ )
            { if( matches_lr[i].distance <= cv::max(4*min_dist, 0.05) ) //cv::max(2*min_dist, 0.01)
                { good_matches_lr.push_back( matches_lr[i]); }
            }
        }

        //Match keypoints
        opencv_apps::Point2D roskey_temp;
        cv::Point2f cvkey_temp;
        opencv_apps::Point2DArray roskey_left, roskey_right;

        for (int i = 0; i < good_matches_lr.size(); ++i)
        {
            //left
            cvkey_temp = keypoints_in_left[good_matches_lr[i].trainIdx].pt;
            roskey_temp.y = cvkey_temp.y;
            roskey_temp.x = cvkey_temp.x;
            roskey_left.points.push_back(roskey_temp);
            //right
            cvkey_temp = keypoints_in_right[good_matches_lr[i].queryIdx].pt;
            roskey_temp.y = cvkey_temp.y;
            roskey_temp.x = cvkey_temp.x;
            roskey_right.points.push_back(roskey_temp);
        }


        //Draw the good left right matches
        std::cout << "good matches left right: " << good_matches_lr.size() << std::endl;
        cv::drawMatches( tmp_right, keypoints_in_right, tmp_left, keypoints_in_left, good_matches_lr, comp_lr, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::imshow("Matches", comp_lr);
        cv::waitKey(0);

        call_left = false;
        call_right = false;
    }

    return;
}

void Detector::leftCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    //Get image from the topic
    try
    {
        cv_ptr_left = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    left = cv_ptr_left -> image;

    if(left.cols > 10)
    {
//        std::cout << "Printing left image" << std::endl;
//        cv::imshow("Left Camera", left);
//        cv::waitKey(5);

        DetectMatches("l");
    }
}

void Detector::rightCallback(const sensor_msgs::Image::ConstPtr &msg)
{

    try
    {
        cv_ptr_right = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    right = cv_ptr_right -> image;
    if(left.cols > 10)
    {
//        std::cout << "Printing right image" << std::endl;
//        cv::imshow("Right Camera", right);
//        cv::waitKey(5);

        DetectMatches("r");
    }

//    //Key-points detection and features extraction
//    keypoints_right.clear();
//    detector.detect(right, keypoints_right);
//    extractor.compute(right, keypoints_right, descriptors_right);

//    //Compute matches and distances between matched keypoints
//    matches_tr.clear();
//    matcher.match( descriptors_template, descriptors_right, matches_tr );
//    std::cout << "hello world" << std::endl;
//    //std::cout << "Matched right: " << matches_tr[0] << std::endl;

//    double max_dist = 0;
//    double min_dist = 100;

//    for( int i = 0; i < descriptors_template.rows; i++ )
//    {
//        double dist = matches_tr[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }

//    //Extraction of good matches
//    good_matches_tr.clear();
//    for( int i = 0; i < descriptors_template.rows; i++ )
//    { if( matches_tr[i].distance <= cv::max(2*min_dist, 0.01) )
//      { good_matches_tr.push_back( matches_tr[i]); }
//    }

//    //Draw the good matches
//    cv::drawMatches( template_img, keypoints_template, right, keypoints_right, good_matches_tr, comp_right, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

//    //Visualization
//    cv::imshow("Right Camera", comp_right);
//    cv::waitKey(5);




}

bool Detector::computePose(esperienza2::computePose::Request &req, esperienza2::computePose::Response &res)
{
    //Uopdate image variables
    ros::spinOnce();

    //Key-points detection and features extraction
    keypoints_right.clear();
    keypoints_left.clear();
    detector.detect(right, keypoints_right);
    extractor.compute(right, keypoints_right, descriptors_right);
    detector.detect(left, keypoints_left);
    extractor.compute(left, keypoints_left, descriptors_left);

    //Compute matches and distances between matched keypoints
    matcher.clear();
    matches_tr.clear();
    matches_tl.clear();

    descriptors.push_back(descriptors_right);
    descriptors.push_back(descriptors_left);
    matcher.add(descriptors);

    matcher.match( descriptors_template, matches_lr, masks);

//    //Extraxt matched keypoynt from both images


//    double max_dist = 0;
//    double min_dist = 100;

//    for( int i = 0; i < descriptors_template.rows; i++ )
//    {
//        double dist = matches_lr[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }

//    //Extraction of good matches
//    good_matches_lr.clear();
//    for( int i = 0; i < descriptors_template.rows; i++ )
//    { if( matches_lr[i].distance <= cv::max(2*min_dist, 0.01) )
//      { good_matches_lr.push_back( matches_lr[i]); }
//    }





    return true;
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "detection_node" );

    Detector det;

    ros::spin();

    return 0;
}
