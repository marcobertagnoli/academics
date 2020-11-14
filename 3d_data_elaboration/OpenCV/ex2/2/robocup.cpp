#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <iostream>

using namespace cv;

/*
 * 
 */



int main(){
	
	Mat img=imread("robocup.jpg");
	Mat img_ball(img.rows, img.cols, CV_8UC3);
	
	for(int i=0; i<img.rows; ++i){
	
		for(int j=0;j<img.cols;++j){
		
			if(img.at<Vec3b>(i,j)[0]<150 && 70<img.at<Vec3b>(i,j)[1]<250 && img.at<Vec3b>(i,j)[2]>200){

				img_ball.at<Vec3b>(i,j)[0]=img.at<Vec3b>(i,j)[0];
				img_ball.at<Vec3b>(i,j)[1]=img.at<Vec3b>(i,j)[1];
				img_ball.at<Vec3b>(i,j)[2]=img.at<Vec3b>(i,j)[2];
			
			}
		
		
			else{
		
				img_ball.at<Vec3b>(i,j)[0]=0;
				img_ball.at<Vec3b>(i,j)[1]=0;
				img_ball.at<Vec3b>(i,j)[2]=0;
		
			}
		
		
		}
	
	}

	namedWindow("Originale");
	namedWindow("Ball");
	imshow("Originale", img);
	imshow("Ball", img_ball);
	waitKey(0);



	return 0;
}
