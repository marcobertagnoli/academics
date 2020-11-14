#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

/*
 * 
 */



int main(){
	
	const int chess_rows=5;
	
	const int chess_cols=6;
	
	const int img_number=25;
	
	
	const float square_size=3;
	
	const double scale=1;
	
	string format=".png";
	
	const Size chess_size=cvSize(chess_rows, chess_cols);
	
	string name;
	
	vector< Mat > img;
	vector< Mat > img_resize;
	vector< Mat > img_gray;
	vector< vector< Point2f > > image_points;
	vector< vector< Point3f > > object_points;
	
	
	

	
	vector<Point3f> chess_points;
	Point3f chess_point;
	vector< Point2f > corners;
	
	
	
	for(int i=1; i<=chess_cols; i++){
		for(int j=1; j<=chess_rows; j++){
		
			chess_point=Point3f (square_size*i, square_size*j, 0.0);
			
			chess_points.push_back (chess_point);
		
		}
	
	
	}
		
	for(int i=1; i<=img_number; i++){
	
		if(i<=9){
		
		name="image_000"+to_string(i)+format;
		
		}
		
		else{
		
		name="image_00"+to_string(i)+format;
		
		}
	
		
		
		cout<< name<<"\n";
		
		img.push_back (imread(name));
		
		Mat img_resizeTemp;
		
		resize(img[i-1], img_resizeTemp, Size(0,0), scale, scale);
		
		img_resize.push_back(img_resizeTemp);
		
		Mat img_grayTemp;
		
		cvtColor(img_resize[i-1], img_grayTemp, CV_BGR2GRAY);
		
		
		img_gray.push_back(img_grayTemp);
		
		
		
		bool found=findChessboardCorners(img_gray[i-1], chess_size , corners);
		cornerSubPix( img_gray[i-1], corners, Size(8,6), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		

		if(!found){}
		
		image_points.push_back(corners);
		
		object_points.push_back(chess_points);
		
		
		/*		
		Mat draw_corners=img_resize[i-1].clone();
		
		drawChessboardCorners( draw_corners , chess_size , Mat(corners), true );
		
		namedWindow("corners");
		imshow("corners", draw_corners);
		waitKey(0);
		*/
		

	
	}

	
	Size img_size=cvSize(img_resize[0].cols, img_resize[0].rows);
	
	
	Mat cameraMatrix= Mat(3, 3, CV_32FC1);
	
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	
	vector<Mat> rotVecs;
	
	vector<Mat> translVecs;
	
	
	calibrateCamera(object_points, image_points, img_size, cameraMatrix, distCoeffs, rotVecs, translVecs);	
	cout << cameraMatrix << '\n';
	
	Mat imgUndist;
	
	undistort(img_resize[0], imgUndist, cameraMatrix, distCoeffs);
	
	namedWindow("Distorted");
	namedWindow("Undistorted");
	imshow("Distorted", img_resize[0]);	
	imshow("Undistorted", imgUndist) ;
	waitKey(0);
	
	
	
	
	
	
		
	return 0;	
	
}

