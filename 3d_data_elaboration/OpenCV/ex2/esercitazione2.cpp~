#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <iostream>

using namespace cv;

/*
 * 
 */



int main(){
	
	Mat img=imread("sunset-lake.jpg");
	Mat img_HSV; //immagine in formato HSV
	Mat img_sat; //immagine saturata
	
	//Conversione da BGR a HSW
	cvtColor(img, img_HSV, CV_BGR2HSV);
	
	//Saturazione
	for(int i=0; i<img_HSV.rows; ++i){
	
		for(int j=0; j<img_HSV.cols; ++j){
		
			img_HSV.at<Vec3b>(i,j)[1]=255;
			img_HSV.at<Vec3b>(i,j)[2]=255;		
		}
	
	}
	
	cvtColor(img_HSV, img_sat, CV_HSV2BGR);	

	//Salvataggio dell'immagine
	imwrite("lago-sat.jpg", img_sat);	
	
	
	
	namedWindow("Originale");
	namedWindow("Saturata");
	
	imshow("Originale", img);
	imshow("Saturata", img_sat);
		
	waitKey(0);
	
    return 0;
}
