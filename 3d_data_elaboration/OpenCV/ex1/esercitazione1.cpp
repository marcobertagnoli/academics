#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <iostream>

using namespace cv;

/*
 * 
 */



int main(int argc, char**argv) 
{

	int sigma=10;
	int kernel_size=9;

	Mat img=imread(argv[1]);
	
	//Filtro Gaussiano

	Mat img_blur=img.clone();
	GaussianBlur(img, img_blur, Size(kernel_size, kernel_size), sigma);

	namedWindow ("Blur");
	imshow("Blur", img_blur);
	waitKey(0);

	//Canny edge detector

	Mat img_bw;

	Mat img_edge;

	cvtColor(img_blur, img_bw, CV_RGB2GRAY);
	
	Canny(img_bw, img_edge, 50, 100);

	namedWindow ("Canny");
	imshow("Canny", img_edge);
	waitKey(0);

	
	//Edge density

	int white=0;

	for(int i=0; i<img_edge.rows; ++i){

		for(int j=0; j<img_edge.cols; ++j){

			if(img_edge.at<uchar>(i,j)==255)
				++white;

		}
	
	}


	double density= (double)white/(img.cols*img.rows);

	// oppure static_cast<double> (white) ...

	std::cout << "La densità è:" << density<<"\n";

	//Harris detector
	
	Mat img_corner;
	
	const int size_harris=2;
	const double k_harris=0.04;
	const int sobel_aperture=5;
	
	cornerHarris(img_bw, img_corner, size_harris, sobel_aperture, k_harris);
	

	//Si espandono i punti notevoli per renderli visibili

	int corner=0;

	for(int i=70; i<img_edge.rows; ++i){

		for(int j=70; j<img_edge.cols; ++j){

			if(img_corner.at<uchar>(i,j)==255){
				++corner;
						
				for(int h=0; h<10; ++h){
			
					for(int y=0; y<10; ++y){
					
						img_corner.at<uchar>(i+h,j+y)=255;
						img_corner.at<uchar>(i-h,j-y)=255;
					}

				}	

			}
				

		}
	
	}

	std::cout << "Si son trovati:" << corner<<"corners"<<"\n";

	namedWindow ("Harris");
	imshow("Harris", img_corner);
	waitKey(0);
       

	return 0;
}
