
//#include <esperienza1/Mappa.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <string>

using namespace cv;

/*
   int main(int argc, char** argv) {
   
	    //Carico le singole immagini per le celle
	    std::string path = ros::package::getPath("esperienza3");
		path = path + "/src/";

	    cella_vuota = imread(path + "cella_vuota.png");
	    cella_robot = imread(path + "cella_robot.png");
	    cella_ostacolo = imread(path + "cella_ostacolo.png");
	    cella_esplorata = imread(path + "cella_esplorata.png");
	    cella_ricerca = imread(path + "cella_ricerca.png");
	    cella_flag = imread(path + "cella_flag.png");
	    cella_rows = cella_vuota.rows;
	    cella_cols = cella_vuota.cols;

		lunghezza = 10;
		altezza = 5;


	    //Inizializzo la mappa completa
	    map = Mat(cella_rows * altezza, cella_cols * lunghezza, cella_vuota.type());

	    //Riempio la mappa
	    for(int row = 0; row < altezza; ++row)
	        for(int col = 0; col < lunghezza; ++col)
	        {
	            //if(mappa[row * lunghezza + col].getBlocco())
	            
	                cella_ostacolo.copyTo(map(Rect(col * cella_cols, row * cella_rows, cella_cols, cella_rows)));
	        
	        }

	    //Aggiunta flag e robot
	    //cella_flag.copyTo(map(Rect( XStop * cella_cols, YStop * cella_rows, cella_cols, cella_rows)));
	    //cella_robot.copyTo(map(Rect( x * cella_cols, y * cella_rows, cella_cols, cella_rows)));

}
