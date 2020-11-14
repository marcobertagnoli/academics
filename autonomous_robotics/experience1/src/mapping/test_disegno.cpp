#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include <ros/package.h>

#include <esperienza1/Mappa.h>

#include "esperienza1/GetPesi.h"
#include "esperienza1/possoAndare.h"
#include "esperienza1/setOstacolo.h"
#include "esperienza1/rimuoviOstacolo.h"

using namespace std;
using namespace cv;



int main(int argc, char const *argv[])
{
	Mappa m;
	string path = ros::package::getPath("esperienza1");

	path = path + "/src/mappa.yaml";
	FileStorage fs;
    fs.open(path, FileStorage::READ);
    //Check sul file
    if (!fs.isOpened())
        {
            cerr << "Failed to open " << path << endl;
            return 1;
        }

    cv::FileNode n;
    
	//creazione della mappa
	n = fs["Dimensione"];
	int lunghezza, altezza, X, Y;
	n["lunghezza"] >> lunghezza;
	n["altezza"] >> altezza;

	m.inizializzazione(lunghezza, altezza);

	//Impostazione goal
	int x_goal;
	int y_goal;
	n = fs["Arrivo"];
	n["x"] >> x_goal;
	n["y"] >> y_goal;

	int check_goal = m.setArrivo(x_goal, y_goal);

	if (check_goal < -5)
	{
		cerr << "Mappa non inizializzata"<< endl;
		return -1;
	}
	else if (check_goal < 0)
	{
		cerr << "Goal esterno alla mappa"<< endl;
		return -1;
	}

	// Aggiunta Ostacoli
	n = fs["Ostacoli"];
	int numero_ostacoli = n["numero_ostacoli"];

	if (numero_ostacoli > 0)
	{
		for (int i = 0; i < numero_ostacoli; ++i)
		{
			n["x"][i] >> X;
			n["y"][i] >> Y;
			m.setOstacolo(X ,Y );
		}
	}

	fs.release();

    Mat map = m.immagineMappa(0, 0, false);

    namedWindow("Mappa");
    imshow("Mappa", map);
    waitKey(0);

    for (int i = 0; i < 5; ++i)
    {
    	map = m.immagineMappa(i, i, false);
	    imshow("Mappa", map);
	    waitKey(0);
    }

	return 0;
}
