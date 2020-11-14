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
#include "esperienza1/drawSpostamento.h"
#include "esperienza1/drawRicerca.h"

#define time 100

using namespace std;
using namespace cv;

Mappa m;
cv::Mat img_map;
bool drawMap;
//int time = 1000;



/******************************Disegno della mappa e nuova direzione***************************************/
char nuovaDirezione( int a, int n, int e, int s, int o )
	{
		cout << "dentro nuovo 'nuovaDirezione'" << a << " " << n << " " << e << " " << s << " " << o << endl;
		char d = 'f';
		if( a == 0 )
			return 'a';

		n = ( n < 0 ) ? INT_MAX : n;     //se sono negativi vuol dire che non posso andare e metto "infinito"
		e = ( e < 0 ) ? INT_MAX : e;
		s = ( s < 0 ) ? INT_MAX : s;
		o = ( o < 0 ) ? INT_MAX : o;

		cout << "dentro nuovo 'nuovaDirezione' 2 " << a << " " << n << " " << e << " " << s << " " << o << endl;

		if( ( e <= n ) && ( e <= s ) && ( e <= o ) )
			{
				d = 'e';
			}

		if( ( s <= n ) && ( s <= e ) && ( s <= o ) )
			{
				d = 's';
			}

		if( ( o <= n ) && ( o <= s ) && ( o <= e ) )
			{
				d = 'o';
			}

		if( ( n <= o ) && ( n <= s ) && ( n <= e ) )
			{
				d = 'n';
			}
		return d;
	}

// void disegnaMappa( int X, int Y, Mappa& m )
// 	{
// 		int D;

// 		for( int i = Y - 1; i >= 0; --i )
// 			{
// 				for( int j = 0; j < X; ++j )
// 					{
// 						D = m.distanzaCella( j, i );

// 						if( D < 0 )
// 							cout << " X   ";
// 						else
// 							{
// 								if( D > 9 )
// 									cout << D << "   ";
// 								else
// 									cout << " " << D << "   ";
// 							}

// 					}

// 				cout << endl;
// 			};
// 	}

/******************************Services callbacks***************************************/

bool get_pesi(esperienza1::GetPesi::Request  &req, esperienza1::GetPesi::Response &res) 
{
  ROS_INFO("GetPesi, richiesta ricevuta: estazione dei valori dalla mappa...");
  res.pesi[0] = m.distanzaCella( req.x, req.y, 'a');
  res.pesi[1] = m.distanzaCella( req.x, req.y, 'n');
  res.pesi[2] = m.distanzaCella( req.x, req.y, 'e');
  res.pesi[3] = m.distanzaCella( req.x, req.y, 's');
  res.pesi[4] = m.distanzaCella( req.x, req.y, 'o');

  cout << "Pesi inviati" << endl;

  //m.abbassaPesoOstacoli();
  //disegnaMappa( 15, 15, m );
  
  return true;
}

bool posso_andare(esperienza1::possoAndare::Request  &req, esperienza1::possoAndare::Response &res) 
{
  bool answer = m.possoAndare(req.x, req.y);
  if(answer)
  	res.answer = 1;
  else
  	res.answer = 0;
  
  return true;
}

bool set_ostacolo(esperienza1::setOstacolo::Request  &req, esperienza1::setOstacolo::Response &res) 
{

  m.setOstacolo(req.x, req.y);

  res.check = 1;

  if(drawMap)
  {
	  img_map = m.immagineMappa();
	  imshow("Mappa", img_map);
	  waitKey(time);
  }
  
  return true;
}

bool rimuovi_ostacolo(esperienza1::setOstacolo::Request  &req, esperienza1::setOstacolo::Response &res) 
{

  m.rimuoviOstacolo(req.x, req.y);

  res.check = 1;

  if(drawMap)
  {
	  img_map = m.immagineMappa();
	  imshow("Mappa", img_map);
	  waitKey(time);
  }
   
  return true;
}

bool draw_spostamento(esperienza1::drawSpostamento::Request  &req, esperienza1::drawSpostamento::Response &res) 
{

  img_map = m.immagineMappa(req.x, req.y, false);
  imshow("Mappa", img_map);
  waitKey(time);

  return true;
}

bool draw_ricerca(esperienza1::drawRicerca::Request  &req, esperienza1::drawRicerca::Response &res) 
{

  img_map = m.immagineMappa(req.x, req.y, true);
  imshow("Mappa", img_map);
  waitKey(time);

  return true;
}


int main(int argc, char **argv)
{
	
	//******** Inizializzo ROS *****************
	ros::init(argc, argv, "Mapping");
	ros::NodeHandle node;

	//Definisco i servizi
	ros::ServiceServer service1 = node.advertiseService("mapping_getPesi", get_pesi);

	ros::ServiceServer service2 = node.advertiseService("mapping_possoAndare", posso_andare);

	ros::ServiceServer service3 = node.advertiseService("mapping_setOstacolo", set_ostacolo);

	ros::ServiceServer service4 = node.advertiseService("mapping_rimuoviOstacolo", rimuovi_ostacolo);

	ros::ServiceServer servise5 = node.advertiseService("mapping_drawSpostamento", draw_spostamento);

	ros::ServiceServer service6 = node.advertiseService("mapping_drawRicerca", draw_ricerca);


	// ************ Inizializzazione Mappa *******************
	ROS_INFO("Initializing the map...");
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

	//Impostazione goal e partenza
	int x_goal, y_goal, x_start, y_start;
	n = fs["Arrivo"];
	n["x"] >> x_goal;
	n["y"] >> y_goal;

	n = fs["Inizio"];
	n["x"] >> x_start;
	n["y"] >> y_start;

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

	ROS_INFO("Mappa inizializzata, servizi pronti, in attesa del client.");


	//Avvio visualizzazione

	fs["DrawMap"] >> drawMap;
	fs.release();

	if(drawMap)
	{
		img_map = m.immagineMappa(x_start, y_start, false);
		namedWindow("Mappa");
		
		while(ros::ok())
		{
			imshow("Mappa", img_map);
			waitKey(time);
			ros::spinOnce();
		}
	}
	
	ros::spin();

	return 0;
}
