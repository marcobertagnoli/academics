#include <iostream>
#include <limits.h>
#include <opencv2/core/core.hpp>
#include <string>

#include "ros/ros.h"
#include <ros/package.h>

#include <esperienza1/Mappa.h>

#include "esperienza1/GetPesi.h"
#include "esperienza1/Direzione.h"
#include "esperienza1/getPosition.h"



using namespace std;
using namespace cv;


int x_attuale, y_attuale; // Posizione attuale del robot
char direction = 'i'; // da inizializzare

char nuovaDirezione( int a, int n, int e, int s, int o )
	{

		char d = 'f';
		if( a == 0 )
			return 'a';

		n = ( n < 0 ) ? INT_MAX : n;     //se sono negativi vuol dire che non posso andare e metto "infinito"
		e = ( e < 0 ) ? INT_MAX : e;
		s = ( s < 0 ) ? INT_MAX : s;
		o = ( o < 0 ) ? INT_MAX : o;

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

bool get_position(esperienza1::getPosition::Request  &req, esperienza1::getPosition::Response &res)
{
  res.x = x_attuale;
  res.y = y_attuale;

  return true;
}



bool nuova_direzione(esperienza1::Direzione::Request  &req, esperienza1::Direzione::Response &res) 
{
	//Client per rischiesta pesi da mapping
	ros::NodeHandle n;
	ros::ServiceClient pesi_client = n.serviceClient<esperienza1::GetPesi>("mapping_getPesi");
  	esperienza1::GetPesi pesi_srv;


	pesi_srv.request.x = x_attuale;
	pesi_srv.request.y = y_attuale;

  	if (pesi_client.call(pesi_srv))
	  {
	   	direction = nuovaDirezione(pesi_srv.response.pesi[0], pesi_srv.response.pesi[1], pesi_srv.response.pesi[2], 
	   		pesi_srv.response.pesi[3], pesi_srv.response.pesi[4]);
	  
	   	std::cout << endl << "Le distanze delle celle adiacenti (a, n, e, s, o) sono: ";
	   	for (int i = 0; i < 5; ++i)
	   	{
	   		cout << "[ " << pesi_srv.response.pesi[i] << " ]";
	   	}

	   	cout << endl << endl;

	   	cout << "Muoversi nella cella: " << direction << endl;

	   	switch(direction)
	   	{
	   		case('n'):
	   			++y_attuale;
	   		break;

	   		case('e'):
	   			++x_attuale;
	   		break;

	   		case('s'):
	   			--y_attuale;
	   		break;

	   		case('o'):
	   			--x_attuale;
	   		break;
	   	}
		
	  }
	  else
	  {
	    ROS_ERROR("Failed to call service navigation_direzione");
	    return 1;
	  }

	  res.direction = direction;
  
  	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigation");

	//Servizio per il calcolo della direzione
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("navigation_direzione", nuova_direzione);
	//Servizio per restituire la posizione attuale
  	ros::ServiceServer service1 = n.advertiseService("navigation_getPosition", get_position);

  	// ************ Inizializzazione Partenza *******************
  	string path = ros::package::getPath("esperienza1");
	path = path + "/src/mappa.yaml"; //Percorso al file di configurazione

	cv::FileStorage fs;
    fs.open(path, FileStorage::READ);
    //Check sul file
    if (!fs.isOpened())
        {
            cerr << "Failed to open " << path << endl;
            return 1;
        }

    cv::FileNode fs_node;

	fs_node = fs["Inizio"];
	fs_node["x"] >> x_attuale;
	fs_node["y"] >> y_attuale;

	ROS_INFO("Inizializzazione del navigatore completata, in attesta del client.");

	ros::spin();

	return 0;
}