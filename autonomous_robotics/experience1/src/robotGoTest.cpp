
#include <iostream>
#include <esperienza1/Mappa.h>
#include "esperienza1/navigate_nxt.h"
#include "ros/ros.h"
#include <cstdlib>
#include "stdlib.h"


using namespace std;

int main(int argc, char **argv)
	{
		
		// ******** Inizializzo ROS *****************
		ros::init(argc, argv, "rotate");
		ros::NodeHandle n;
		Navigate_Nxt nav(n);
		
		vector<bool> Cones;
		
		nav.rotate('l');
		//nav.rotate('l');
		nav.robotGo();

		Cones = nav.findCones( 2,2);
		cout << Cones[0]<< Cones[1]<< Cones[2]<< Cones[3] << endl;
		
		//sleep(10);


		
		ros::spin();
		ros::shutdown();
		
		return 0;
	}
