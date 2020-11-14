#include "esperienza1/navigate_nxt.h"
#include "ros/ros.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rotate");
    ros::NodeHandle n;

	Navigate_Nxt nav(n);
	

	//while(ros::ok())//fino a che ros non viene fermato
    //{
	
	nav.center();	
	nav.robotGo();
	nav.rotate('r');
	nav.robotGo();
	nav.rotate('f');
	nav.robotGo();
	nav.robotGo();

	nav.center();
		
    //}
	
	ros::spin();
	ros::shutdown();
	
	return 0;
}

/*
 * Problemi:
 * - con rate 10 hz, e 10 nelle code, il percorso a - go - o -go viene abba bene sempre
 * - a volte le rotazioni sono di poco sballate
 * 
 * */
