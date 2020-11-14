#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nxt_msgs/Color.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include <sstream>

char color = 'g'; //g: va dritto; t:gira di 90°; s:fermo; b:retro; a: allinea

double x, y, orientation;
double distance = 0;
double x_old = 0;
double y_old = 0;


void intensityCallback(const nxt_msgs::Color::ConstPtr& msg)
{

    if (msg->r < 0.4)
    {
        color = 'w';
    }
    else
    {
        color = 'g';
    }
}

/*
void robotGo(const ros::Publisher& vel, double Space, double Vmax, double Amax, double periode)
{
	geometry_msgs::Twist vels;
	geometry_msgs::Vector3 lin, ang;

	// trovo tempo di accelerazione e tempo con vel. di regime
	double Ta = Vmax/Amax;
	double T = Space/Vmax + Vmax/Amax;

	// faccio partire robot
	vel_ist = 0;
	passo = Ta/periode;

	int i = passo;

	// Tratto di accelerazione
	while(i <= Ta)
	    {
			// calcolo velocità ad ogni istante
			vel_ist = vel_ist + (Vmax/Ta)*passo;
			// assegno vel istantanea al robot
	        lin.x = vel_ist;
	        lin.y = 0;
	        lin.z = 0;
	        ang.x = 0;
	        ang.y = 0;
	        ang.z = 0;

	        vels.linear = lin;
	        vels.angular = ang;
	        vel.publish(vels);

			i=i+passo;
	    }

	// Tratto di regime
	// ora i è =Ta
	while(i <= (T-Ta))
	    {


	    }


		    }
}*/


void robotStop(const ros::Publisher& VEL) //HO TOLTO IL PUNTATORE DINAMICO: CANCELLARE L'ISTANZA!
{
	geometry_msgs::Twist vels;
	geometry_msgs::Vector3 lin, ang;

	// Mi fermo
    for(int i=0; i<50; i++)
    {
        lin.x = 0;
        lin.y = 0;
        lin.z = 0;
        ang.x = 0;
        ang.y = 0;
        ang.z = 0;

        vels.linear = lin;
        vels.angular = ang;
        VEL.publish(vels);
    }


}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg -> pose.pose.position.x;
    y = msg -> pose.pose.position.y;

    distance = distance + sqrt( (x - x_old) * (x - x_old) + (y - y_old) * (y - y_old) );

    x_old=x;
    y_old=y;

    orientation = msg -> pose.pose.orientation.z;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "coll_avoid");
    ros::NodeHandle n;

	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber intensity_sub = n.subscribe("intensity_sensor", 1, intensityCallback);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);

	int hz = 10;
	float periodo = (float)1/hz;
    ros::Rate loop_rate(hz); //  publish rate

    float t = 0.0;

    while(ros::ok())//fino a che ros non viene fermato
    {	
    	//Aggiornamento del timer
    	t = t + periodo;

    	geometry_msgs::Twist vels;
    	geometry_msgs::Vector3 lin, ang; //Velocità angolare e lineare

    	if (color == 'g')
		{
            //Go
	    	lin.x = 0.1;
	    	lin.y = 0;
	    	lin.z = 0;
	    	ang.x = 0;
	    	ang.y = 0;
	    	ang.z = 0;

            vels.linear = lin;
            vels.angular = ang;
            vel_pub.publish(vels);

            ROS_INFO("...searching...");

		}


    	else
    	{  
            ROS_INFO("Threshold found: entering into next square");

            // Ho trovato la linea bianca: avanzo di 20 cm

            distance = 0;
            while (distance < 0.20)
            {
                //Entro con l'intero robot nella casella
                lin.x = 0.1;
                lin.y = 0;
                lin.z = 0;
                ang.x = 0;
                ang.y = 0;
                ang.z = 0;

                vels.linear = lin;
                vels.angular = ang;
                vel_pub.publish(vels);

                ROS_INFO("Percorsi: %.4g cm", distance);

                ros::spinOnce();

                loop_rate.sleep();
            }

            // Mi fermo
            robotStop(vel_pub);

            // TIMEOUT: adesso ho sorpassato la soglia. Mi fermo, anche se il sensore ora vede VERDE
            ROS_INFO("Entered into a new square. Task finished.");
            break;


	 	}

    	ros::spinOnce();
    	loop_rate.sleep();



    	// PROVA
        /*
    	Plotdata x(-3.0, 3.0), y = sin(x) - 0.5*x;
        plot(x, y);
        */


        //return 0;

    }


    return 0;

}

