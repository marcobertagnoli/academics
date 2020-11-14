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
    if (msg->r < 0.3)  
    {
        color = 'w';
    }
    else
    {
        color = 'g';
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

    	if (color == 'w')
    	{  
            ROS_INFO("Found first Limit...centering...");
    		//Stop
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
                vel_pub.publish(vels);
            }

            //Allineamento
            while( orientation > 0.05 && orientation < -0.05)
            {
                if(orientation > 0)
                {
                    lin.x = 0;
                    lin.y = 0;
                    lin.z = 0;
                    ang.x = 0;
                    ang.y = 0;
                    ang.z = -1;

                    vels.linear = lin;
                    vels.angular = ang;
                    vel_pub.publish(vels);
                }
                else
                {
                    lin.x = 0;
                    lin.y = 0;
                    lin.z = 0;
                    ang.x = 0;
                    ang.y = 0;
                    ang.z = 1;

                    vels.linear = lin;
                    vels.angular = ang;
                    vel_pub.publish(vels);
                }
            }

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
                vel_pub.publish(vels);
            }
            

            distance=0;

            while (distance < 0.10)
            {
                //Movimento fino al centro
                lin.x = -0.1;
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

            //Stop
            lin.x = 0;
            lin.y = 0;
            lin.z = 0;
            ang.x = 0;
            ang.y = 0;
            ang.z = 0;

            vels.linear = lin;
            vels.angular = ang;
            vel_pub.publish(vels);

            ROS_INFO("Centered...searching second limit...");


	 	}
    	else
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

    	// vels.linear = lin;
    	// vels.angular = ang;
    	// ROS_INFO("time: %f",t);
    	// ROS_INFO("Linear velocity: %.4g", lin.x);
     //    ROS_INFO("Angular velocity: %.4g", ang.z);

    	// vel_pub.publish(vels);

    	ros::spinOnce();

    	loop_rate.sleep();

    }


    return 0;

}