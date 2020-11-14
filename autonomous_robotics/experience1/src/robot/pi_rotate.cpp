#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include <sstream>


double rotation=0;
double orientation_old=0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double orientation = msg -> pose.pose.orientation.z;

    rotation = rotation + orientation - orientation_old;

    orientation_old = orientation;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "coll_avoid");
    ros::NodeHandle n;

	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    ros::Subscriber odom_sub = n.subscribe("odom", 0, odomCallback);

	int hz = 10;
	float periodo = (float)1/hz;
    ros::Rate loop_rate(hz); //  publish rate

    float t = 0.0;

    geometry_msgs::Twist vels;
    geometry_msgs::Vector3 lin, ang;

    while(rotation < 0.15)
    {	
        t = t + periodo;
         
        lin.x = 0;
        lin.y = 0;
        lin.z = 0;
        ang.x = 0;
        ang.y = 0;
        ang.z = 0.5*t;

        vels.linear = lin;
        vels.angular = ang;
        vel_pub.publish(vels);

    	ros::spinOnce();
    	loop_rate.sleep();
    }

    while(rotation < 0.55)
    {   

        vel_pub.publish(vels);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // t = 0;

    // double ang_vel = ang.z;

    // while(ang.z != 0)
    // {
    //     t = t + periodo;
         
    //     lin.x = 0;
    //     lin.y = 0;
    //     lin.z = 0;
    //     ang.x = 0;
    //     ang.y = 0;
    //     ang.z = ang_vel - 0.5*t;

    //     vels.linear = lin;
    //     vels.angular = ang;
    //     vel_pub.publish(vels);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    lin.x = 0;
    lin.y = 0;
    lin.z = 0;
    ang.x = 0;
    ang.y = 0;
    ang.z = 0;

    vels.linear = lin;
    vels.angular = ang;
    vel_pub.publish(vels);
        


    ros::spinOnce();

    loop_rate.sleep();


    return 0;

}