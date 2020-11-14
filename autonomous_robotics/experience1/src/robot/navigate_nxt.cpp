#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "esperienza1/navigate_nxt.h"
#include <math.h>
#include "nxt_msgs/Color.h"
#include "sensor_msgs/JointState.h" 
#include "std_msgs/Float64.h"
#include <esperienza1/Mappa.h>
#include "nxt_msgs/Range.h"
#include "std_msgs/String.h"


Navigate_Nxt::Navigate_Nxt(ros::NodeHandle& n)
{
    node = n;
    actual_orientation = 'f';
    color = 'g';
	rotation = 0;
	orientation = 0;
	orientation_old = 0;
	distance = 0;
	x_old = 0;
	y_old = 0;
	odom_first_callback = false; 
	
	vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    odom_sub = node.subscribe("odom", 0, &Navigate_Nxt::odomCallback, this);
    intensity_sub = node.subscribe("intensity_sensor", 0, &Navigate_Nxt::intensityCallback, this);
    angle_pub = node.advertise<std_msgs::Float64>("angle", 0);
    joints_sub = node.subscribe("joint_states", 0, &Navigate_Nxt::jointStatesCallback, this);

}

void Navigate_Nxt::robotStop(const ros::Publisher& VEL, ros::Rate& loop_rate )
{
    geometry_msgs::Twist vels;
    geometry_msgs::Vector3 lin, ang;

    for (int i = 0; i < 5; ++i)
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

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Navigate_Nxt::intensityCallback(const nxt_msgs::Color::ConstPtr& msg)
{
    if (msg->r < 0.45)  
    {
        color = 'w';
    }
    else
    {
        color = 'g';
    }
}



void Navigate_Nxt::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	
    orientation = msg -> pose.pose.orientation.z;
    
    x = msg -> pose.pose.position.x;
    y = msg -> pose.pose.position.y;
    
    if (odom_first_callback){
		orientation_old = orientation;
		x_old=x;
		y_old=y;
		odom_first_callback = false;
	
	}
	    
    rotation = rotation + fabs(orientation - orientation_old);
    distance = distance + sqrt( (x - x_old) * (x - x_old) + (y - y_old) * (y - y_old) );
    
    orientation_old = orientation;
	x_old=x;
	y_old=y;
    
}

void Navigate_Nxt::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	velocity_joint = msg -> velocity.at(0);
    //ROS_INFO("Vel_joint %.4g", velocity_joint);
}

void Navigate_Nxt::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	central_joint = msg -> position.at(2);
}

//void ultrasCallback(const std_msgs::Float64::ConstPtr& msg)
void Navigate_Nxt::ultrasCallback(const nxt_msgs::Range::ConstPtr& msg)
{
    range = msg -> range;
}

void Navigate_Nxt::rotate(char direction)
{   
    rotation = 0;
    odom_first_callback = true;
    ros::spinOnce();

    //ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    //ros::Subscriber odom_sub = node.subscribe("odom", 0, odomCallback);

    int hz = 5;
    float periodo = (float)1/hz;
    ros::Rate loop_rate(hz); //  publish rate

    float t = 0.0;

    geometry_msgs::Twist vels;
    geometry_msgs::Vector3 lin, ang;

    double target;
    double soglia;

    //Calcolo orientazione desiderate
    if (direction == 'f')
    {
        target = 0;
        soglia = 0.007; //0.007
    }
    else if (direction == 'b')
    {
        if (orientation <= 0)
        {
            target = -0.98; //0.9963

            soglia = 0.0003;  //0.0005
        }
        else
        {
            target = 0.98;   //0.9963

            soglia = 0.0003;  //0.0005
        }
    }
    else if (direction == 'l')
    {
        target = 0.66;     //0.669

        soglia = 0.003;    //0.003
    }
    else if (direction == 'r')
    {
        target = -0.66; //-0.669

        soglia = 0.003;    //0.003
    }
    else
    {
        ROS_INFO("Error in NavigateNxt::rotate(char): wrong input");
        return;
    }

	if ( actual_orientation == 'b' && direction == 'l' && orientation < 0 )
				{
					while(orientation < 0 )
						{   
							t = t + periodo;
							 
							lin.x = 0;
							lin.y = 0;
							lin.z = 0;
							ang.x = 0;
							ang.y = 0;
							ang.z = fmin (0.25 * t, 1.5);
							 //ROS_INFO("tempo %f", t);    
							 ROS_INFO("VelRotazione_z %.4g", ang.z);
					
							vels.linear = lin;
							vels.angular = ang;
							vel_pub.publish(vels);

							ros::spinOnce();
							loop_rate.sleep();

							}
					
				}
	if ( actual_orientation == 'b' && direction == 'r' && orientation > 0 )
				{
					while(orientation > 0 )
						{   
							t = t + periodo;
							 
							lin.x = 0;
							lin.y = 0;
							lin.z = 0;
							ang.x = 0;
							ang.y = 0;
							ang.z = fmin (-0.25 * t, -1.5);
							 //ROS_INFO("tempo %f", t);    
							 ROS_INFO("VelRotazione_z %.4g", ang.z);
					
							vels.linear = lin;
							vels.angular = ang;
							vel_pub.publish(vels);

							ros::spinOnce();
							loop_rate.sleep();

							}
					
				}
	
	
    double difference = target - orientation;
   // ROS_INFO("Differenza %.4g", difference);

    while(rotation < 0.15 && fabs(difference) > 0.1)
    {   
        t = t + periodo;
         
        lin.x = 0;
        lin.y = 0;
        lin.z = 0;
        ang.x = 0;
        ang.y = 0;

        if(difference > 0)
        {
            ang.z = 0.25 * t;
         //ROS_INFO("tempo %f", t);    
         ROS_INFO("VelRotazione_z %.4g", ang.z);
        }
        else
        {
            ang.z = - 0.25 * t;
           // ROS_INFO("tempo %f", t);
            ROS_INFO("VelRotazione_z %.4g", ang.z);
        }

        vels.linear = lin;
        vels.angular = ang;
        vel_pub.publish(vels);

        ros::spinOnce();
        loop_rate.sleep();

        difference = target - orientation;
        ROS_INFO("Differenza %.4g", difference);
    }

    while(fabs(difference) > 0.1)
    {
        if(difference * vels.angular.z < 0)
            break;
        
        vel_pub.publish(vels);

        ros::spinOnce();
        loop_rate.sleep();
        difference = target - orientation;
        ROS_INFO("Differenza %.4g", difference);
    }


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

    //Aggiustamento posizione
    t = 0;
    difference = target - orientation;
  //  ROS_INFO("Differenza %.4g", difference);

    while(fabs(difference) > soglia)
    {
        if (difference > 0)
        {
            if(difference * vels.angular.z < 0)
            {
                t = 0;
            }

            vels.angular.z = fmin(0.1 * t + 0.5, 1.5);
           // ROS_INFO("Velocità: %.4g", vels.angular.z);
        }
        else
        {
            if(difference * vels.angular.z < 0)
            {
                t = 0;
            }

            vels.angular.z = fmax(-0.1 * t - 0.5, -1.5);
          //  ROS_INFO("Velocita': %.4g", vels.angular.z);
        }
            
        vel_pub.publish(vels);

        ros::spinOnce();
        loop_rate.sleep();

        if(direction == 'b' && orientation < 0)
        {
            target = -0.98;
        }
        else if (direction == 'b' && orientation > 0)
        {
            target = 0.98;
        }


        difference = target - orientation;
      //  ROS_INFO("Differenza %.4g", difference);
        t = t + periodo;
    }
    
    ROS_INFO("Rotazione completa 1");

    lin.x = 0;
    lin.y = 0;
    lin.z = 0;
    ang.x = 0;
    ang.y = 0;
    ang.z = 0;

    vels.linear = lin;
    vels.angular = ang;
    vel_pub.publish(vels);

    actual_orientation = direction;

    ros::spinOnce();
    loop_rate.sleep();

}


void Navigate_Nxt::robotGoStraight(double length)
{	
	odom_first_callback = true;
	
	ROS_INFO("RobotGOStraight .. ");
	distance = 0;
	
		ros::spinOnce();
		orientation_old = orientation;
		x_old=x;
		y_old=y;
	
	
	//ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    //ros::Publisher vel_pub = VV;

	//ros::Subscriber odom_sub = node.subscribe("odom", 0, Navigate_Nxt::odomCallback&, this);

	int hz = 5;
	float periodo = (float)1/hz;  
    ros::Rate loop_rate(hz); //  publish rate

    float t = 0.0; 

    geometry_msgs::Twist vels;
    geometry_msgs::Vector3 lin, ang;
   // ROS_INFO("(INIZIO) Percorsi: %.4g cm", distance);
   // ROS_INFO("(INIZIO) tempo %f s", t);
 //   ROS_INFO("(INIZIO) vel %f m/s", lin.x);
    ROS_INFO("1. Prima del while ");
    
    //while((lin.x < 0.15))  //finchè non ho raggiunto i 0.15 m/s
    while(distance < length) 
    {	
	//	ROS_INFO("(MID) Percorsi: %.4g cm", distance);
   // 	ROS_INFO("(MID) tempo %f s", t);
    //	ROS_INFO("(MID) vel %f m/s", lin.x);
		ROS_INFO("(0.10) Percorsi: %.4g cm", distance);

        t = t + periodo;
         
        lin.x = fmin(0.05*t, 0.12);
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
    	
    }

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

   // ROS_INFO("(0.10) Percorsi: %.4g cm", distance);
}

void Navigate_Nxt::robotGo()
{
    Navigate_Nxt nav(node);

    //ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    //ros::Subscriber intensity_sub = node.subscribe("intensity_sensor", 0, intensityCallback); // SI PUO TOGLIERE?

    int hz = 5;
    float periodo = (float)1/hz;  
    ros::Rate loop_rate(hz); //  publish rate

    geometry_msgs::Twist vels;
    geometry_msgs::Vector3 lin, ang;

    float t = 0.0; 

    ROS_INFO("Searching the line...");

    //find the line
    while(color == 'g')
    {
        t = t + periodo;
         
        lin.x = fmin(0.05*t, 0.12);
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
    }

   ROS_INFO("Line founded");

   nav.robotGoStraight(0.15);


    // lin.x = 0;
    // lin.y = 0;
    // lin.z = 0;
    // ang.x = 0;
    // ang.y = 0;
    // ang.z = 0;

    // vels.linear = lin;
    // vels.angular = ang;
    // vel_pub.publish(vels);

    // ros::spinOnce();
    // loop_rate.sleep();
}

void Navigate_Nxt::center()
{
    Navigate_Nxt nav(node);

    nav.rotate('f');

    //ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    //ros::Subscriber intensity_sub = node.subscribe("intensity_sensor", 0, intensityCallback);

    int hz = 5;
    float periodo = (float)1/hz;  
    ros::Rate loop_rate(hz); //  publish rate

    geometry_msgs::Twist vels;
    geometry_msgs::Vector3 lin, ang;

    float t = 0.0; 

    ROS_INFO("Searching the line...");

    //find the line
    while(color == 'g')
    {
        t = t + periodo;
         
        lin.x = fmin(0.05*t, 0.13);
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
    }

   // ROS_INFO("Found line, tourning back...");


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

    nav.rotate('b');

    ////ROS_INFO("Centering first direction...");

    nav.robotGoStraight(0.08);

    //ROS_INFO("Searching line in second direction...");

    nav.rotate('r');

    while(color == 'g')
    {
        t = t + periodo;
         
        lin.x = fmin(0.2*t, 0.13);
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
    }

    //ROS_INFO("Found line, tourning back...");


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

    nav.rotate('l');

    //ROS_INFO("Centering second direction...");

    nav.robotGoStraight(0.1);

    nav.rotate('f');
    nav.rotate('f');
}


std::vector<bool> Navigate_Nxt::findCones(int x, int y)
{
    //Navigate_Nxt nav(node);

    std::vector<bool> Cones;

    double detection_distance = 0.4;

    //ros::Publisher angle_pub = node.advertise<std_msgs::Float64>("angle", 0);
    //ros::Subscriber intensity_sub = node.subscribe("ultrasonic_sensor", 0, ultrasCallback);
    //ros::Subscriber joints_sub = node.subscribe("joint_states", 0, jointStatesCallback);
    
    //Inserire subscriber per giunto centrale

    int hz = 5;
    ros::Rate loop_rate(hz); //  publish rate

    //nav.rotate(actual_orientation);

    ros::spinOnce();

	std_msgs::Float64 msg;

    double f = 0.0; 
    double f_joint = 0;
    double b = 195.0;
    double b_joint = 9.72;
    double r = 95.0;
    double r_joint = 4.57;
    double l = -105.0;
    double l_joint = -5.07;
    double tollerance = 0.3;


	ROS_INFO(" ORIENTAZIONE %c", actual_orientation);
	ros::spinOnce();
	
	
    switch(actual_orientation)
    {
		case('f'):
        {   
			ROS_INFO(" CASE F, in ");
			
			// di default tutti i blocchi sono settati a zero; percio il metodo possoAndare
			// serve soltanto a tenere il robot all'interno della mappa
			
			msg.data = b;
			
			//ROS_INFO(" central_joint %f", central_joint);
			//ROS_INFO(" (b_joint - tollerance) %f", (b_joint - tollerance));
			//ROS_INFO(" (b_joint + tollerance) %f", (b_joint + tollerance));
			
			
            while((central_joint < (b_joint - tollerance)) || (central_joint > (b_joint + tollerance)))
            { 
				angle_pub.publish(msg);
                //ROS_INFO(" 1");
                loop_rate.sleep();
				ros::spinOnce();
            }
			
            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = r;

			
            while((central_joint < (r_joint - tollerance)) || (central_joint > (r_joint + tollerance)))
            { 
				angle_pub.publish(msg);
                 //ROS_INFO(" 2");
                
                loop_rate.sleep();
                ros::spinOnce();
            };

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = f;
            while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            { 
				angle_pub.publish(msg);
                //ROS_INFO(" 3");
                
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
			
			msg.data = l;
			
            while((central_joint < (l_joint - tollerance)) || (central_joint > (l_joint + tollerance)))
            { 
				angle_pub.publish(msg);
                //ROS_INFO(" 4");
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
			
			msg.data = f;
			// raddrizzo
			while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            {
				angle_pub.publish(msg); 
                loop_rate.sleep();
				ros::spinOnce();
            }
            ROS_INFO(" CASE F, OUT ");
            break;
        }

        case('b'):
        {   
			ROS_INFO(" CASE b, in ");
			msg.data = f;
            while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = l;
            while((central_joint < (l_joint - tollerance)) || (central_joint > (l_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = b;
            while((central_joint < (b_joint - tollerance)) || (central_joint > (b_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = r;
            while((central_joint < (r_joint - tollerance)) || (central_joint > (r_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);


			msg.data=f;
			
			while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }
            break;
            ROS_INFO(" CASE b, O ");
        }    
        
        case('l'):
        {   
			msg.data = l;
            while((central_joint < (l_joint - tollerance)) || (central_joint > (l_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }
            
            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = b;
            while((central_joint < (b_joint - tollerance)) || (central_joint > (b_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = r;
            while((central_joint < (r_joint - tollerance)) || (central_joint > (r_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = f;
            while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
            break;
        }

        case('r'):
        {   
			msg.data = r;
            while((central_joint < (r_joint - tollerance)) || (central_joint > (r_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = f;
            while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = l;
            while((central_joint < (l_joint - tollerance)) || (central_joint > (l_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = b;
            while((central_joint < (b_joint - tollerance)) || (central_joint > (b_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            }
            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
            msg.data = f;
			while((central_joint < (f_joint - tollerance)) || (central_joint > (f_joint + tollerance)))
            { 
                angle_pub.publish(msg);
                loop_rate.sleep();
				ros::spinOnce();
            };
            break;
        } 

    }

    return Cones;
} 



/*
std::vector<bool> Navigate_Nxt::findCones(int x, int y)
{
    Navigate_Nxt nav(node);

    std::vector<bool> Cones;

    double detection_distance = 0.4;

    ros::Publisher angle_pub = node.advertise<std_msgs::Float64>("angle", 0);
    ros::Subscriber intensity_sub = node.subscribe("ultrasonic_sensor", 0, ultrasCallback);

    int hz = 5;
    ros::Rate loop_rate(hz); //  publish rate

    nav.rotate(actual_orientation);

    ros::spinOnce();

	std_msgs::Float64 msg;

    double f = 0.0; 
    double b = 195.0;
    double r = 95.0;
    double l = -110.0;


	
    switch(actual_orientation)
    {
		case('f'):
        {   
			// di default tutti i blocchi sono settati a zero; percio il metodo possoAndare
			// serve soltanto a tenere il robot all'interno della mappa
			
			msg.data = b;
            angle_pub.publish(msg); //sud
            ros::spinOnce();
            loop_rate.sleep();
			
            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = r;
            angle_pub.publish(msg); //est
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = f;
            angle_pub.publish(msg); // nord
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
			
			msg.data = l;
            angle_pub.publish(msg);  // ovest
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
			
			msg.data = 0;
            angle_pub.publish(msg);
            loop_rate.sleep();
            
        }

        case('b'):
        {   
			msg.data = f;
            angle_pub.publish(msg); // nord
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = l;
            angle_pub.publish(msg); // ovest
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = b;
            angle_pub.publish(msg); // sud
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = r;
            angle_pub.publish(msg);  // est
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = 0;
            angle_pub.publish(msg);
            loop_rate.sleep();
        }    
        
        case('l'):
        {   
			msg.data = l;
            angle_pub.publish(msg); //est
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);

			msg.data = b;
            angle_pub.publish(msg); //nord
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = r;
            angle_pub.publish(msg); // ovest
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = f;
            angle_pub.publish(msg);  // sud
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
        }

        case('r'):
        {   
			msg.data = r;
            angle_pub.publish(msg); // ovest
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = f;
            angle_pub.publish(msg); // sud
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = l;
            angle_pub.publish(msg); // est
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = b;
            angle_pub.publish(msg);  // nord
            ros::spinOnce();
            loop_rate.sleep();

            if(range < detection_distance)
            {
                Cones.push_back(true);
            }
            else
                Cones.push_back(false);
            
			msg.data = f;
            angle_pub.publish(msg);
            loop_rate.sleep();
        } 

    }

    return Cones;
} 

*/
