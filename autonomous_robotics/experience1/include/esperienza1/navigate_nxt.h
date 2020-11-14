#ifndef NAVIGATE_NXT_H_
#define NAVIGATE_NXT_H_

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


class Navigate_Nxt
{
	public:
		Navigate_Nxt(ros::NodeHandle&);
		//virtual ~NavigateNxt();
		void rotate(char); // o: senso orario; a:senso antiorario....2a implementazione: l=orienta a sinistra, r=orienta a destra, f=guarda avanti, b=guarda dietro;
		void robotGoStraight(double);
		void robotGo();
		void center();
		std::vector<bool> findCones(int, int); //prende in ingresso la mappa e la posizione attuale quindi setta gli ostacoli vicini
	private:
		char color;
		double rotation;
		double orientation;
		double orientation_old;
		double distance;
		double x,y;
		double x_old;
		double y_old;
		bool odom_first_callback;
		double velocity_joint; // = {};
		float range;
		double central_joint;

	// 	void odomCallback(const nav_msgs::Odometry::ConstPtr&)
		ros::NodeHandle node;
		void robotStop(const ros::Publisher& VEL, ros::Rate& loop_rate);
		char actual_orientation;
		void intensityCallback(const nxt_msgs::Color::ConstPtr& msg);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
		void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
		void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
		void ultrasCallback(const nxt_msgs::Range::ConstPtr& msg);
		
		ros::Publisher vel_pub;
		ros::Subscriber odom_sub;
		ros::Subscriber intensity_sub;
		ros::Publisher angle_pub;
        ros::Subscriber joints_sub;


		// double rotation;
		// double orientation_old;

};

#endif /* NAVIGATE_NXT_H_ */
