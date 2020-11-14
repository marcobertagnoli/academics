#include <iostream>
#include <limits.h>

#include "ros/ros.h"
#include <ros/package.h>

#include "esperienza1/Direzione.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Robot");

  ros::NodeHandle n;
  // Si crea il client per il servizio add_two_ints
  ros::ServiceClient client = n.serviceClient<esperienza1::Direzione>("navigation_direzione");
  esperienza1::Direzione srv;

  char direction = 'i'; //Esegue la richiesta almeno una volta


  while(true)
  {  
    if (client.call(srv))
    {
      direction = srv.response.direction;

      if(direction == 'a')
        break;

      cout << endl << endl << "Mi sposto verso: " << direction << endl;
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
  }

  cout << endl << "ARRIVATO" << endl << endl;

  return 0;
}