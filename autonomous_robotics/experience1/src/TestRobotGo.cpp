
#include <iostream>
#include <esperienza1/Mappa.h>
#include "esperienza1/navigate_nxt.h"
#include "ros/ros.h"

using namespace std;

int main(int argc, char **argv)
	{
		
		// ******** Inizializzo ROS *****************
		ros::init(argc, argv, "rotate");
		ros::NodeHandle n;
		Navigate_Nxt nav(n);
		
		//while(ros::ok())//fino a che ros non viene fermato
		//{
		
		//nav.center();			
		
		

		
		// ************ Parte di Mapping *******************
		const double SQUARE_SIZE = 0.29;
		
		// Set grandezza mappa, Start, Goal
		int X = 3, Y = 3;
		Mappa m( X, Y );
				
		// Set Ostacoli
		//m.Mappa::setOstacolo(1,1);
		//m.Mappa::setOstacolo(0,0);
		
		// Set Goal
		m.setArrivo( 1, 2 );
		
		// Set Start
		int x = 1, y = 0;
		

		/*for(int i = 0; i < Y; ++i)
		 {
		 for(int j = 0; j < X; ++j)
		 cout << m.valoreCella(j, i) << "   ";
		 cout << endl;
		 };

		 cout << endl;
		 cout << endl;*/

		// stampa mappa
		for( int i = Y - 1; i >= 0; --i )
			{
				for( int j = 0; j < X; ++j )
					cout << m.distanzaCella( j, i ) << "   ";
				cout << endl;
			};

	    char d;
	    
	    
	    //nav.rotate('r');
		nav.robotGo();
		//nav.center();
		ROS_INFO("Stop");
		
		
		/*while(m.distanzaCella(x,y) != 0)
			{
				//findCones(x, y, m ); // setto i blocchi
				d = m.nuovaDirezione(x, y);
				cout << d << endl;
				x = d=='e' ? x+1 : x;
				x = d=='o' ? x-1 : x;
				y = d=='n' ? y+1 : y;
				y = d=='s' ? y-1 : y;
				cout << x << ", " << y << endl << endl;
				switch(d){
					case ('e'):
						nav.rotate('r');
						nav.robotGo();
						cout << "Vado a est" << endl;
						break;
					case ('o'):
						nav.rotate('l');
						nav.robotGo();
						cout << "Vado a ovest" << endl;
						break;
					case ('n'):
						nav.rotate('f');
						nav.robotGo();
						cout << "Vado a nord" << endl;
						break;
					case ('s'):
						nav.rotate('b');
						nav.robotGo();
						cout << "Vado a sud" << endl;
						break;		
				}
	
				if(d == 'f'){
						cout << "Sono fuori dalla mappa, cos'è successo?!? :'(" << endl;
						return -1;
					}
					
			} */

		 // m.nuovaDirezione(x, y);
		// cout << x << ", " << y << endl << endl;



		ros::spin();
		ros::shutdown();
		
		return 0;
	}
;

