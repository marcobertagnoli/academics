#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include <ros/package.h>

#include "esperienza1/Direzione.h"
#include "esperienza1/getPosition.h"
#include "esperienza1/GetPesi.h"
#include "esperienza1/setOstacolo.h"
#include "esperienza1/rimuoviOstacolo.h"
#include "esperienza1/drawSpostamento.h"
#include "esperienza1/drawRicerca.h"

#include <cstdlib>
#include <string>
#include <esperienza1/Mappa.h>
#include "esperienza1/navigate_nxt.h"

using namespace std;
using namespace cv;

int main( int argc, char **argv )
	{
		ros::init( argc, argv, "robot" );     //nome nodo

		ros::NodeHandle n;
		// Client per il servizio Direzione
		ros::ServiceClient client = n.serviceClient< esperienza1::Direzione >( "navigation_direzione" );
		esperienza1::Direzione srv;
		// Client per il servizio getPosition
		ros::ServiceClient client1 = n.serviceClient< esperienza1::getPosition >( "navigation_getPosition" );
		esperienza1::getPosition srv1;
		// Client per il servizio setOstacolo
		ros::ServiceClient client2 = n.serviceClient< esperienza1::setOstacolo >( "mapping_setOstacolo" );
		esperienza1::setOstacolo srv2;
		// Client per il servizio rimuoviOstacolo
		ros::ServiceClient client3 = n.serviceClient< esperienza1::rimuoviOstacolo >( "mapping_rimuoviOstacolo" );
		esperienza1::rimuoviOstacolo srv3;
		// Client per il servizio drawSpostamento
		ros::ServiceClient client4 = n.serviceClient< esperienza1::drawSpostamento >( "mapping_drawSpostamento" );
		esperienza1::drawSpostamento srv4;
		// Client per il servizio drawRicerca
		ros::ServiceClient client5 = n.serviceClient< esperienza1::drawRicerca >( "mapping_drawRicerca" );
		esperienza1::drawRicerca srv5;

		//Check su terzo punto esperienza e goal
		ROS_INFO( "Initializing the map..." );
		string path = ros::package::getPath( "esperienza1" );

		path = path + "/src/mappa.yaml";
		FileStorage fs;
		fs.open( path, FileStorage::READ );
		//Check sul file
		if( !fs.isOpened() )
			{
				cerr << "Failed to open " << path << endl;
				return 1;
			}
		bool TerzoPunto;
		fs["TerzoPunto"] >> TerzoPunto;
		cout << TerzoPunto << endl;
		bool drawMap;
		fs["DrawMap"] >> drawMap;

		int x_goal;
		int y_goal;
		cv::FileNode fs_node;
		fs_node = fs["Arrivo"];
		fs_node["x"] >> x_goal;
		fs_node["y"] >> y_goal;
		fs.release();

		Navigate_Nxt nav( n );

		char direction = 'f';     //entra nel while

		while( direction != 'a' )
			{
				if( true )  //TerzoPunto
					{
						if( client1.call( srv1 ) )
							{
								std::vector< bool > Cones;

								if( ( srv1.response.x != x_goal ) && ( srv1.response.y != y_goal ) )
									{
										//Aggiorno l'immagione della mappa
										if( drawMap )
											{
												srv5.request.x = srv1.response.x;
												srv5.request.y = srv1.response.y;
												client5.call( srv5 );
											}
											
										ROS_INFO( "FIND CONES" );
										Cones = nav.findCones( srv1.response.x, srv1.response.y );

										for( int i = 0; i < 4; ++i )
											{
												if( Cones[i] )
													{
														switch( i )
															{
															case ( 0 ):
																srv2.request.x = srv1.response.x;
																srv2.request.y = srv1.response.y - 1;
																if( client2.call( srv2 ) )
																	{
																		ROS_INFO( "Aggiunto ostacolo in: ( %d, %d )", srv1.response.x, ( srv1.response.y - 1 ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_setOstacolo" );
																		return 1;
																	}
															case ( 1 ):
																srv2.request.x = srv1.response.x + 1;
																srv2.request.y = srv1.response.y;
																if( client2.call( srv2 ) )
																	{
																		ROS_INFO( "Aggiunto ostacolo in: ( %d, %d )", ( srv1.response.x + 1 ), ( srv1.response.y ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_setOstacolo" );
																		return 1;
																	}
															case ( 2 ):
																srv2.request.x = srv1.response.x;
																srv2.request.y = srv1.response.y + 1;
																if( client2.call( srv2 ) )
																	{
																		ROS_INFO( "Aggiunto ostacolo in: ( %d, %d )", srv1.response.x, ( srv1.response.y + 1 ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_setOstacolo" );
																		return 1;
																	}
															case ( 3 ):
																srv2.request.x = srv1.response.x - 1;
																srv2.request.y = srv1.response.y;
																if( client2.call( srv2 ) )
																	{
																		ROS_INFO( "Aggiunto ostacolo in: ( %d, %d )", ( srv1.response.x - 1 ), ( srv1.response.y ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_setOstacolo" );
																		return 1;
																	}

															}
													}
												else
													{
														switch( i )
															{
															case ( 0 ):
																srv3.request.x = srv1.response.x;
																srv3.request.y = srv1.response.y - 1;
																if( client3.call( srv3 ) )
																	{
																		ROS_INFO( "Rimosso ostacolo in: ( %d, %d )", srv1.response.x, ( srv1.response.y - 1 ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_rimuoviOstacolo" );
																		return 1;
																	}
															case ( 1 ):
																srv3.request.x = srv1.response.x + 1;
																srv3.request.y = srv1.response.y;
																if( client3.call( srv3 ) )
																	{
																		ROS_INFO( "Rimosso ostacolo in: ( %d, %d )", ( srv1.response.x + 1 ), ( srv1.response.y ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_rimuoviOstacolo" );
																		return 1;
																	}
															case ( 2 ):
																srv3.request.x = srv1.response.x;
																srv3.request.y = srv1.response.y + 1;
																if( client3.call( srv3 ) )
																	{
																		ROS_INFO( "Rimosso ostacolo in: ( %d, %d )", srv1.response.x, ( srv1.response.y + 1 ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_rimuoviOstacolo" );
																		return 1;
																	}
															case ( 3 ):
																srv3.request.x = srv1.response.x - 1;
																srv3.request.y = srv1.response.y;
																if( client3.call( srv3 ) )
																	{
																		ROS_INFO( "Rimosso ostacolo in: ( %d, %d )", ( srv1.response.x - 1 ), ( srv1.response.y ) );
																	}
																else
																	{
																		ROS_ERROR( "Failed to call service mapping_rimuoviOstacolo" );
																		return 1;
																	}
															}
													}
											}
									}
							}
						else
							{
								ROS_ERROR( "Failed to call service get_position" );
								return 1;
							}
					}

				if( client.call( srv ) )
					{
						direction = srv.response.direction;

						if( drawMap )
							{
								client1.call( srv1 );

								srv4.request.x = srv1.response.x;
								srv4.request.y = srv1.response.y;
								client4.call( srv4 );

								// switch( direction )
								// 	{
								// 	case ( 'e' ):
								// 		srv4.request.x = srv1.response.x + 1;
								// 		srv4.request.y = srv1.response.y;
								// 		client4.call( srv4 );
								// 		break;
								// 	case ( 'o' ):
								// 		srv4.request.x = srv1.response.x - 1;
								// 		srv4.request.y = srv1.response.y;
								// 		client4.call( srv4 );
								// 		break;
								// 	case ( 'n' ):
								// 		srv4.request.x = srv1.response.x;
								// 		srv4.request.y = srv1.response.y + 1;
								// 		client4.call( srv4 );
								// 		break;
								// 	case ( 's' ):
								// 		srv4.request.x = srv1.response.x;
								// 		srv4.request.y = srv1.response.y - 1;
								// 		client4.call( srv4 );
								// 		break;
								// 	}
							}

						ROS_INFO( "Robot rotating %c", direction );
						switch( direction )
							{
							case ( 'e' ):
								nav.rotate( 'r' );
								nav.robotGo();
								std::cout << "Vado a est" << std::endl;
								break;
							case ( 'o' ):
								nav.rotate( 'l' );
								nav.robotGo();
								std::cout << "Vado a ovest" << std::endl;
								break;
							case ( 'n' ):
								nav.rotate( 'f' );
								nav.robotGo();
								std::cout << "Vado a nord" << std::endl;
								break;
							case ( 's' ):
								nav.rotate( 'b' );
								nav.robotGo();
								std::cout << "Vado a sud" << std::endl;
								break;
							}

					}
				else
					{
						ROS_ERROR( "Failed to call service direction" );
						return 1;
					}
			}

		ROS_INFO( "FINE" );

		return 0;
	}
