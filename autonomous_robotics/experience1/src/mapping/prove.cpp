/*
 * prove.cpp
 *
 *  Created on: Apr 13, 2016
 *      Author: a
 */

#include <iostream>
#include <limits.h>
#include <opencv2/core/core.hpp>

using namespace std;

char nuovaDirezione( int a, int n, int e, int s, int o )
	{
		cout << "dentro nuovo 'nuovaDirezione'" << a << " " << n << " " << e << " " << s << " " << o << endl;
		char d = 'f';
		if( a == 0 )
			return 'a';

		n = ( n < 0 ) ? INT_MAX : n;     //se sono negativi vuol dire che non posso andare e metto "infinito"
		e = ( e < 0 ) ? INT_MAX : e;
		s = ( s < 0 ) ? INT_MAX : s;
		o = ( o < 0 ) ? INT_MAX : o;

		cout << "dentro nuovo 'nuovaDirezione' 2 " << a << " " << n << " " << e << " " << s << " " << o << endl;

		if( ( e <= n ) && ( e <= s ) && ( e <= o ) )
			{
				d = 'e';
			}

		if( ( s <= n ) && ( s <= e ) && ( s <= o ) )
			{
				d = 's';
			}

		if( ( o <= n ) && ( o <= s ) && ( o <= e ) )
			{
				d = 'o';
			}

		if( ( n <= o ) && ( n <= s ) && ( n <= e ) )
			{
				d = 'n';
			}
		return d;
	}

void disegnaMappa( int X, int Y, Mappa& m )
	{
		int D;

		for( int i = Y - 1; i >= 0; --i )
			{
				for( int j = 0; j < X; ++j )
					{
						D = m.distanzaCella( j, i );

						if( D < 0 )
							cout << " X   ";
						else
							{
								if( D > 9 )
									cout << D << "   ";
								else
									cout << " " << D << "   ";
							}

					}

				cout << endl;
			};
	}

void nuoviBlocchi( Mappa& m )
	{
		int x, y;
		do
			{
				cout << "ci sono blocchi????" << endl;
				cin >> x >> y;
				if( x > -1 && y > -1 )
					m.setOstacolo( x, y );
				cout << "inserito ostacolo" << endl;
			} while( x > -1 && y > -1 );
		//cout << "fine ci sono blocchi" << endl;
	}

int main()
	{

		int X = 5, Y = 4;
		Mappa m( X, Y );

		m.setArrivo( 4, 3 );

		//disegnaMappa( X, Y, m );

		int x = 0, y = 0;
		char d;

		while( m.distanzaCella( x, y ) != 0 )
			{
				disegnaMappa( X, Y, m );
				nuoviBlocchi( m );
				//cout << "c1" << endl;
				d = nuovaDirezione( m.distanzaCella( x, y, 'a' ), m.distanzaCella( x, y, 'n' ), m.distanzaCella( x, y, 'e' ), m.distanzaCella( x, y, 's' ), m.distanzaCella( x, y, 'o' ) );
				cout << d << endl;
				x = d == 'e' ? x + 1 : x;
				x = d == 'o' ? x - 1 : x;
				y = d == 'n' ? y + 1 : y;
				y = d == 's' ? y - 1 : y;
				cout << x << ", " << y << endl << endl;
				if( d == 'f' )
					{
						cout << "Sono fuori dalla mappa, cos'è successo?!? :'(" << endl;
						return -1;
					}
			}

		char q = nuovaDirezione( m.distanzaCella( x, y, 'a' ), m.distanzaCella( x, y, 'n' ), m.distanzaCella( x, y, 'e' ), m.distanzaCella( x, y, 's' ), m.distanzaCella( x, y, 'o' ) );
		cout << q << endl << endl;

		return 0;
	}

