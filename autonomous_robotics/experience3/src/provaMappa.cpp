
#include <esperienza1/Mappa.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <string>

using namespace cv;

Mappa::Mappa( int l, int a )
	{
		inizializzato = true;
		lunghezza = l;
		altezza = a;
		durataBlocco = 10;     //sarebbe da adoperare la "macro" in cima
		XStop = YStop = XUltimo = YUltimo = -1;     //inizializza l'arrivo in maniera fittizia fuori dalla mappa per evitare warning in compilazione
		Cella c( lunghezza * altezza );

		for( int i = 0; i < altezza; ++i )
			for( int j = 0; j < lunghezza; ++j )
				mappa.push_back( c );

		drawMap = false;
		primaImmagine = true;        
	}

Mappa::Mappa()
	{
		inizializzato = false;
	}

Mappa::~Mappa()
	{
	}

void Mappa::inizializzazione(int l, int a)
	{
		inizializzato = true;
		lunghezza = l;
		altezza = a;
		durataBlocco = 10;     //sarebbe da adoperare la "macro" in cima
		XStop = YStop = XUltimo = YUltimo = -1;     //inizializza l'arrivo in maniera fittizia fuori dalla mappa per evitare warning in compilazione
		Cella c( lunghezza * altezza );

		for( int i = 0; i < altezza; ++i )
			for( int j = 0; j < lunghezza; ++j )
				mappa.push_back( c );

        drawMap = false;
        primaImmagine = true;
	}

bool Mappa::eInizializzato() const
	{
		return inizializzato;
	}

Mat Mappa::immagineMappa(int x, int y, bool search)
{
    if (primaImmagine)
    {
	    drawMap = true;

	    //Carico le singole immagini per le celle
	    std::string path = ros::package::getPath("esperienza1");
		path = path + "/src/mapping/";

	    cella_vuota = imread(path + "cella_vuota.png");
	    cella_robot = imread(path + "cella_robot.png");
	    cella_ostacolo = imread(path + "cella_ostacolo.png");
	    cella_esplorata = imread(path + "cella_esplorata.png");
	    cella_ricerca = imread(path + "cella_ricerca.png");
	    cella_flag = imread(path + "cella_flag.png");
	    cella_rows = cella_vuota.rows;
	    cella_cols = cella_vuota.cols;

	    //Inizializzo la mappa completa
	    map = Mat(cella_rows * altezza, cella_cols * lunghezza, cella_vuota.type());

	    //Riempio la mappa
	    for(int row = 0; row < altezza; ++row)
	        for(int col = 0; col < lunghezza; ++col)
	        {
	            if(mappa[row * lunghezza + col].getBlocco())
	            {
	                cella_ostacolo.copyTo(map(Rect(col * cella_cols, row * cella_rows, cella_cols, cella_rows)));
	            }
	            else
	            {
	                cella_vuota.copyTo(map(Rect( col * cella_cols, row * cella_rows, cella_cols, cella_rows)));                
	            }
	        }

	    //Aggiunta flag e robot
	    cella_flag.copyTo(map(Rect( XStop * cella_cols, YStop * cella_rows, cella_cols, cella_rows)));
	    cella_robot.copyTo(map(Rect( x * cella_cols, y * cella_rows, cella_cols, cella_rows)));

	    old_pos_x = x;
	    old_pos_y = y;


	    primaImmagine = false;
	}
	else
	{
		if (search)
	    {
	    	if (sonoDentro( x + 1, y ))
	    	{
	    		cella_ricerca.copyTo(map(Rect( (x + 1) * cella_cols, y * cella_rows, cella_cols, cella_rows)));
	    	}
	    	if (sonoDentro( x - 1, y ))
	    	{
	    		cella_ricerca.copyTo(map(Rect( (x - 1) * cella_cols, y * cella_rows, cella_cols, cella_rows)));
	    	}
	    	if (sonoDentro( x, y + 1 ))
	    	{
	    		cella_ricerca.copyTo(map(Rect( x * cella_cols, (y + 1) * cella_rows, cella_cols, cella_rows)));
	    	}
	    	if (sonoDentro( x, y - 1 ))
	    	{
	    		cella_ricerca.copyTo(map(Rect( x * cella_cols, (y - 1) * cella_rows, cella_cols, cella_rows)));
	    	}
	    }
	    else
	    {
	    	cella_esplorata.copyTo(map(Rect(old_pos_x * cella_cols, old_pos_y * cella_rows, cella_cols, cella_rows)));
			mappa[old_pos_y * lunghezza + old_pos_x].setEsplorata();
			cella_robot.copyTo(map(Rect( x * cella_cols, y * cella_rows, cella_cols, cella_rows)));

			old_pos_x = x;
		    old_pos_y = y;
	    }
	}

    //Flip dell'immagine, necessario in quanto la mappa e l'immagine hanno sistemi di coordinate diversi
    Mat flipped;
    flip(map, flipped, 0);
    
    return flipped;
}

Mat Mappa::immagineMappa()
{
	Mat flipped;
    flip(map, flipped, 0);

	return flipped;
}

/*
bool Mappa::sonoDentro( int x, int y ) const
	{
		if( ( x >= 0 ) && ( x < lunghezza ) && ( y >= 0 ) && ( y < altezza ) )
			{
				//std::cout << "controllo indici positivo " << x << ";" << y << std::endl;
				return true;
			}

		else
			{
				//std::cout << "controllo indici negativo " << x << ";" << y << std::endl;
				return false;
			}

	}

bool Mappa::possoAndare( int x, int y ) const
	{
		if( eInizializzato() )
			if( sonoDentro( x, y ) )
				if( !mappa[y * lunghezza + x].getBlocco() )
					return true;

		return false;
	}

void Mappa::resettaDistanze()
	{
		for( int i = 0; i < altezza; ++i )     //possibilmente ottimizzare
			for( int j = 0; j < lunghezza; ++j )
				mappa[i * lunghezza + j].setDistanza( lunghezza * altezza );
	}

int Mappa::setArrivo( int x, int y )
	{
		if( eInizializzato() )
			{
				if( sonoDentro( x, y ) )
					{
						XStop = x;     //salvo i valori del'arrivo per eventuali ripopolamenti dovuti a nuovi blocchi o la loro scomparsa
						YStop = y;
						//std::cout << "calcolo mappa" << std::endl;
						mappa[y * lunghezza + x].setDistanza( 0 );

						coda.push( x );
						coda.push( y );

						popolaMappa();

						return 0;
					}
				else
					return -1;
			}
		return -10;
	}

void Mappa::popolaMappa()
	{
		int x, y;

		do
			{
				x = coda.front();     //prendo il primo elemento senza rimuoverlo
				coda.pop();     //rimuovoo il primo elemento
				y = coda.front();     //prendo il secondo elemento senza rimuoverlo
				coda.pop();     //rimuovoo il secondo elemento

				//std::cout << "popolo " << x << ";" << y << std::endl;     //controllo in debug

				int v = mappa[y * lunghezza + x].getDistanza() + 1;     //valore da inserire nella prossima cella se necessario perche' non ancora visitato o con percorso maggiore

				if( possoAndare( x + 1, y ) )
					{
						if( ( ( mappa[y * lunghezza + ( x + 1 )].getDistanza() ) > v ) )     //aggiungere controllo blocchi
							{
								mappa[y * lunghezza + ( x + 1 )].setDistanza( v );

								coda.push( x + 1 );
								coda.push( y );

							}
					}

				if( possoAndare( x - 1, y ) )
					{
						if( ( ( mappa[y * lunghezza + ( x - 1 )].getDistanza() ) > v ) )     //aggiungere controllo blocchi
							{
								mappa[y * lunghezza + ( x - 1 )].setDistanza( v );

								coda.push( x - 1 );
								coda.push( y );

							}
					}

				if( possoAndare( x, y + 1 ) )
					{
						if( ( ( mappa[( y + 1 ) * lunghezza + x].getDistanza() ) > v ) )     //aggiungere controllo blocchi
							{
								mappa[( y + 1 ) * lunghezza + x].setDistanza( v );

								coda.push( x );
								coda.push( y + 1 );

							}
					}

				if( possoAndare( x, y - 1 ) )
					{
						if( ( ( mappa[( y - 1 ) * lunghezza + x].getDistanza() ) > v ) )     //aggiungere controllo blocchi
							{
								mappa[( y - 1 ) * lunghezza + x].setDistanza( v );

								coda.push( x );
								coda.push( y - 1 );
							}
					}
			} while( !coda.empty() );
	}

void Mappa::setOstacolo( int x, int y )
	{
		if( eInizializzato() )
			if( sonoDentro( x, y ) )
				{
					mappa[y * lunghezza + x].setBlocco( durataBlocco );

					resettaDistanze();

					setArrivo( XStop, YStop );     //ricalcolo i nuovi percorsi
		
					if (drawMap) //Disegno l'ostacolo
					{
						cella_ostacolo.copyTo(map(Rect( x * cella_cols, y * cella_rows, cella_cols, cella_rows)));
					}
				}
		
	}

int Mappa::distanzaCella( int x, int y ) const
	{
		if( eInizializzato() )
			{
				if( sonoDentro( x, y ) )
					{
						if( mappa[y * lunghezza + x].getBlocco() )
							return -1;
						else
							return mappa[y * lunghezza + x].getDistanza();
					}

				return -2;     //sono fuori dalla mappa
			}
		return -10;
	}

int Mappa::distanzaCella( int x, int y, char d ) const
	{
		switch( d )
			{
			case 'n':
				return distanzaCella( x, y + 1 );     //ritorna distanza casella a nord

			case 's':
				return distanzaCella( x, y - 1 );     //ritorna distanza casella a

			case 'e':
				return distanzaCella( x + 1, y );     //ritorna distanza casella a est

			case 'o':
				return distanzaCella( x - 1, y );     //ritorna distanza casella a ovest

			case 'a':
				return distanzaCella( x, y );     //ritorna distanza casella stessa sulla quale e' stato invocato
			}
		return -5;     //errore grave: hai immesso un comando non valido
	}

void Mappa::abbassaPesoOstacoli()
	{
		if( eInizializzato() )
			{
				int c;
				bool bandiera = false;
				for( int i = 0; i < altezza; ++i )
					for( int j = 0; j < lunghezza; ++j )
						{
							c = mappa[i * lunghezza + j].decrementaBlocco();
							if( c == 1 )
								bandiera = true;     //un blocco e' scomparso
							if( c == -1 )
								std::cout << "errore grave" << std::endl;
						}

				if( bandiera )     //se almeno un blocco e' scomparso
					{
						resettaDistanze();
						setArrivo( XStop, YStop );     //ricalcolo la mappa
					}
			}
	}

void Mappa::rimuoviOstacolo( int x, int y )
	{
		if( eInizializzato() )
			if( sonoDentro( x, y ) )
				{
					mappa[y * lunghezza + x].resetBlocco();
					resettaDistanze();
					setArrivo( XStop, YStop );

					if (drawMap) //Disegno la cella vuota
					{
						if (mappa[y * lunghezza + x].esplorata())
						{
							cella_esplorata.copyTo(map(Rect( x * cella_cols, y * cella_rows, cella_cols, cella_rows)));
						}
						else
						{
							cella_vuota.copyTo(map(Rect( x * cella_cols, y * cella_rows, cella_cols, cella_rows)));
						}
					}

				}
	}
*/
/*char Mappa::nuovaDirezione( int x, int y )
 {

 if( sonoDentro( x, y ) )
 {

 if( distanzaCella( x, y ) == 0 )     //se sono all'arrivo
 {
 std::cout << "sono arrivato" << std::endl;
 return 'a';     //ritorno 'a' di arrivo
 }

 int n, s, e, o;

 char u = 'n';     //caso generale vado a nord
 XUltimo = x;
 YUltimo = y + 1;

 //metto nelle variabili di direzione il valore che hanno in distanza
 // ------> da implementare controllo blocco  <-------
 e = ( possoAndare( x + 1, y ) ) ? distanzaCella( x + 1, y ) : lunghezza * altezza;
 o = ( possoAndare( x - 1, y ) ) ? distanzaCella( x - 1, y ) : lunghezza * altezza;
 n = ( possoAndare( x, y + 1 ) ) ? distanzaCella( x, y + 1 ) : lunghezza * altezza;
 s = ( possoAndare( x, y - 1 ) ) ? distanzaCella( x, y - 1 ) : lunghezza * altezza;

 if( ( e <= n ) && ( e <= s ) && ( e <= o ) )
 {
 u = 'e';
 XUltimo = x + 1;
 YUltimo = y;
 }

 if( ( s <= n ) && ( s <= e ) && ( s <= o ) )
 {
 u = 's';
 XUltimo = x;
 YUltimo = y - 1;
 }

 if( ( o <= n ) && ( o <= s ) && ( o <= e ) )
 {
 u = 'o';
 XUltimo = x - 1;
 YUltimo = y;
 }

 abbassaPesoOstacoli();

 return u;
 }
 else
 return 'f';     // non posso muovermi se sono fuori dalla mappa!!
 }*/
