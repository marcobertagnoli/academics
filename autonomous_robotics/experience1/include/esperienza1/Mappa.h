#ifndef MAPPA_H_
#define MAPPA_H_

#include <esperienza1/Cella.h>
#include <vector>
#include <queue>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>

using namespace cv;

class Mappa
	{
        Mat cella_vuota; //Immagini delle singole celle
        Mat cella_esplorata;
        Mat cella_robot;
        Mat cella_ricerca;
        Mat cella_ostacolo;
        Mat cella_flag;
        int cella_rows, cella_cols, old_pos_x, old_pos_y; //Dimensioni della singola cella
        bool primaImmagine;

        Mat map; //Immagine della mappa completa

        bool inizializzato;
		bool drawMap;
		int lunghezza, altezza;     //dimensioni mappa
		int XStop, YStop;     //coordinate arrivo per ricalcolo mappa
		int XUltimo, YUltimo;     //ultima posizione nota
		int durataBlocco;     //durata dei blocchi
		std::vector< Cella > mappa;     //matrice principale di lavoro
		std::queue< int > coda;     // coda ottimizzazione popolamento
		bool eInizializzato() const;
		void popolaMappa();     //riempie la mappa con i valori corretti di distanza
		bool sonoDentro( int x, int y ) const;     //indica se la cella xy e' allinterno della mappa
		void resettaDistanze();     //rimette le distanze al massimo

	public:

		Mappa( int l, int a );
		Mappa();
		virtual ~Mappa();
        Mat immagineMappa(int x, int y, bool); /* Restituisce un immagine contenente la mappa, 
        											riceve in ingresso la posizione attuale del robot il suo stato ed il percorso seguito dal robot*/
        Mat immagineMappa();
		void inizializzazione(int l, int a);
		int setArrivo( int x, int y );     //setta cella d'arrivo e popola la mappa
		void setOstacolo( int x, int y );     //setta cella ad ostacolo
		void rimuoviOstacolo( int x, int y );     //riumovi un blocco ostacolo
//		char nuovaDirezione( int x, int y );     //ritornal la miglior direzione per arrivare all'arrivo
		int distanzaCella( int x, int y ) const;     //da mettere privato alla fine del debugging
		int distanzaCella( int x, int y , char d) const;
		bool possoAndare( int x, int y ) const;     //indica se la cella xy e' allinterno della mappa e non e' un blocco
		void abbassaPesoOstacoli();     //permette di aggiornare i blocchi

	};

#endif /* MAPPA_H_ */
