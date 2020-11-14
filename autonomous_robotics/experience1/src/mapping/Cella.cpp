/*
 * Cella.cpp
 *
 *  Created on: 12 apr 2016
 *      Author: Angelo
 */

#include <esperienza1/Cella.h>

Cella::Cella( int c )
	{
		distanza = c;
		eBlocco = false;
		contoRovescia = 0;
		cellaEsplorata = false;
	}
;

Cella::~Cella()
	{
	}

void Cella::setDistanza( int d )
	{
		distanza = d;
	}
;

void Cella::setEsplorata()
{
	cellaEsplorata = true;
}
;

bool Cella::esplorata()
{
	return cellaEsplorata;
}

void Cella::setBlocco( int c )
	{
		if( c > 0 )     //se il contatore e' maggiore di 0 la cella e' un blocco
			{
				contoRovescia = c;
				eBlocco = true;
			}
	}
;

void Cella::resetBlocco()
	{
		eBlocco = false;
	}
;

int Cella::decrementaBlocco()
	{
		if( eBlocco )
			{
				contoRovescia--;
				if( contoRovescia == 0 )
					{
						resetBlocco();
						return 1;     //il blocco e' scomparso
					}
				return 0;
			};
		if( contoRovescia < 0 )
			return -1;     //ERRORE GRAVE NON PREVISTO, DA RICONTROLLARE IL CODICE :(

		return 2;     //invocato il decrementa blocco su una cella libera
	}

int Cella::getDistanza() const
	{
		return distanza;
	}
;

bool Cella::getBlocco() const
	{
		return eBlocco;
	}
;

