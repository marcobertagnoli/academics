/*
 * Cella.h
 *
 *  Created on: 12 apr 2016
 *      Author: Angelo
 */

#ifndef CELLA_H_
#define CELLA_H_

class Cella
	{
		int distanza, contoRovescia;
		bool eBlocco;
		bool cellaEsplorata;
	public:
		Cella(int d);
		virtual ~Cella();
		void setDistanza(int d);
		void setBlocco(int c);
		void resetBlocco();
		int decrementaBlocco();
		int getDistanza() const;
		bool getBlocco() const;
		void setEsplorata();
		bool esplorata();
	};

#endif /* CELLA_H_ */
