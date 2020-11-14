/*
 * Spostamento.h
 *
 *  Created on: 07 apr 2016
 *      Author: Angelo
 */

#ifndef SPOSTAMENTO_H_
#define SPOSTAMENTO_H_

class Spostamento
	{
		double distanzaAttuale, distanzaTotale;
		double deltaV, deltaT;
		double vMax, vTeo;

		void aggiornaDistanza(double v);
		void calcolaVelocità();

	public:
		Spostamento(double x, double dt, double dv, double vM);
		virtual ~Spostamento();
		double getVelocità();
		//double getVelocità( int i ); //metodo arresto rapido

	};

#endif /* SPOSTAMENTO_H_ */
