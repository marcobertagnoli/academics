/*
 * Spostamento.cpp
 *
 *  Created on: 07 apr 2016
 *      Author: Angelo
 */

#include "Spostamento.h"

Spostamento::Spostamento( double x, double dt, double dv, double vM )
	{
		distanzaTotale = x;
		deltaV = dv;
		deltaT = dt;
		vMax = vM;
		distanzaAttuale = 0;
		vTeo = 0;
	};

Spostamento::~Spostamento()
	{
		// TODO Auto-generated destructor stub
	};

double Spostamento::getVelocit�()
	{
		if( vTeo < vMax )
			{
				aggiornaDistanza (vTeo);
				return vTeo;
			}
		else
			{
				aggiornaDistanza (vMax);
				return vMax;
			};
	};
//double getVelocit�( int i ); //metodo arresto rapido
void Spostamento::aggiornaDistanza( double v )
	{
		distanzaAttuale += v * deltaT;
	};

void Spostamento::calcolaVelocit�()
	{
		if(vTeo == 0)
			vTeo = deltaV/2;
		else
			if(distanzaAttuale < ( distanzaTotale/2 ))
				vTeo += deltaV;
			if( distanzaAttuale > ( distanzaTotale/2 ))
				{
					vTeo -= deltaV;
					if( vTeo < 0 )
						vTeo = 0;
				};
	};

