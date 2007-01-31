/****************************************************************************
 *
 * $Id: vpMe.cpp,v 1.4 2007-01-31 15:25:59 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 *
 *****************************************************************************/

/*!
	\file vpMe.cpp
	\brief Moving edges
*/

#include <visp/vpMe.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS


struct point
{
  double x ;
  double y ;
} ;

struct droite
{
  double a ;
  double b ;
  double c ;
} ;







template <class Type>
inline void
permute(Type & a, Type & b)
{
  Type t = a;
  a = b;
  b = t;
}

static droite
droite_cartesienne(point P, point Q)
{
  droite PQ;

  PQ.a = P.y-Q.y;
  PQ.b = Q.x-P.x;
  PQ.c = Q.y*P.x - Q.x*P.y;

  return(PQ);
}



static point
point_intersection(droite D1, droite D2)
{
 point  I;
 double det;  // déterminant des 2 vect.normaux

 det = (D1.a*D2.b - D2.a*D1.b); // interdit D1,D2 parallèles
 I.x = (D2.c*D1.b - D1.c*D2.b)/det;
 I.y = (D1.c*D2.a - D2.c*D1.a)/det;

 return(I);
}

static void
recale(point & P,
			 double Xmin, double Ymin, double Xmax, double Ymax)
{
  if(vpMath::equal(P.x,Xmin))
		P.x=Xmin; // à peu près => exactement !
  if(vpMath::equal(P.x,Xmax))
		P.x=Xmax;

  if(vpMath::equal(P.y,Ymin))
		P.y=Ymin;
  if(vpMath::equal(P.y,Ymax))
		P.y=Ymax;
}


static void
permute(point &A,  point &B)
{
  point C ;

  if (A.x>B.x) // fonction sans doute a tester...
  {
    C = A ;
    A = B ;
    B = C ;
  }
}


// vrai si partie visible
static bool
clipping  (point  A, point B,
 					 double Xmin, double Ymin, double Xmax, double Ymax,
					 point  & Ac , point & Bc )// résultat: A,B clippés
{
 droite AB, D[4];
 D[0].a = 1;     D[0].b = 0;     D[0].c = -Xmin;
 D[1].a = 1;     D[1].b = 0;     D[1].c = -Xmax;
 D[2].a = 0;     D[2].b = 1;     D[2].c = -Ymin;
 D[3].a = 0;     D[3].b = 1;     D[3].c = -Ymax;

 point P[2];
 P[0]=A; P[1]=B;
 int code_P[2], // codes de P[n]
 i, bit_i, 		  // i -> (0000100...)
 n;

 AB = droite_cartesienne(A,B);

 while(1)                               // 2 sorties directes internes
 {
   // CALCULE CODE DE VISIBILITE (Sutherland & Sproul)
   // ================================================
   for(n=0; n<2; n++)
   {
     code_P[n] = 0000;

     if( P[n].x < Xmin )
			 code_P[n] |= 1;         // positionne bit0
     if( P[n].x > Xmax )
			 code_P[n] |= 2;         //    ..      bit1
     if( P[n].y < Ymin )
			 code_P[n] |= 4;         //    ..      bit2
     if( P[n].y > Ymax )
			 code_P[n] |= 8;         //    ..      bit3
   }


   // 2 CAS OU L'ON PEUT CONCLURE => sortie
   // =====================================
   if((code_P[0] | code_P[1])==0000)  // Aucun bit à 1
		/* NE TRIE PLUS LE RESULTAT ! S_relative() en tient compte
        { if(P[0].x < P[1].x) // Rend le couple de points
                { Ac=P[0];  Bc=P[1]; }  //  clippés (ordonnés selon
          else  { Ac=P[1];  Bc=P[0]; }  //  leur abscisse x)
		*/
    {
			Ac=P[0];  Bc=P[1];
      if(vpMath::equal(Ac.x,Bc.x) && vpMath::equal(Ac.y,Bc.y))
      	return(false);    // AB = 1 point = invisible
      else
				return(true);    // Partie de AB clippée visible!
    }

   if((code_P[0] & code_P[1])!=0000)  // au moins 1 bit commun
   {
		 return(false);  // AB complètement invisible!
   }


   // CAS GENERAL (on sait que code_P[0 ou 1] a au moins un bit à 1
   //   - clippe le point P[n] qui sort de la fenêtre (coupe Droite i)
   //   - reboucle avec le nouveau couple de points
   // ================================================================
   if(code_P[0] != 0000)
   {
	 		n=0;   // c'est P[0] qu'on clippera
      for(i=0,bit_i=1;  !(code_P[0] & bit_i);  i++,bit_i<<=1);
   }
   else
	 {
			n=1;   // c'est P[1] qu'on clippera
      for(i=0,bit_i=1;  !(code_P[1] & bit_i);  i++,bit_i<<=1);
   }

   P[n] = point_intersection(AB,D[i]); // clippe le point concerné


   // RECALE EXACTEMENT LE POINT (calcul flottant => arrondi)
   // AFIN QUE LE CALCUL DES CODES NE BOUCLE PAS INDEFINIMENT
   // =======================================================
   recale(P[n], Xmin,Ymin,Xmax,Ymax);

 }

}


// calcule la surface relative des 2 portions définies
// par le segment PQ sur le carré Xmin,Ymin,Xmax,Ymax
// Rem : P,Q triés sur x, et donc seulement 6 cas
static double
S_relative(point P, point Q,
         	 double Xmin, double Ymin, double Xmax, double Ymax)
{

  if(Q.x < P.x)         // tri le couple de points
    permute(P,Q);  //  selon leur abscisse x



  recale(P, Xmin,Ymin,Xmax,Ymax);  // permet des calculs de S_relative
  recale(Q, Xmin,Ymin,Xmax,Ymax);  //  moins approximatifs.



  if(P.x==Xmin && Q.x==Xmax)
	  return( fabs(Ymax+Ymin-P.y-Q.y) );

  if( (P.y==Ymin && Q.y==Ymax) ||
      (Q.y==Ymin && P.y==Ymax))
	  return( fabs(Xmax+Xmin-P.x-Q.x) );

  if( P.x==Xmin && Q.y==Ymax )
		 return( 1-(Ymax-P.y)*(Q.x-Xmin) );
  if( P.x==Xmin && Q.y==Ymin )
		 return( 1-(P.y-Ymin)*(Q.x-Xmin) );
  if( P.y==Ymin && Q.x==Xmax )
		 return( 1-(Xmax-P.x)*(Q.y-Ymin) );
  if( P.y==Ymax && Q.x==Xmax )
		 return( 1-(Xmax-P.x)*(Ymax-Q.y) );


 	printf("utils_ecm: ERREUR dans S_relative (%f,%f) (%f,%f) %f %f %f %f\n",
        P.x,P.y,Q.x,Q.y,Xmin,Ymin,Xmax,Ymax);
  exit(-1);
  return(0);  // DEBUG Stoppe net l'execution

}


static void
calcul_masques(vpColVector &angle, // définitions des angles theta
	       int n,             // taille masques (PAIRE ou IMPAIRE Ok)
	       vpMatrix *M)        // résultat M[theta](n,n)
{
  // Le coef |a| = |1/2n| n'est pas incorporé dans M(i,j) (=> que des int)

  int i_theta,  // indice (boucle sur les masques)
      i,j;      // indices de boucle sur M(i,j)
  double X,Y,   // point correspondant/centre du masque
    theta, cos_theta, sin_theta, tan_theta,
    moitie = ((double)n)/2.0; // moitie REELLE du masque
  point P1,Q1,P,Q;  // clippe Droite(theta) P1,Q1 -> P,Q
  int    sgn;       // signe de M(i,j)
  double v;         // ponderation de M(i,j)

 int nb_theta = angle.getRows() ;

 for(i_theta=0; i_theta<nb_theta; i_theta++)
 {
   theta = M_PI/180*angle[i_theta]; // indice i -> theta(i) en radians
   																//  angle[] dans [0,180[
   cos_theta = cos(theta);        // vecteur directeur de l'ECM
   sin_theta = sin(theta);        //  associe au masque

   // PRE-CALCULE 2 POINTS DE D(theta) BIEN EN DEHORS DU MASQUE
   // =========================================================
   if( angle[i_theta]==90 )                     // => tan(theta) infinie !
   {
     P1.x=0; P1.y=-n;
     Q1.x=0; Q1.y=n;
   }
   else
   {
     tan_theta = sin_theta/cos_theta;       // pente de la droite D(theta)
     P1.x=-n; P1.y=tan_theta*(-n);
     Q1.x=n;  Q1.y=tan_theta*n;
   }

   // CALCULE MASQUE M(theta)
   // ======================
   M[i_theta].resize(n,n);  // allocation (si necessaire)

   for(i=0,Y=-moitie+0.5 ;   i<n  ; i++,Y++)
   {
     for(j=0,X=-moitie+0.5 ;   j<n  ; j++,X++)
     {
       // produit vectoriel dir_droite*(X,Y)
       sgn = vpMath::sign(cos_theta*Y - sin_theta*X);

       // Résultat = P,Q
       if( clipping(P1,Q1, X-0.5,Y-0.5,X+0.5,Y+0.5, P,Q) )
       {
	 // v dans [0,1]
	 v=S_relative(P,Q, X-0.5,Y-0.5,X+0.5,Y+0.5);
       }
       else
	 v=1; // PQ ne coupe pas le pixel(i,j)

       M[i_theta][i][j] = vpMath::round(100*sgn*v);

       // 2 chiffres significatifs
       // M(i,j) sans incorporer le coef a
     }
   }
 }

}

#endif

void
vpMe::initMask()
{

  int i ;

  mask = new vpMatrix[n_mask] ;

  vpColVector angle(n_mask) ;

  int angle_pas ;
  angle_pas = 180 / n_mask ;

  int k =0 ;
  for (i = 0 ; i < 180 ; i += angle_pas)
    angle[k++] = i ;

  calcul_masques(angle, mask_size, mask ) ;

}



void
vpMe::print( )
{

  cout<< endl ;
  cout<<"Moving edges settings " <<endl  ;
  cout<< endl ;
  cout<<" Size of the convolution masks...."<<mask_size<<"x"<<mask_size<<" pixels"<<endl ;
  cout<<" Number of masks.................."<<n_mask<<"        "<<endl ;
  cout<<" Query range +/- J................"<<range<<" pixels  "<<endl ;
  cout<<" Likelyhood test ratio............"<<threshold<<endl ;
  cout<<" Contrast tolerance +/-..........."<< mu1 * 100<<"% and "<<mu2 * 100<<"%     "<<endl ;
  cout<<" Sample step......................"<<sample_step<<" pixels"<<endl ;
  cout<<" Strip............................"<<strip<<" pixels  "<<endl ;
  cout<<" Min_Samplestep..................."<<min_samplestep<<" pixels  "<<endl ;
  cout<<" Aberration......................."<<aberration<< endl ;
  cout<<" init_aberration.................."<<init_aberration<<endl ;

}

vpMe::vpMe()
{


  mask = NULL ;
  threshold = 1500 ;
  mu1 = 0.5 ;
  mu2 = 0.5 ;
  sample_step = 10 ;
  range = 4 ;
  mask_size = 5 ;
  n_mask = 180 ;
  mask_sign = 0 ;
  anglestep = (180 / n_mask) ;
  strip = 2 ;
  min_samplestep = 4 ;
  aberration = 2.0 ;
  init_aberration = 5.0 ;
  initMask() ;
}

vpMe::~vpMe()
{
  if (mask != NULL)
  {
    if (mask != NULL)
      delete []mask ;
  }
}




void
vpMe::setNumberMask(int a)
{
  if (mask != NULL)
  {
    for (int i=0 ; i < n_mask ; i++)
      mask[i].~vpMatrix()  ;
    if (mask != NULL) delete []mask ;
  }
  n_mask = a  ;
  anglestep = 180 / a ;
  initMask() ;
}

void
vpMe::setSizeMask(int a)
{
  if (mask != NULL)
  {
    for (int i=0 ; i < n_mask ; i++)
	    mask[i].~vpMatrix()  ;
    if (mask != NULL) delete []mask ;
  }
  mask_size =a  ;
  initMask() ;
}




