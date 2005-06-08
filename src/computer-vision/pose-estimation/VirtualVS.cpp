
/*                                                                -*-c++-*-
    Copyright (C) 1998 Eric Marchand IRISA-INRIA Rennes Vista Project

    Contact:
       Eric Marchand
       IRISA-INRIA Rennes
       35042 Rennes Cedex
       France

    email: marchand@irisa.fr
    www  : http://www.irisa.fr/vista

    Sundar.cc : premiere version Eric Marchand le  21 avril 1999

*/


#include <CPose.h>
#include <basicfeatures/CPoint.h>
#include <derived-features/simulation/CPointSimu.h>
#include <robot/simulation/UtilsSimu.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

int
CPose::PoseVirtualVS(CMatrixHomogeneous &cMo)
{
  if (DEBUG_LEVEL1)
    cout << "begin CPose::PoseVVS()" << endl ;

  // points du modele exprimes dans le repere de la camera
  CPoint cP ; 
    
  double  residu_1 = 1e8 ;
  double r =1e8-1;
  // on arete le processus iteratuf quand l'erreur est constante
  // a 1e-8 pres
  while((int)((residu_1 - r)*1e8) !=0)
  {
    residu_1 = r ;
    CMatrix L;  // matrice d'interaction
    CColVector error ; // vecteur d'erreur
      
    // calcul de la matrice d'interaction et de l'erreur
    for (int i=0 ; i < npt ; i++)
    {
      // Calcul de la projection du modele 3D pour la pose cMo
      
      // changement de repere
      cP = cMo*P[i] ;
      // projection perspective
      cP.Projection() ;
      // calcul de la matrice d'interaction pour un point
      CMatrix H ;
      cP.Interaction(H) ;
      // on empile les matrice d'interaction et l'erreur pour former
      // la matrice d'interaction globale et le vecteur d'erreur
      if (i==0)
      {
	error = cP.state-P[i].state ;
	L = H ;
      }
      else
      {
	L =  StackMatrices(L,H) ;
	error = StackMatrices(error, cP.state-P[i].state) ;
      }
    }
    
    
    // calcul du residu
    r = error.SumSquare() ;
    if (r>residu_1) break ;
    
    // calcul de la pseudo inverse de la matrice d'interaction
    CMatrix Lp ;
    L.PseudoInverse(Lp,1e-16) ;
    
    // calcul de la loi de commande
    CColVector Tc ;
    Tc = -lambda*Lp*error ;
    
    // mise a jour de la pose
    UpdatePose(Tc,cMo) ;
    
  }


  if (DEBUG_LEVEL1)
    cout << "end CPose::PoseVVS()" << endl ;
  return OK;
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
