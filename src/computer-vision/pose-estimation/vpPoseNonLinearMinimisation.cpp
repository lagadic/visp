
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoseNonLinearMinimisation.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoseNonLinearMinimisation.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
 *
 * Description
 * ============
 *     Compute the pose using virtual visual servoing approach
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
\file vpPoseNonLinearMinimisation.cpp
\brief Compute the pose using virtual visual servoing approach
*/

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeaturePoint.h>


/*!
  \brief Compute the pose using virtual visual servoing approach

  This approach is described in

  E. Marchand, F. Chaumette. Virtual Visual Servoing: a framework for real-time
  augmented reality. In EUROGRAPHICS 2002 Conference Proceeding, G. Drettakis,
  H.-P. Seidel (eds.), Computer Graphics Forum, Volume 21(3), Pages 289-298,
  Sarrebruck, Allemagne, 2002.

 */

void
vpPose::poseVirtualVS(vpHomogeneousMatrix & cMo)
{
  try{
    // points du modele exprimes dans le repere de la camera
    vpPoint cP ;

    double  residu_1 = 1e8 ;
    double r =1e8-1;
    // on arete le processus iteratif quand l'erreur est constante
    // a 1e-8 pres
    while((int)((residu_1 - r)*1e33) !=0)
    {
      residu_1 = r ;
      vpMatrix L;  // matrice d'interaction
      vpColVector error ; // vecteur d'erreur

      // calcul de la matrice d'interaction et de l'erreur
      vpPoint P ;
      listP.front() ;
      while (!listP.outside())
      {

	vpFeaturePoint point ;
	P = listP.value() ;
	vpFeatureBuilder::create(point,P) ;
	// Calcul de la projection du modele 3D pour la pose cMo
	// changement de repere
	// projection perspective
	P.track(cMo) ;

	// calcul de la matrice d'interaction pour un point
	vpMatrix H ;
	H = point.interaction() ;

	// on empile les matrice d'interaction et l'erreur pour former
	// la matrice d'interaction globale et le vecteur d'erreur

	{
	  L =  vpMatrix::stackMatrices(L,H) ;
	  error = vpMatrix::stackMatrices(error, point.get_s() - P.p) ;
	}

	//   exit(1) ;
	listP.next() ;

      }

      // calcul du residu
      r = error.sumSquare() ;
      if (r>residu_1) break ;

      // calcul de la pseudo inverse de la matrice d'interaction
      vpMatrix Lp ;
      L.pseudoInverse(Lp,1e-16) ;

      // calcul de la loi de commande
      vpColVector v ;
      v = -lambda*Lp*error ;

      // mise a jour de la pose
      cMo = cMo.expMap(-v) ;

    }
  }catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
