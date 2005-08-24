
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
 *  $Id: vpPoseNonLinearMinimisation.cpp,v 1.2 2005-08-24 15:53:02 chaumett Exp $
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

    double  residu_1 = 1e8 ;
    double r =1e8-1;

     // on arete le processus iteratif quand l'erreur est constante
    // a 1e-8 pres
    while((int)((residu_1 - r)*1e12) !=0)
    {
      //       cout << "----------------------------" << endl;
      //       cout << cMo << endl ;

       residu_1 = r ;
      vpMatrix L;  // matrice d'interaction
      vpColVector error ; // vecteur d'erreur

      // calcul de la matrice d'interaction et de l'erreur
      vpPoint P ;
      listP.front() ;
      while (!listP.outside())
      {

	double xi,yi,xp,yp;
	vpColVector err(2);

       	vpFeaturePoint point ;
	P = listP.value() ;
	vpFeatureBuilder::create(point,P) ;

	xi = point.get_x();  /* point measured in the image */
	yi = point.get_y();

	//	cout << "measured point : " << xi << " " << yi << endl;

	// Calcul de la projection du modele 3D pour la pose cMo
	// changement de repere
	// projection perspective
	P.track(cMo) ;

	xp = P.get_x();  /* point projected from cMo */
	yp = P.get_y();

	//	cout << "projected point : " << xp << " " << yp << endl;

	err[0] = xp-xi;
	err[1] = yp-yi;

	point.set_xyZ(xp,yp,P.get_Z());
	// calcul de la matrice d'interaction pour un point
	vpMatrix H ;
	H = point.interaction() ;

	// on empile les matrice d'interaction et l'erreur pour former
	// la matrice d'interaction globale et le vecteur d'erreur

	  L =  vpMatrix::stackMatrices(L,H) ;
	  //        error = vpMatrix::stackMatrices(error, -(point.get_s() - P.p)) ;
      	  error = vpMatrix::stackMatrices(error, err) ;

	listP.next() ;

      }
      // 	cout << L << endl;
      // 	cout << "error : " << error.t() <<endl;

      // calcul du residu
      r = error.sumSquare() ;

      // cout << "residu : " << r << endl;

      // FC      if (r>residu_1) break ;

      // calcul de la pseudo inverse de la matrice d'interaction
      vpMatrix Lp ;
      L.pseudoInverse(Lp,1e-16) ;

      // calcul de la loi de commande
      vpColVector v ;
      v = -lambda*Lp*error ;

      //  cout << "v : " << v.t() << endl;

      // mise a jour de la pose
      cMo = cMo.expMap(v) ;

      //  cout << cMo << endl ;
      //      int in;
      //      scanf("%d", &in);
    }
  }catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
