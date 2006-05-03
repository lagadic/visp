
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoseVirtualVisualServoing.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoseVirtualVisualServoing.cpp,v 1.4 2006-05-03 07:26:31 fspindle Exp $
 *
 * Description
 * ============
 *     Compute the pose using virtual visual servoing approach
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpPoseVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach
*/

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpExponentialMap.h>

// define the maximum number of iteration
#ifdef VVS_NB_ITER_MAX
#undef VVS_NB_ITER_MAX
#endif
#define VVS_NB_ITER_MAX 1000
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

    // we stop the minimization when the error is bellow 1e-8

    int iter = 0 ;
    while((int)((residu_1 - r)*1e12) !=0)
    {

      residu_1 = r ;
      vpMatrix L;  // interaction matrix
      vpColVector error ; // error vector

      // Compute the interaction matrix and the error
      vpPoint P ;
      listP.front() ;
      while (!listP.outside())
      {

	double xi,yi,xp,yp;
	vpColVector err(2);

       	vpFeaturePoint point ;
	P = listP.value() ;
	vpFeatureBuilder::create(point,P) ;

	//	P.print() ;
	xi = point.get_x();  /* point measured in the image */
	yi = point.get_y();


	// forward projection of the 3D model for a given pose
	// change frame coordinates
	// perspective projection
	P.track(cMo) ;

	xp = P.get_x();  /* point projected from cMo */
	yp = P.get_y();


	err[0] = xp-xi;
	err[1] = yp-yi;

	point.set_xyZ(xp,yp,P.get_Z());
	// compute the interaction matrix
	vpMatrix H ;
	H = point.interaction() ;

	// stack the interaction matrix and the errir vector

	L =  vpMatrix::stackMatrices(L,H) ;
	error = vpMatrix::stackMatrices(error, err) ;

	listP.next() ;

      }

      // compute the residual
      r = error.sumSquare() ;

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp ;
      L.pseudoInverse(Lp,1e-16) ;

      // compute the VVS control law
      vpColVector v ;
      v = -lambda*Lp*error ;


      //cout << "r=" << r <<endl ;
      // update the pose

      cMo = vpExponentialMap::direct(v).inverse()*cMo ; ;
      if (iter++>VVS_NB_ITER_MAX) break ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}

