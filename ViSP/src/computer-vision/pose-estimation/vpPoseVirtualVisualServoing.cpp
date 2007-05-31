/****************************************************************************
 *
 * $Id: vpPoseVirtualVisualServoing.cpp,v 1.9 2007-05-31 13:08:20 asaunier Exp $
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
 * This file is part of the ViSP toolkit
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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpPoseVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach
*/

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpRobust.h>

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


      //std::cout << "r=" << r <<std::endl ;
      // update the pose

      cMo = vpExponentialMap::direct(v).inverse()*cMo ;
      if (iter++>VVS_NB_ITER_MAX) break ;
    }
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

}

/*!
  \brief Compute the pose using virtual visual servoing approach and
  a robust cotrol law

  This approach is described in

  A.I. Comport, E. Marchand, M. Pressigout, F. Chaumette. Real-time
  markerless tracking for augmented reality: the virtual visual servoing
  framework. IEEE Trans. on Visualization and Computer Graphics,
  12(4):615-628, Juillet 2006.

*/

void
vpPose::poseVirtualVSrobust(vpHomogeneousMatrix & cMo)
{
  try{

    double  residu_1 = 1e8 ;
    double r =1e8-1;

    // we stop the minimization when the error is bellow 1e-8
    vpMatrix W ;
    vpRobust robust(2*listP.nbElement()) ;
    vpColVector w,res ;
    
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

      robust.setIteration(0);
      w.resize(error.getRows()/2) ;
      res.resize(error.getRows()/2) ;

      for(int k=0 ; k <error.getRows()/2 ; k++)
      {
	res[k] = vpMath::sqr(error[2*k]) + vpMath::sqr(error[2*k+1]) ;
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);
      // vpTRACE(" ") ;

      W.resize(error.getRows(), error.getRows()) ;
      // compute the pseudo inverse of the interaction matrix
      for (int k=0 ; k < error.getRows()/2 ; k++)
      {
	W[2*k][2*k] = w[k] ;
	W[2*k+1][2*k+1] = w[k] ;
      }
       vpTRACE(" ") ;
     // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp ;
      (W*L).pseudoInverse(Lp,1e-16) ;
      vpTRACE(" ") ;

      // compute the VVS control law
      vpColVector v ;
      v = -lambda*Lp*W*error ;


      //cout << "r=" << r <<endl ;
      // update the pose

      cMo = vpExponentialMap::direct(v).inverse()*cMo ; ;
      if (iter++>VVS_NB_ITER_MAX) break ;
    }
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

}

