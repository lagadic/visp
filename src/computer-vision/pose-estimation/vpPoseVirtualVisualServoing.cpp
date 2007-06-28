/****************************************************************************
 *
 * $Id: vpPoseVirtualVisualServoing.cpp,v 1.10 2007-06-28 11:52:04 marchand Exp $
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
   try
  {

    double  residu_1 = 1e8 ;
    double r =1e8-1;

    // we stop the minimization when the error is bellow 1e-8

    int iter = 0 ;

    int nb = listP.nbElement() ;
    vpMatrix L(2*nb,6) ;
    vpColVector err(2*nb) ;
    vpColVector sd(2*nb),s(2*nb) ;

    listP.front() ;
    vpPoint P;
    vpList<vpPoint> lP ;

    // create sd
    int k =0 ;
    while (!listP.outside())
    {
      	P = listP.value() ;
	sd[2*k] = P.get_x() ;
	sd[2*k+1] = P.get_y() ;
	lP += P ;
     	listP.next() ;
	k+=1 ;
     }
    while((int)((residu_1 - r)*1e12) !=0)
    {

      residu_1 = r ;

      // Compute the interaction matrix and the error
      int k =0 ;
      lP.front() ;
      while (!lP.outside())
      {
	double xi,yi,xp,yp;


	P = lP.value() ;
	// forward projection of the 3D model for a given pose
	// change frame coordinates
	// perspective projection
	P.track(cMo) ;

	double x = s[2*k] = P.get_x();  /* point projected from cMo */
	double y = s[2*k+1] = P.get_y();
	double Z = P.get_Z() ;
	L[2*k][0] = -1/Z  ;
	L[2*k][1] = 0 ;
	L[2*k][2] = x/Z ;
	L[2*k][3] = x*y ;
	L[2*k][4] = -(1+x*x) ;
	L[2*k][5] = y ;

	L[2*k+1][0] = 0 ;
	L[2*k+1][1]  = -1/Z ;
	L[2*k+1][2] = y/Z ;
	L[2*k+1][3] = 1+y*y ;
	L[2*k+1][4] = -x*y ;
	L[2*k+1][5] = -x ;

	lP.next() ;

	k+=1 ;
     }
      err = s - sd ;

  // compute the residual
      r = err.sumSquare() ;

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp ;
      L.pseudoInverse(Lp,1e-16) ;

      // compute the VVS control law
      vpColVector v ;
      v = -lambda*Lp*err ;


      //std::cout << "r=" << r <<std::endl ;
      // update the pose

      cMo = vpExponentialMap::direct(v).inverse()*cMo ; ;
      if (iter++>vvsIterMax) break ;

    }
    lP.kill() ;
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
    robust.setThreshold(0.0000) ;
    vpColVector w,res ;

    int nb = listP.nbElement() ;
    vpMatrix L(2*nb,6) ;
    vpColVector error(2*nb) ;
    vpColVector sd(2*nb),s(2*nb) ;

    listP.front() ;
    vpPoint P;
    vpList<vpPoint> lP ;

    // create sd
    int k =0 ;
    while (!listP.outside())
    {
      	P = listP.value() ;
	sd[2*k] = P.get_x() ;
	sd[2*k+1] = P.get_y() ;
	lP += P ;
     	listP.next() ;
	k+=1 ;
     }
    int iter = 0 ;
      res.resize(s.getRows()/2) ;
      w.resize(s.getRows()/2) ;
      W.resize(s.getRows(), s.getRows()) ;
      w =1 ;
    while((int)((residu_1 - r)*1e12) !=0)
    {

      residu_1 = r ;
      vpColVector error ; // error vector

      // Compute the interaction matrix and the error
      vpPoint P ;
      lP.front() ;
      k =0 ;
      while (!lP.outside())
      {


	P = lP.value() ;
	// forward projection of the 3D model for a given pose
	// change frame coordinates
	// perspective projection
	P.track(cMo) ;

	double x = s[2*k] = P.get_x();  // point projected from cMo
	double y = s[2*k+1] = P.get_y();
	double Z = P.get_Z() ;
	L[2*k][0] = -1/Z  ;
	L[2*k][1] = 0 ;
	L[2*k][2] = x/Z ;
	L[2*k][3] = x*y ;
	L[2*k][4] = -(1+x*x) ;
	L[2*k][5] = y ;

	L[2*k+1][0] = 0 ;
	L[2*k+1][1]  = -1/Z ;
	L[2*k+1][2] = y/Z ;
	L[2*k+1][3] = 1+y*y ;
	L[2*k+1][4] = -x*y ;
	L[2*k+1][5] = -x ;

	lP.next() ;


	k+=1 ;

      }
      error = s - sd ;

      // compute the residual
      r = error.sumSquare() ;


      for(int k=0 ; k <error.getRows()/2 ; k++)
      {
	res[k] = vpMath::sqr(error[2*k]) + vpMath::sqr(error[2*k+1]) ;
      }
      robust.setIteration(0);
      robust.MEstimator(vpRobust::TUKEY, res, w);


      // compute the pseudo inverse of the interaction matrix
      for (int k=0 ; k < error.getRows()/2 ; k++)
      {
	W[2*k][2*k] = w[k] ;
	W[2*k+1][2*k+1] = w[k] ;
      }
     // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp ;
      (W*L).pseudoInverse(Lp,1e-6) ;


      // compute the VVS control law
      vpColVector v ;
      v = -lambda*Lp*W*error ;

      cMo = vpExponentialMap::direct(v).inverse()*cMo ; ;
      if (iter++>vvsIterMax) break ;
    }
    lP.kill() ;

  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

}


