/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

    unsigned int nb = listP.size() ;
    vpMatrix L(2*nb,6) ;
    vpColVector err(2*nb) ;
    vpColVector sd(2*nb),s(2*nb) ;
    vpColVector v ;
    
    vpPoint P;
    std::list<vpPoint> lP ;

    // create sd
    unsigned int k =0 ;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
    {
      P = *it;
      sd[2*k] = P.get_x() ;
      sd[2*k+1] = P.get_y() ;
      lP.push_back(P);
      k ++;
    }

    while((int)((residu_1 - r)*1e12) !=0)
    {      
      residu_1 = r ;

      // Compute the interaction matrix and the error
      unsigned int k =0 ;
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it)
      {
        P = *it;
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

        k+=1 ;
      }
      err = s - sd ;

      // compute the residual
      r = err.sumSquare() ;

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp ;
      L.pseudoInverse(Lp,1e-16) ;

      // compute the VVS control law
      v = -lambda*Lp*err ;

      //std::cout << "r=" << r <<std::endl ;
      // update the pose

      cMo = vpExponentialMap::direct(v).inverse()*cMo ; ;
      if (iter++>vvsIterMax) break ;
    }
    
    if(computeCovariance)
      covarianceMatrix = vpMatrix::computeCovarianceMatrix(L,v,-lambda*err);
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
    vpRobust robust(2*listP.size()) ;
    robust.setThreshold(0.0000) ;
    vpColVector w,res ;

    unsigned int nb = listP.size() ;
    vpMatrix L(2*nb,6) ;
    vpColVector error(2*nb) ;
    vpColVector sd(2*nb),s(2*nb) ;
    vpColVector v ;

    listP.front() ;
    vpPoint P;
    std::list<vpPoint> lP ;

    // create sd
    unsigned int k =0 ;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
    {
      P = *it;
      sd[2*k] = P.get_x() ;
      sd[2*k+1] = P.get_y() ;
      lP.push_back(P) ;
      k ++;
    }
    int iter = 0 ;
    res.resize(s.getRows()/2) ;
    w.resize(s.getRows()/2) ;
    W.resize(s.getRows(), s.getRows()) ;
    w =1 ;

    while((int)((residu_1 - r)*1e12) !=0)
    {
      residu_1 = r ;

      // Compute the interaction matrix and the error
      vpPoint P ;
      k =0 ;
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it)
      {
        P = *it;
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

        k ++;

      }
      error = s - sd ;

      // compute the residual
      r = error.sumSquare() ;

      for(unsigned int k=0 ; k <error.getRows()/2 ; k++)
      {
        res[k] = vpMath::sqr(error[2*k]) + vpMath::sqr(error[2*k+1]) ;
      }
      robust.setIteration(0);
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int k=0 ; k < error.getRows()/2 ; k++)
      {
        W[2*k][2*k] = w[k] ;
        W[2*k+1][2*k+1] = w[k] ;
      }
      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp ;
      (W*L).pseudoInverse(Lp,1e-6) ;

      // compute the VVS control law
      v = -lambda*Lp*W*error ;

      cMo = vpExponentialMap::direct(v).inverse()*cMo ; ;
      if (iter++>vvsIterMax) break ;
    }
    
    if(computeCovariance)
      covarianceMatrix = vpMatrix::computeCovarianceMatrix(L,v,-lambda*error, W*W); // Remark: W*W = W*W.t() since the matrix is diagonale, but using W*W is more efficient.
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

}


