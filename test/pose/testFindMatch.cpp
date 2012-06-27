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
 * Compute the pose of a 3D object using the Dementhon method. Assuming that
 * the correspondance between 2D points and 3D points is not done, we use
 * the RANSAC algorithm to achieve this task
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>

#include <stdlib.h>
#include <stdio.h>

#define L 0.1


/*!
  \example testFindMatch.cpp

  Find Matches using Ransac.

*/

int
main()
{
  {
    std::cout << "Find Matches using Ransac" << std::endl;
    unsigned int nb3D = 4;
    unsigned int nb2D = 8;
    std::vector<vpPoint> P(nb3D);
    std::vector<vpPoint> p(nb2D);

    P[0].setWorldCoordinates(-L,-L, 0 ) ;
    P[1].setWorldCoordinates(L,-L, 0 ) ;
    P[2].setWorldCoordinates(L,L, 0 ) ;
    P[3].setWorldCoordinates(-L,L, 0 ) ;
    //P[4].setWorldCoordinates(-0,0, L ) ; //ERREUR DANS LAGRANGE ET DEMENTHON

    vpHomogeneousMatrix cMo_ref(0,0.2,1,vpMath::rad(0),0,0) ;
    
    for(unsigned int i=0 ; i < nb3D ; i++)
    {
      vpPoint pt = P[i];
      pt.project(cMo_ref);
      p[i].set_x(pt.get_x());
      p[i].set_y(pt.get_y());
    }

    p[4].set_x(0.02) ;
    p[4].set_y(0.05) ;
    
    p[5].set_x(0.02) ;
    p[5].set_y(-0.05) ;
    
    p[6].set_x(0.07) ;
    p[6].set_y(-0.05) ;
    
    p[7].set_x(0.24) ;
    p[7].set_y(0.07) ;

    unsigned int ninliers ;
    std::vector<vpPoint> inliers;
    double threshold = 1e-6;
    unsigned int nbInlierToReachConsensus = 4;
    
    vpHomogeneousMatrix cMo ;
    
    vpPose::findMatch(p,P,nbInlierToReachConsensus,threshold,ninliers,inliers,cMo);
    
    std::cout << "Inliers: " << std::endl;
    for (unsigned int i = 0; i < inliers.size() ; i++)
    {
      inliers[i].print() ;
      std::cout << std::endl;
    }
    
    std::cout << "cMo :\n" << vpPoseVector(cMo).t() << std::endl << std::endl;

  }
}
