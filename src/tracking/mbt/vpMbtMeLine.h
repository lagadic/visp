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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 \file vpMbtMeLine.h
 \brief Make the complete tracking of an object by using its CAD model.
*/

#ifndef vpMbtMeLine_HH
#define vpMbtMeLine_HH

#include <visp/vpPoint.h>
#include <visp/vpMe.h>
#include <visp/vpMeTracker.h>

/*!
  \class vpMbtMeLine

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtMeLine : public vpMeTracker
{
  private:
    vpMeSite PExt[2] ;
    double rho, theta, theta_1;
    double delta ,delta_1;
    int sign;
    double a,b,c;
  
  public: 
    int imin, imax;
    int jmin, jmax;
    double expecteddensity;
  
  public:  
    vpMbtMeLine();
    ~vpMbtMeLine();
    void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta);
    void track(const vpImage<unsigned char> &I);
    void updateParameters(const vpImage<unsigned char> &I, double rho, double theta);
    void updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta);
    void display(const vpImage<unsigned char>& /*I*/, vpColor /*col*/) {;}
    
     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$
   
     \return : The a coefficient of the moving edge  
    */
    inline double get_a() const { return this->a;}
    
     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$
   
     \return : The b coefficient of the moving edge  
    */
    inline double get_b() const { return this->b;}
    
     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$
   
     \return : The c coefficient of the moving edge  
    */
    inline double get_c() const { return this->c;}
  
  private:
    void sample(const vpImage<unsigned char>&image);
    void reSample(const vpImage<unsigned char>&image);
    void reSample(const vpImage<unsigned char>&image, vpImagePoint ip1, vpImagePoint ip2);
    void updateDelta();
    void bubbleSortI();
    void bubbleSortJ();
    void suppressPoints(const vpImage<unsigned char> &I);
    void setExtremities();
    void seekExtremities(const vpImage<unsigned char> &I);
    void findSignal(const vpImage<unsigned char>& I, const vpMe *me, double *conv);
} ;

#endif
#endif

