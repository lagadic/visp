/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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

#include <visp/vpConfig.h>

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
    void initTracking(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta);
    void track(vpImage<unsigned char> &I);
    void updateParameters(vpImage<unsigned char> &I, double rho, double theta);
    void updateParameters(vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta);
    void display(vpImage<unsigned char>& /*I*/, vpColor /*col*/) {;}
    
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
    void sample(vpImage<unsigned char>&image);
    void reSample(vpImage<unsigned char>&image);
    void reSample(vpImage<unsigned char>&image, vpImagePoint ip1, vpImagePoint ip2);
    void updateDelta();
    void bubbleSortI();
    void bubbleSortJ();
    void suppressPoints(vpImage<unsigned char> &I);
    void setExtremities();
    void seekExtremities(vpImage<unsigned char> &I);
    void findSignal(vpImage<unsigned char>& I, const vpMe *me, double *conv);
} ;

#endif
#endif

