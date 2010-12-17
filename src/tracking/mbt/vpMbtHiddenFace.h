/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 \file vpMbtHiddenFace.h
 \brief Make the complete tracking of an object by using its CAD model.
*/

#ifndef vpMbtHiddenFace_HH
#define vpMbtHiddenFace_HH

#include <visp/vpConfig.h>

#include <visp/vpPoint.h>
#include <visp/vpList.h>

/*!
  \class vpMbtPolygon

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtPolygon
{
  public:
    //! Index of the polygon. Cannot be unsigned int because deafult value is -1.
    int index;
    //! Number of points used to define the polygon.
    unsigned int nbpt;
    bool isvisible;
    bool isappearing;
    
  private:
    double angle_1;
    int negative;
    
  public: 
    vpPoint *p ;
    vpMbtPolygon() ;
    ~vpMbtPolygon() ;
    void setIndex(const int i ) { index = i ; } 
    void setNbPoint(const unsigned int nb)  ;
    unsigned int getNbPoint() const {return nbpt ;  }
    void addPoint(const unsigned int n, const vpPoint &P) ; 

    int getIndex() const {return index ;}
    void changeFrame(const vpHomogeneousMatrix &cMo) ;
    bool isVisible(const vpHomogeneousMatrix &cMo) ;
    bool isVisible(const vpHomogeneousMatrix &cMo, const double alpha) ;
    bool isVisible() const {return isvisible;}
    bool isAppearing() const {return isappearing;}
};

#if defined(VISP_BUILD_SHARED_LIBS) && defined(VISP_USE_MSVC)
template class VISP_EXPORT vpList<vpMbtPolygon *>;
#endif

/*!
  \class vpMbtHiddenFaces

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtHiddenFaces
{
  private:
    vpList<vpMbtPolygon *> Lpol ;
  public :
    vpMbtHiddenFaces() ;
    ~vpMbtHiddenFaces() ;
    void addPolygon(vpMbtPolygon *p)  ;
    unsigned int setVisible(const vpHomogeneousMatrix &cMo) ;
    bool isVisible(const int index) ;
    bool isAppearing(const int index);
    void reset();
    
    vpList<vpMbtPolygon *>& getPolygon() { return Lpol;}
} ;

#endif
#endif

