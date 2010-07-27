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
    int index;
    int nbpt;
    bool isvisible;
    bool isappearing;
    
  private:
    double angle_1;
    int negative;
    
  public: 
    vpPoint *p ;
    vpMbtPolygon() ;
    ~vpMbtPolygon() ;
    void setIndex( int i ) { index = i ; } 
    void setNbPoint(int nb)  ;
    int getNbPoint() {return nbpt ;  }
    void addPoint(int n, vpPoint &P) ; 

    int getIndex() {return index ;}
    void changeFrame(vpHomogeneousMatrix &cMo) ;
    bool isVisible(vpHomogeneousMatrix &cMo) ;
    bool isVisible() {return isvisible;}
    bool isAppearing() {return isappearing;}
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
    int setVisible(vpHomogeneousMatrix &cMo) ;
    bool isVisible(int index) ;
    bool isAppearing(int index);
    void reset();
    
    vpList<vpMbtPolygon *>& getPolygon() { return Lpol;}
} ;

#endif
#endif

