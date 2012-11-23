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
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 \file vpMbtHiddenFace.h
 \brief Make the complete tracking of an object by using its CAD model.
*/

#ifndef vpMbtHiddenFace_HH
#define vpMbtHiddenFace_HH

#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>

#include <visp/vpPoint.h>
#include <vector>
#include <list>

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
  //! flag to specify whether the face is visible or not
  bool isvisible;
  //! flag to specify whether the face is appearing or not
  bool isappearing;
  //! corners in the object frame
  vpPoint *p;
    
public: 
                           vpMbtPolygon() ;
                          ~vpMbtPolygon() ;
                                    
            void          addPoint(const unsigned int n, const vpPoint &P) ;               
          
            void          changeFrame(const vpHomogeneousMatrix &cMo) ;
            
   /*!
    Get the index of the face.

    \return index : the index of the face.
  */
  inline    int           getIndex() const {return index ;}
            
  /*!
    Return the number of corners.

    \return number of corner of the face
  */
  inline    unsigned int  getNbPoint() const {return nbpt ;}              
          
  vpPoint &     getPoint(const unsigned int _index);

  inline    bool          isAppearing() const {return isappearing;}
            bool          isVisible(const vpHomogeneousMatrix &cMo, const bool &depthTest = false) ;
  virtual   bool          isVisible(const vpHomogeneousMatrix &cMo, const double alpha) ;
            bool          isVisible() const {return isvisible;}
  
  /*!
    Set the index of the face.

    \param i : the new index of the face.
  */
  virtual inline void     setIndex(const int i ) { index = i ; } 
  virtual        void     setNbPoint(const unsigned int nb)  ;
};

/*!
  \class vpMbtHiddenFaces

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtHiddenFaces
{
  private:
  std::list<vpMbtPolygon *> Lpol ;
  //! Boolean specifying if a polygon has to be entirely in front of the camera or not.
  bool depthTest;
  
  public :
                    vpMbtHiddenFaces() ;
                  ~vpMbtHiddenFaces() ;
                
    void          addPolygon(vpMbtPolygon *p)  ;
    
    /*!
      Get the depthTest value.

      \return true if all the points of a polygon has to be in front of the camera, false otherwise.
    */
    bool          getDepthTest(){return depthTest;}
    
    std::list<vpMbtPolygon *>& getPolygon() {return Lpol;}
    bool          isAppearing(const int index);
    bool          isVisible(const int index) ;

    void          reset();   
    
    /*!
      Set the depthTest value.

      \param d : New value.
    */
    void          setDepthTest(const bool &d){depthTest = d;} 
    unsigned int  setVisible(const vpHomogeneousMatrix &cMo) ;
} ;

#endif

