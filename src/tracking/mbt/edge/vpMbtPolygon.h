/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Implements a polygon of the model used by the model-based tracker.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 \file vpMbtPolygon.h
 \brief Implements a polygon of the model used by the model-based tracker.
*/

#ifndef vpMbtPolygon_HH
#define vpMbtPolygon_HH

#include <visp/vpPoint.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>

#include <vector>

/*!
  \class vpMbtPolygon
  
  \brief Implementation of a polygon of the model used by the model-based tracker.

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtPolygon
{
public:
  typedef enum
    {
      NO_CLIPPING = 0,
      NEAR_CLIPPING = 1,
      FAR_CLIPPING = 2,
      LEFT_CLIPPING = 4,
      RIGHT_CLIPPING = 8,
      UP_CLIPPING = 16,
      DOWN_CLIPPING = 32,
      FOV_CLIPPING = 60,
      ALL_CLIPPING = 63
    } vpMbtPolygonClippingType;
  
public:
  //! Index of the polygon. Cannot be unsigned int because default value is -1.
  int index;
  //! Number of points used to define the polygon.
  unsigned int nbpt;
  //! Number of corners inside the image during the last call to getNbCornerInsideImage
  unsigned int nbCornersInsidePrev;
  //! flag to specify whether the face is visible or not
  bool isvisible;
  //! flag to specify whether the face is appearing or not
  bool isappearing;
  //! corners in the object frame
  vpPoint *p;
  //! Region of interest clipped
  std::vector<std::pair<vpPoint,unsigned int> > roiPointsClip;
  //! Clipping flag
  unsigned int clippingFlag;
  //! Distance for near clipping
  double distNearClip;
  //! Distance for near clipping
  double distFarClip;
  
private:            
            bool          getClippedPointsFov(const vpPoint &p1, const vpPoint &p2, 
                                           vpPoint &p1Clipped, vpPoint &p2Clipped, 
                                           unsigned int &p1ClippedInfo, unsigned int &p2ClippedInfo,
                                           const vpColVector &normal,
                                           const unsigned int &flag);
            
            bool          getClippedPoints(const vpPoint &p1, const vpPoint &p2, 
                                           vpPoint &p1Clipped, vpPoint &p2Clipped, 
                                           unsigned int &p1ClippedInfo, unsigned int &p2ClippedInfo,
                                           const vpCameraParameters &cam, const std::vector<vpColVector> &fovNormals);
    
public: 
                           vpMbtPolygon() ;
            virtual       ~vpMbtPolygon() ;
                                    
            void          addPoint(const unsigned int n, const vpPoint &P) ;               
          
            void          changeFrame(const vpHomogeneousMatrix &cMo) ;
            
            void          computeRoiClipped(const vpCameraParameters &cam = vpCameraParameters());
   
  /*!
    Get the clipping used.
    
    \sa vpMbtPolygonClipping
    
    \return Clipping flags.
  */          
  inline    unsigned int  getClipping() const { return clippingFlag; } 
  
   /*!
    Get the far distance for clipping.
    
    \return Far clipping value.
  */
  inline    double       getFarClippingDistance() const { return distFarClip; }
  
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
  
  /*!
    Return the number of corners at the previous computation.

    \return number of corner of the face at the previous computation
  */
  inline    unsigned int  getNbCornerInsidePrevImage() const { return nbCornersInsidePrev; }
  
            unsigned int  getNbCornerInsideImage(const vpImage<unsigned char>& I, const vpCameraParameters &cam);
            
  /*!
    Get the near distance for clipping.
    
    \return Near clipping value.
  */
  inline    double        getNearClippingDistance() const { return distNearClip; }
          
            vpPoint &     getPoint(const unsigned int _index);
            
  std::vector<vpImagePoint> getRoi(const vpCameraParameters &cam);

  std::vector<vpImagePoint> getRoi(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo);

            void          getRoiClipped(const vpCameraParameters &cam, std::vector<vpImagePoint>&roi);

            void          getRoiClipped(const vpCameraParameters &cam, std::vector<vpImagePoint>&roi, const vpHomogeneousMatrix &cMo);
    
            void          getRoiClipped(const vpCameraParameters &cam, std::vector<std::pair<vpImagePoint,unsigned int> > &roi);

            void          getRoiClipped(const vpCameraParameters &cam, std::vector<std::pair<vpImagePoint,unsigned int> > &roi, const vpHomogeneousMatrix &cMo);

  inline    bool          isAppearing() const {return isappearing;}
  virtual   bool          isVisible(const vpHomogeneousMatrix &cMo, const double alpha, const bool &modulo = false) ;
            bool          isVisible() const {return isvisible;}
            
  /*!
    Specify which clipping to use.
    
    \sa vpMbtPolygonClipping
    
    \param flags : New clipping flags.
  */
  inline    void          setClipping(const unsigned int &flags) { clippingFlag = flags; }
  
  /*!
    Set the far distance for clipping.
    
    \param dist : Far clipping value.
  */
  inline    void          setFarClippingDistance(const double &dist) { distFarClip = dist; clippingFlag = (clippingFlag | vpMbtPolygon::FAR_CLIPPING);}
  
  /*!
    Set the index of the face.

    \param i : the new index of the face.
  */
  virtual inline void     setIndex(const int i ) { index = i ; } 
  virtual        void     setNbPoint(const unsigned int nb)  ;
  
  /*!
    Set the near distance for clipping.
    
    \param dist : Near clipping value.
  */
  inline    void          setNearClippingDistance(const double &dist) { distNearClip = dist; clippingFlag = (clippingFlag | vpMbtPolygon::NEAR_CLIPPING);}
  
public:
  static   void           getMinMaxRoi(const std::vector<vpImagePoint> &roi, int & i_min, int &i_max, int &j_min, int &j_max);
  static   bool           roiInsideImage(const vpImage<unsigned char>& I, const std::vector<vpImagePoint>& corners);
  
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
  /*!
    @name Deprecated functions
  */
  bool      isVisible(const vpHomogeneousMatrix &cMo, const bool &depthTest = false) ;
#endif
};

#endif

