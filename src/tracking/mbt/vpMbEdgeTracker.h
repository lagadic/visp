/****************************************************************************
 *
 * $Id: vpMbEdgeTracker.h 2807 2010-09-14 10:14:54Z fspindle $
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

/*!
 \file vpMbEdgeTracker.h
 \brief Make the complete tracking of an object by using its CAD model.
*/

#ifndef vpMbEdgeTracker_HH
#define vpMbEdgeTracker_HH

#include <visp/vpConfig.h>

#include <visp/vpMbTracker.h>
#include <visp/vpPoint.h>
#include <visp/vpMe.h>
#include <visp/vpMbtMeLine.h>
#include <visp/vpMbtDistanceLine.h>

#include <iostream>
#include <fstream>

#if defined(VISP_HAVE_COIN)
//Inventor includes
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#endif

#ifdef VISP_HAVE_OPENCV
#  if VISP_HAVE_OPENCV_VERSION >= 0x020101
#    include <opencv2/core/core.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#    include <opencv2/imgproc/imgproc_c.h>
#  else
#    include <cv.h>
#  endif
#endif

/*!
  \class vpMbEdgeTracker
  \ingroup ModelBasedTracking 
  \brief Make the complete tracking of an object by using its CAD model.
*/

class VISP_EXPORT vpMbEdgeTracker: public vpMbTracker
{
  protected :
    
    /*! If this flag is true, the interaction matrix
     extracted from the FeatureSet is computed at each
     iteration in the visual servoing loop.
    */
    int compute_interaction;
    //! The gain of the virtual visual servoing stage. 
    double lambda;
    
    //! The moving edges parameters. 
    vpMe  me;
    //! Vector of list of all the lines tracked (each line is linked to a list of moving edges). Each element of the vector is for a scale (element 0 = level 0 = no subsampling).
    std::vector< vpList< vpMbtDistanceLine*> > lines;
    //! Index of the polygon to add, and total number of polygon extracted so far. 
    int nline ;
    
    //! Index of the polygon to add, and total number of polygon extracted so far. 
    int index_polygon;
    //! Set of faces describing the object. 
    vpMbtHiddenFaces faces;
    //! Number of polygon (face) currently visible. 
    int nbvisiblepolygone;
    
    //! If true, the moving edges are displayed during the track() method. 
    bool displayMe;
    
    //! Percentage of good points over total number of points below which tracking is supposed to have failed.
    double percentageGdPt;
    
    //! Vector of scale level to use for the multi-scale tracking.
    std::vector<bool> scales;
    
    //! Pyramid of image associated to the current image. This pyramid is compted in the init() and in the track() methods.
    std::vector< const vpImage<unsigned char>* > Ipyramid;
    
    //! Current scale level used. This attribute must not be modified outsied of the downScale() and upScale() methods, as it used to specify to some methods which set of distanceLine use. 
    unsigned int scaleLevel;
  
 public:
  
  vpMbEdgeTracker(); 
  virtual ~vpMbEdgeTracker(); 
  
  inline void setPose(const vpHomogeneousMatrix &cMo) {this->cMo = cMo;}
  
  /*!
    Set the value of the gain used to compute the control law.
    
    \param lambda : the desired value for the gain.
  */
  inline void setLambda(const double lambda) {this->lambda = lambda;}
  
  void setMovingEdge(const vpMe &_me);
  void loadConfigFile(const std::string& _filename);
  void loadConfigFile(const char* filename);
  void loadModel(const char* cad_name);
  void init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo) ;
  void track(const vpImage<unsigned char> &I);
  void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1);
  void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1);
  void resetTracker();
  void reInitModel(const vpImage<unsigned char>& I, const char* cad_name, const vpHomogeneousMatrix& _cMo);
   
  /*!
    Enable to display the points along the line with a color corresponding to their state.
    
    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If red : The point is removed because of the robust method in the virtual visual servoing.
    
    \param displayMe : set it to true to display the points.
  */
  void setDisplayMovingEdges(const bool displayMe) {this->displayMe = displayMe;}
  
  /*!
    Set the first threshold used to check if the tracking failed. It corresponds to the percentage of good point which is necessary.
    
    The condition which has to be be satisfied is the following : \f$ nbGoodPoint > threshold1 \times (nbGoodPoint + nbBadPoint)\f$.
    
    The threshold is ideally between 0 and 1.
    
    \param threshold1 : The new value of the threshold. 
  */
  void setFirstThreshold(const double  threshold1) {percentageGdPt = threshold1;}


  /*!
    Get the moving edge parameters.
    
    \return an instance of the moving edge parameters used by the tracker.
  */
  inline void getMovingEdge(vpMe &_me ) { _me = this->me;}
  
  unsigned int getNbPoints(const unsigned int _level=0);
  vpMbtPolygon* getPolygon(const unsigned int _index); 
  unsigned int getNbPolygon();
  vpList<vpMbtDistanceLine *>* getLline(const unsigned int _level = 0);
  
  void setScales(const std::vector<bool>& _scales);
  
  /*!
    Return the scales levels used for the tracking. 
    
    \return The scales levels used for the tracking. 
  */
  std::vector<bool> getScales() const {return scales;}

 protected:
  void computeVVS(const vpImage<unsigned char>& _I);
  void initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo) ;
  void trackMovingEdge(const vpImage<unsigned char> &I) ;
  void updateMovingEdge(const vpImage<unsigned char> &I) ;
  void visibleFace(const vpHomogeneousMatrix &cMo, bool &newvisibleline) ;
  void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo);
  void addPolygon(vpMbtPolygon &p) ;
  void addLine(vpPoint &p1, vpPoint &p2, int polygone = -1, std::string name = "");
  void removeLine(const std::string& name);
  virtual void initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace = -1);
  
  void testTracking();
  void initPyramid(const vpImage<unsigned char>& _I, std::vector<const vpImage<unsigned char>* >& _pyramid);
  void cleanPyramid(std::vector<const vpImage<unsigned char>* >& _pyramid);
  void reInitLevel(const unsigned int _lvl);
  void downScale(const unsigned int _scale);
  void upScale(const unsigned int _scale);
  
};

#endif

