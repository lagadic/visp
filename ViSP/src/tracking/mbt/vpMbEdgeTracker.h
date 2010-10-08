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

/*!
  \class vpMbEdgeTracker
  \ingroup ModelBasedTracking
  \brief Make the complete tracking of an object by using its CAD model.
*/

class VISP_EXPORT vpMbEdgeTracker
{
  protected :
    vpCameraParameters cam ;
    std::string modelFileName;
    
    /* If this flag is true, the interaction matrix
     extracted from the FeatureSet is computed at each
     iteration in the visual servoing loop.
    */
    int compute_interaction;
    double lambda;
    vpHomogeneousMatrix cMo;
    
    vpMe  me;
    vpList<vpMbtDistanceLine *> Lline ;
    int nline ;
    
    int index_polygon;
    vpMbtHiddenFaces faces;
    int nbvisiblepolygone;
    
    vpMbtPolygon *caoPolygonPoint;
    vpMbtPolygon *caoPolygonLine;
    
    bool displayMe;
    
    double percentageGdPt;
   // double percentageTtPt;
  
 public:
  
  vpMbEdgeTracker(); 
  virtual ~vpMbEdgeTracker(); 
  
  /*!
    Set the value of the gain used to compute the control law.
    
    \param lambda : the desired value for the gain.
  */
  inline void setLambda(const double lambda) {this->lambda = lambda;}
  
  /*! 
    Get the pose of the camera.
    cMo is the matrix which can be used to express 
    coordinates from the object frame to camera frame.
    
    \param cMo : the vpHomogeneousMatrix used to store the pose.
  */
  inline void getPose(vpHomogeneousMatrix &cMo) const {cMo = this->cMo;}

  /*!
   Get the camera paramters.
   
   \param cam : The vpCameraParameters used to store the camera parameters.
 */
  inline void getCameraParameters(vpCameraParameters &cam) const {cam = this->cam;}
  
  /*!
   Set the camera paramters.
   \param cam : The camera parameters.
 */
  inline void setCameraParameters(const vpCameraParameters &cam) {this->cam = cam;}
  
  void setMovingEdge(vpMe &_me);
  void loadConfigFile(const char* filename);
  void loadModel(const char* cad_name);
  void initClick(const vpImage<unsigned char>& I, const char *filename, bool displayHelp = false);
  void init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo) ;
  void track(const vpImage<unsigned char> &I);
  void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1);
  void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1);
  void resetTracker();
  void reInitModel(vpImage<unsigned char>& I, const char* cad_name, vpHomogeneousMatrix& _cMo);
  
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
  */
  void setFirstThreshold(const double  threshold1) {percentageGdPt = threshold1;}


  /*!
    get the moving edge parameters.
    
    \return an instance of the moving edge parameters used by the tracker.
  */
  inline void getMovingEdge(vpMe &_me ) { _me = this->me;}
  
  unsigned int getNbPoints();

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
  void loadCAOModel(std::ifstream &file_id);
  void computeJTR(const vpMatrix& interaction, const vpColVector& error, vpMatrix& JTR);
  
#if defined(VISP_HAVE_COIN)
  void loadVRMLModel(const char* file_id);
  void extractFaces(SoVRMLIndexedFaceSet* face_set);
  void extractLines(SoVRMLIndexedLineSet* line_set);
#endif  
  void testTracking();
    
 public:
	vpList<vpMbtDistanceLine *>* getLline(){ return &(this->Lline);};
};





#endif
