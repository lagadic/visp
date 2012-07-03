/****************************************************************************
 *
 * $Id: vpMbEdgeTracker.h 2807 2010-09-14 10:14:54Z fspindle $
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

/*!
 \file vpMbEdgeTracker.h
 \brief Make the complete tracking of an object by using its CAD model.
*/

#ifndef vpMbEdgeTracker_HH
#define vpMbEdgeTracker_HH

#include <visp/vpPoint.h>
#include <visp/vpMbTracker.h>
#include <visp/vpMe.h>
#include <visp/vpMbtMeLine.h>
#include <visp/vpMbtDistanceLine.h>
#include <visp/vpMbtDistanceCylinder.h>
#include <visp/vpXmlParser.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>

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

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#  include <visp/vpList.h>
#endif

/*!
  \class vpMbEdgeTracker
  \ingroup ModelBasedTracking 
  \brief Make the complete tracking of an object by using its CAD model.

  This class realise the tracking of an object or a scene given its 3D model. A
  video example can be found in the \e http://www.irisa.fr/lagadic/visp/computer-vision.html
  webpage.
  The tracker requires the knowledge of the 3D model (given either in a vrml
  file or in a cao file (see the loadCAOModel() method for more details)).
  It may also use an wml file used to tune the behavior of the tracker and an
  init file used to compute the pose at the very first image.

  The following code shows the simplest way to use the tracker.

\code
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpException.h>

int main()
{
  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;

  // Acquire an image

  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
  tracker.getCameraParameters(cam);   // Get the camera parameters used by the tracker (from the configuration file).
  tracker.loadModel("cube.cao");      // Load the 3d model in cao format. No 3rd party library is required
  tracker.initClick(I, "cube.init");  // Initialise manually the pose by clicking on the image points associated to the 3d points containned in the cube.init file.

  while(true){
    // Acquire a new image
    tracker.track(I);     // Track the object on this image
    tracker.getPose(cMo); // Get the pose
    tracker.display(I, cMo, cam, vpColor::darkRed, 1); // Display the model at the computed pose.
  }

  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeTracker::loadConfigFile()
  vpXmlParser::cleanup();

  return 0;
}
\endcode

  For application with large inter-images displacement, multi-scale tracking is also possible, by setting the number of scales used and by activating (or not) them 
  using a vector of booleans, as presented in the following code:

\code
  ...
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;

  std::vector< bool > scales(3); //Three scales used
  scales.push_back(true); //First scale : active
  scales.push_back(false); //Second scale (/2) : not active
  scales.push_back(true); //Third scale (/4) : active
  tracker.setScales(scales); // Set active scales for multi-scale tracking

  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
  tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
  ...
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeTracker::loadConfigFile()
  vpXmlParser::cleanup();
\endcode

  The tracker can also be used without display, in that case the initial pose
  must be known (object always at the same initial pose for example) or computed
  using another method:

\code
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

int main()
{
  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;

  //acquire an image

  // Get the cMo from a known pose or a 'client' in a server/client configuration

  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
  tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
  tracker.loadModel("cube.wrl"); // load the 3d model, to read .wrl model coi is required, if coin is not installed .cao file can be used.
  tracker.initFromPose(I, cMo); // initialise manually the pose by clicking on the image points associated to the 3d points containned in the cube.init file.

  while(true){
    // acquire a new image
    tracker.track(I); // track the object on this image
    tracker.getPose(cMo); // get the pose
  }

  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeTracker::loadConfigFile()
  vpXmlParser::cleanup();

  return 0;
}
\endcode

  Finally it can be used not to track an object but just to display a model at a
  given pose:

\code
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

int main()
{
  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;

  //acquire an image

  // Get the cMo from a known pose or a 'server' in a server/client configuration

  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
  tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
  tracker.loadModel("cube.wrl"); // load the 3d model, to read .wrl model coi is required, if coin is not installed .cao file can be used.

  while(true){
    // acquire a new image
    // Get the pose using any method
    tracker.display(I, cMo, cam, vpColor::darkRed, 1);// display the model at the given pose.
  }
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeTracker::loadConfigFile()
  vpXmlParser::cleanup();

  return 0;
}
\endcode

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
    std::vector< std::list< vpMbtDistanceLine*> > lines;
    //! Vector of the tracked cylinders.
    std::vector< std::list< vpMbtDistanceCylinder*> > cylinders;

    //! Index of the polygon to add, and total number of polygon extracted so far. 
    unsigned int nline;

    //! Index of the cylinder to add, and total number of polygon extracted so far.
    unsigned int ncylinder;
    
    //! Index of the polygon to add, and total number of polygon extracted so far. Cannot be unsigned because the default index of a polygon is -1.
    int index_polygon;
    //! Set of faces describing the object. 
    vpMbtHiddenFaces faces;
    //! Number of polygon (face) currently visible. 
    unsigned int nbvisiblepolygone;
    
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
  
  /*!
    Set the value of the gain used to compute the control law.
    
    \param lambda : the desired value for the gain.
  */
  inline void setLambda(const double lambda) {this->lambda = lambda;}
  
  void setMovingEdge(const vpMe &_me);
  void loadConfigFile(const std::string &filename);
  void loadConfigFile(const char* filename);
  void loadModel(const std::string &cad_name);
  void loadModel(const char* cad_name);

private:
	void init(const vpImage<unsigned char>& I);
	
public:
  void track(const vpImage<unsigned char> &I);
  void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1, const bool displayFullModel = false);
  void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1, const bool displayFullModel = false);
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
  double getFirstThreshold() { return percentageGdPt;}


  /*!
    Get the moving edge parameters.
    
    \return an instance of the moving edge parameters used by the tracker.
  */
  inline void getMovingEdge(vpMe &_me ) { _me = this->me;}
  
  unsigned int getNbPoints(const unsigned int _level=0);
  vpMbtPolygon* getPolygon(const unsigned int _index); 
  unsigned int getNbPolygon();
  void getLline(std::list<vpMbtDistanceLine *>& linesList, const unsigned int _level = 0);
  void getLcylinder(std::list<vpMbtDistanceCylinder *>& cylindersList, const unsigned int _level = 0);
  
  void setScales(const std::vector<bool>& _scales);
  
  /*!
    Return the scales levels used for the tracking. 
    
    \return The scales levels used for the tracking. 
  */
  std::vector<bool> getScales() const {return scales;}



  /*!
    Set the camera parameters.

    \param _cam : the new camera parameters
  */
  virtual void setCameraParameters(const vpCameraParameters& _cam) {
    this->cam = _cam; 
    cameraInitialised = true;

    for (unsigned int i = 0; i < scales.size(); i += 1){
      if(scales[i]){
        for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
          (*it)->setCameraParameters(cam);
        }

        for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
          (*it)->setCameraParameters(cam);
        }
      }
    }
  }

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
  void addCylinder(const vpPoint &P1, const vpPoint &P2, const double r, const std::string& name = "");
  void removeCylinder(const std::string& name);
  virtual void initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace = -1);
  virtual void initCylinder(const vpPoint& _p1, const vpPoint _p2, const double _radius, const unsigned int _indexCylinder=0);
  
  void testTracking();
  void initPyramid(const vpImage<unsigned char>& _I, std::vector<const vpImage<unsigned char>* >& _pyramid);
  void cleanPyramid(std::vector<const vpImage<unsigned char>* >& _pyramid);
  void reInitLevel(const unsigned int _lvl);
  void downScale(const unsigned int _scale);
  void upScale(const unsigned int _scale);

  public:
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */

  /*!
    \deprecated This method is deprecated. You should use
    void getLline(std::list<vpMbtDistanceLine *> &, const unsigned int)
    instead.
  */
  vp_deprecated vpList<vpMbtDistanceLine *>* getLline(const unsigned int _level = 0);
  /*!
    \deprecated This method is deprecated. You should use
    void getLcylinder(std::list<vpMbtDistanceCylinder *> &, const unsigned int)
    instead.
  */
  vp_deprecated vpList<vpMbtDistanceCylinder *>* getLcylinder(const unsigned int _level = 0);
  
  /*!
    \deprecated This method is deprecated. You should use
    void vpMbTracker::initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo)
    instead.
  */
  vp_deprecated void init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo) ;
#endif

  
};

#endif

