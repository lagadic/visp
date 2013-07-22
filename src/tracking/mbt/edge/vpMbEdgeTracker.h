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

  This class allows to track an object or a scene given its 3D model. A
  video can be found in the \e http://www.irisa.fr/lagadic/visp/computer-vision.html  web page. The \ref tutorial-tracking-mb is also a good starting point to use this class.

  The tracker requires the knowledge of the 3D model that could be provided in a vrml
  or in a cao file. The cao format is described in loadCAOModel().
  It may also use an xml file used to tune the behavior of the tracker and an
  init file used to compute the pose at the very first image.

  The following code shows the simplest way to use the tracker.

\code
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpException.h>
#include <visp/vpDisplayX.h>

int main()
{
  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker. 
  vpCameraParameters cam;
  
  // Acquire an image
  vpImageIo::read(I, "cube.pgm");
  
#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Edge Tracker");
#endif

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  tracker.getCameraParameters(cam);   // Get the camera parameters used by the tracker (from the configuration file).
  tracker.loadModel("cube.cao");      // Load the 3d model in cao format. No 3rd party library is required
  tracker.initClick(I, "cube.init");  // Initialise manually the pose by clicking on the image points associated to the 3d points contained in the cube.init file.

  while(true){
    // Acquire a new image
    vpDisplay::display(I);
    tracker.track(I);     // Track the object on this image
    tracker.getPose(cMo); // Get the pose
    
    tracker.display(I, cMo, cam, vpColor::darkRed, 1); // Display the model at the computed pose.
    vpDisplay::flush(I);
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
#include <visp/vpImageIo.h>

int main()
{
  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used in entry (has to be defined), then computed using the tracker. 
  
  //acquire an image
  vpImageIo::read(I, "cube.pgm"); // Example of acquisition

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  tracker.loadModel("cube.cao"); // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.
  tracker.initFromPose(I, cMo); // initialize the tracker with the given pose.

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
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>

int main()
{
  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used to display the model. 
  vpCameraParameters cam;
  
  // Acquire an image
  vpImageIo::read(I, "cube.pgm");
  
#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Edge Tracker");
#endif

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
  tracker.loadModel("cube.cao"); // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.

  while(true){
    // acquire a new image
    // Get the pose using any method
    vpDisplay::display(I);
    tracker.display(I, cMo, cam, vpColor::darkRed, 1, true); // Display the model at the computed pose.
    vpDisplay::flush(I);
  }
  
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeTracker::loadConfigFile()
  vpXmlParser::cleanup();

  return 0;
}
\endcode

*/

class VISP_EXPORT vpMbEdgeTracker: virtual public vpMbTracker
{
  protected :
    
    /*! If this flag is true, the interaction matrix
     extracted from the feature set is computed at each
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
    vpMbHiddenFaces<vpMbtPolygon> faces;
    
    //! Number of polygon (face) currently visible. 
    unsigned int nbvisiblepolygone;
    
    //! Percentage of good points over total number of points below which tracking is supposed to have failed.
    double percentageGdPt;
    
    //! Vector of scale level to use for the multi-scale tracking.
    std::vector<bool> scales;
    
    //! Pyramid of image associated to the current image. This pyramid is computed in the init() and in the track() methods.
    std::vector< const vpImage<unsigned char>* > Ipyramid;
    
    //! Current scale level used. This attribute must not be modified outside of the downScale() and upScale() methods, as it used to specify to some methods which set of distanceLine use. 
    unsigned int scaleLevel;
    
    //! Use Ogre3d for visibility tests
    bool useOgre;
  
    //! Angle used to detect a face appearance
    double angleAppears;
  
    //! Angle used to detect a face disappearance
    double angleDisappears;
    
    //! Distance for near clipping
    double distNearClip;
    
    //! Distance for near clipping
    double distFarClip;
    
    //! Flags specifying which clipping to used
    unsigned int clippingFlag;
  
public:
  
  vpMbEdgeTracker(); 
  virtual ~vpMbEdgeTracker();
  
  void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false);
  void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false);
  
  /*! Return the angle used to test polygons appearance. */
  virtual inline double getAngleAppear() { return angleAppears; }   
  
  /*! Return the angle used to test polygons disappearance. */
  virtual inline double getAngleDisappear() { return angleDisappears; } 
  
  /*!
    Get the clipping used.
    
    \sa vpMbtPolygonClipping
    
    \return Clipping flags.
  */          
  virtual inline  unsigned int getClipping() const { return clippingFlag; } 
  
  /*! Return a reference to the faces structure. */
  vpMbHiddenFaces<vpMbtPolygon> & getFaces() { return faces;}
  
  /*!
    Get the far distance for clipping.
    
    \return Far clipping value.
  */
  virtual inline double getFarClippingDistance() const { return distFarClip; }
  
  double getFirstThreshold() { return percentageGdPt;}
  
  void getLline(std::list<vpMbtDistanceLine *>& linesList, const unsigned int level = 0);
  void getLcylinder(std::list<vpMbtDistanceCylinder *>& cylindersList, const unsigned int level = 0);
  
  /*!
    Get the moving edge parameters.
    
    \return an instance of the moving edge parameters used by the tracker.
  */
  inline void getMovingEdge(vpMe &me ) { me = this->me;}
  
  /*!
    Get the near distance for clipping.
    
    \return Near clipping value.
  */
  virtual inline double getNearClippingDistance() const { return distNearClip; }
  
  unsigned int getNbPoints(const unsigned int level=0);
  unsigned int getNbPolygon();
  vpMbtPolygon* getPolygon(const unsigned int index);
  
  /*!
    Return the scales levels used for the tracking. 
    
    \return The scales levels used for the tracking. 
  */
  std::vector<bool> getScales() const {return scales;}
  
  void loadConfigFile(const std::string &configFile);
  void loadConfigFile(const char* configFile);
  void loadModel(const std::string &cad_name);
  void loadModel(const char* cad_name);  
  
  void reInitModel(const vpImage<unsigned char>& I, const char* cad_name, const vpHomogeneousMatrix& cMo);
  void resetTracker();
  
  /*!
    Set the angle used to test polygons appearance.
    If the angle between the normal of the polygon and the line going
    from the camera to the polygon center has a value lower than
    this parameter, the polygon is considered as appearing.
    The polygon will then be tracked.

    \warning This angle will only be used when setOgreVisibilityTest(true)
    is called.

    \param a : new angle in radian.
  */
  virtual inline  void setAngleAppear(const double &a) { angleAppears = a; }   
  
  /*!
    Set the angle used to test polygons disappearance.
    If the angle between the normal of the polygon and the line going
    from the camera to the polygon center has a value greater than
    this parameter, the polygon is considered as disappearing.
    The tracking of the polygon will then be stopped.

    \warning This angle will only be used when setOgreVisibilityTest(true)
    is called.

    \param a : new angle in radian.
  */
  virtual inline void setAngleDisappear(const double &a) { angleDisappears = a; }
  
  /*!
    Set the camera parameters.

    \param cam : the new camera parameters
  */
  virtual void setCameraParameters(const vpCameraParameters& cam) {
    this->cam = cam;

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
  
  virtual void  setClipping(const unsigned int &flags);
  
  /*!
    Enable to display the points along the line with a color corresponding to their state.
    
    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If red : The point is removed because of the robust method in the virtual visual servoing.
    
    \param displayMe : set it to true to display the points.
  */
  void setDisplayMovingEdges(const bool displayMe) {displayFeatures = displayMe;}
  
  virtual void setFarClippingDistance(const double &dist);
  
  /*!
    Set the first threshold used to check if the tracking failed. It corresponds to the percentage of good point which is necessary.
    
    The condition which has to be be satisfied is the following : \f$ nbGoodPoint > threshold1 \times (nbGoodPoint + nbBadPoint)\f$.
    
    The threshold is ideally between 0 and 1.
    
    \param threshold1 : The new value of the threshold. 
  */
  void setFirstThreshold(const double  threshold1) {percentageGdPt = threshold1;}
  
  /*!
    Set the value of the gain used to compute the control law.
    
    \param lambda : the desired value for the gain.
  */
  inline void setLambda(const double lambda) {this->lambda = lambda;}
  
  void setMovingEdge(const vpMe &me);
  
  virtual void setNearClippingDistance(const double &dist);
  
  virtual void setOgreVisibilityTest(const bool &v);
  
  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo);
  
  void setScales(const std::vector<bool>& _scales);

  void track(const vpImage<unsigned char> &I);

protected:
  void addCylinder(const vpPoint &P1, const vpPoint &P2, const double r, const std::string& name = "");
  void addLine(vpPoint &p1, vpPoint &p2, int polygone = -1, std::string name = "");
  void addPolygon(vpMbtPolygon &p) ;
  void cleanPyramid(std::vector<const vpImage<unsigned char>* >& _pyramid);
  void computeVVS(const vpImage<unsigned char>& _I);
  void downScale(const unsigned int _scale);
  void init(const vpImage<unsigned char>& I);
  virtual void initCylinder(const vpPoint& _p1, const vpPoint _p2, const double _radius, const unsigned int _indexCylinder=0);
  virtual void initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace = -1);
  void initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo) ;
  void initPyramid(const vpImage<unsigned char>& _I, std::vector<const vpImage<unsigned char>* >& _pyramid);
  void reInitLevel(const unsigned int _lvl);
  void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo);
  void removeCylinder(const std::string& name);
  void removeLine(const std::string& name);
  void testTracking();
  void trackMovingEdge(const vpImage<unsigned char> &I) ;
  void updateMovingEdge(const vpImage<unsigned char> &I) ;
  void upScale(const unsigned int _scale); 
  void visibleFace(const vpImage<unsigned char> &_I, const vpHomogeneousMatrix &_cMo, bool &newvisibleline) ; 
  
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  vp_deprecated void visibleFace(const vpHomogeneousMatrix &_cMo, bool &newvisibleline);
#endif
};

#endif

