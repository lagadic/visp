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
 * Description:
 * Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 \file vpMbEdgeKltTracker.h
 \brief Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
*/

#ifndef vpMbEdgeKltTracker_HH
#define vpMbEdgeKltTracker_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_OPENCV

#include <visp/vpRobust.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMbtXmlParser.h>
#include <visp/vpMbTracker.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpPoseVector.h>
#include <visp/vpMbKltTracker.h>

/*!
  \class vpMbEdgeKltTracker
  \ingroup ModelBasedTracking
  \warning This class is only available if OpenCV is installed, and used.
  
  \brief Hybrid tracker based on moving-edges and keypoints tracked using KLT 
  tracker.
  
  The tracker requires the knowledge of the 3D model that could be provided in a vrml
  or in a cao file. The cao format is described in loadCAOModel().
  It may also use an xml file used to tune the behavior of the tracker and an
  init file used to compute the pose at the very first image.

  The following code shows the simplest way to use the tracker. The \ref tutorial-tracking-mb is also a good starting point to use this class.
  
\code
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpException.h>
#include <visp/vpDisplayX.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbEdgeKltTracker tracker; // Create an hybrid model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;

  // Acquire an image
  vpImageIo::read(I, "cube.pgm");

#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Hybrid Tracker");
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

#if defined VISP_HAVE_XML2
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode  

  The tracker can also be used without display, in that case the initial pose
  must be known (object always at the same initial pose for example) or computed
  using another method:

\code
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImageIo.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbEdgeKltTracker tracker; // Create an hybrid model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used in entry (has to be defined), then computed using the tracker. 
  
  //acquire an image
  vpImageIo::read(I, "cube.pgm"); // Example of acquisition

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  tracker.loadModel("cube.cao"); // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.
  tracker.initFromPose(I, cMo); // initialise the tracker with the given pose.

  while(true){
    // acquire a new image
    tracker.track(I); // track the object on this image
    tracker.getPose(cMo); // get the pose 
  }
  
#if defined VISP_HAVE_XML2
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode

  Finally it can be used not to track an object but just to display a model at a
  given pose:

\code
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbEdgeKltTracker tracker; // Create an hybrid model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used to display the model. 
  vpCameraParameters cam;
  
  // Acquire an image
  vpImageIo::read(I, "cube.pgm");
  
#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Hybrid Tracker");
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
  
#endif
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode
*/
class VISP_EXPORT vpMbEdgeKltTracker: public vpMbKltTracker, public vpMbEdgeTracker
{
protected:
  //! If true, compute the interaction matrix at each iteration of the minimization. Otherwise, compute it only on the first iteration.
  bool compute_interaction;
  //! The gain of the virtual visual servoing stage.
  double lambda;
  //! The threshold used in the robust estimation of KLT.
  double thresholdKLT;
  //! The threshold used in the robust estimation of MBT.
  double thresholdMBT;
  //! The maximum iteration of the virtual visual servoing stage.
  unsigned int  maxIter;

public:
  
  vpMbEdgeKltTracker();
  virtual         ~vpMbEdgeKltTracker();

  virtual void    display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                          const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false);
  virtual void    display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                          const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false);
  
  /*! Return the angle used to test polygons appearance. */
  virtual inline  double  getAngleAppear() { return vpMbKltTracker::getAngleAppear(); }   
  
          /*! Return the angle used to test polygons disappearance. */
  virtual inline  double  getAngleDisappear() { return vpMbKltTracker::getAngleDisappear(); } 
  
          /*!
            Get the clipping used.
            
            \sa vpMbtPolygonClipping
            
            \return Clipping flags.
          */          
  inline  unsigned int    getClipping() { return vpMbKltTracker::clippingFlag; } 
  
  /*! Return a reference to the faces structure. */
  vpMbHiddenFaces<vpMbtKltPolygon> & getFaces() { return vpMbKltTracker::faces;}

          /*!
            Get the value of the gain used to compute the control law.

            \return the value for the gain.
          */
  inline  double  getLambda() {return lambda;}

          /*!
            Get the maximum iteration of the virtual visual servoing stage.

            \return the number of iteration
          */
  inline  unsigned int getMaxIter() {return maxIter;}
  
          /*!
            Get the near distance for clipping.
            
            \return Near clipping value.
          */
  inline  double  getNearClippingDistance() { return vpMbKltTracker::getNearClippingDistance(); }

          void    loadConfigFile(const char* configFile);
  virtual void    loadConfigFile(const std::string& configFile);
  virtual void    loadModel(const std::string& modelFile);
  
          void    resetTracker();
          
          /*!
            Set the angle used to test polygons appearance.
            If the angle between the normal of the polygon and the line going
            from the camera to the polygon center has a value lower than
            this parameter, the polygon is considered as appearing.
            The polygon will then be tracked.

            \param a : new angle in radian.
          */
  virtual inline  void setAngleAppear(const double &a) { vpMbKltTracker::setAngleAppear(a); }
  
          /*!
            Set the angle used to test polygons disappearance.
            If the angle between the normal of the polygon and the line going
            from the camera to the polygon center has a value greater than
            this parameter, the polygon is considered as disappearing.
            The tracking of the polygon will then be stopped.

            \param a : new angle in radian.
          */
  virtual inline  void setAngleDisappear(const double &a) { vpMbKltTracker::setAngleDisappear(a); }

  virtual void    setCameraParameters(const vpCameraParameters& cam);
  
          /*!
            Specify which clipping to use.
            
            \sa vpMbtPolygonClipping
            
            \param flags : New clipping flags.
          */
          void    setClipping(const unsigned int &flags) {vpMbEdgeTracker::setClipping(flags); vpMbKltTracker::setClipping(flags);}

          /*!
            Set the value of the gain used to compute the control law.

            \param lambda : the desired value for the gain.
          */
  inline  void    setLambda(const double lambda) {this->lambda = lambda; vpMbEdgeTracker::setLambda(lambda); vpMbKltTracker::setLambda(lambda);}

          /*!
            Set the maximum iteration of the virtual visual servoing stage.

            \param max : the desired number of iteration
          */
  inline  void    setMaxIter(const unsigned int max) {maxIter = max;}
  
          /*!
            Set the near distance for clipping.
            
            \param dist : Near clipping value.
          */
  inline  void   setNearClippingDistance(const double &dist) { vpMbEdgeTracker::setNearClippingDistance(dist); vpMbKltTracker::setNearClippingDistance(dist); }
          
          /*!
            Use Ogre3D for visibility tests
            
            \warning This function has to be called before the initialization of the tracker.
            
            \param v : True to use it, False otherwise
          */
  virtual inline  void    setOgreVisibilityTest(const bool &v) { vpMbKltTracker::setOgreVisibilityTest(v); }
  
  virtual void    setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo);

  virtual void    testTracking(){};
  virtual void    track(const vpImage<unsigned char>& I);

protected:
          void    computeVVS(const vpImage<unsigned char>& I, const unsigned int &nbInfos, vpColVector &w_mbt,
                             vpColVector &w_klt, const unsigned int lvl=0);

  virtual void    init(const vpImage<unsigned char>& I);
  virtual void    initCylinder(const vpPoint& , const vpPoint , const double , const unsigned int );
  virtual void    initFaceFromCorners(const std::vector<vpPoint>& corners, const unsigned int indexFace = -1);
  unsigned int    initMbtTracking(const unsigned int level=0);

          bool    postTracking(const vpImage<unsigned char>& I, vpColVector &w_mbt, vpColVector &w_klt,
                               const unsigned int lvl=0);
          void    postTrackingMbt(vpColVector &w, const unsigned int level=0);

  unsigned int    trackFirstLoop(const vpImage<unsigned char>& I, vpColVector &factor, const unsigned int lvl = 0);
          void    trackSecondLoop(const vpImage<unsigned char>& I, vpMatrix &L, vpColVector &_error,
                                  vpHomogeneousMatrix& cMo, const unsigned int lvl=0);
};

#endif

#endif //VISP_HAVE_OPENCV
