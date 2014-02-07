/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Model based tracker using only KLT
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/
/*!
 \file vpMbKltTracker.h
 \brief Model based tracker using only KLT
*/

#ifndef vpMbKltTracker_h
#define vpMbKltTracker_h

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_OPENCV

#include <visp/vpMbTracker.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpMbtKltPolygon.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMbtKltXmlParser.h>
#include <visp/vpHomography.h>
#include <visp/vpRobust.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMbtKltPolygon.h>

/*!
  \class vpMbKltTracker
  \ingroup ModelBasedTracking
  \warning This class is only available if OpenCV is installed, and used.
  
  \brief Model based tracker using only KLT
  
  The tracker requires the knowledge of the 3D model that could be provided in a vrml
  or in a cao file. The cao format is described in loadCAOModel().
  It may also use an xml file used to tune the behavior of the tracker and an
  init file used to compute the pose at the very first image.

  The following code shows the simplest way to use the tracker. The \ref tutorial-tracking-mb is also a good starting point to use this class.
  
\code
#include <visp/vpMbKltTracker.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpException.h>
#include <visp/vpDisplayX.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbKltTracker tracker; // Create a model based tracker via KLT points.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker. 
  vpCameraParameters cam;
  
  // Acquire an image
  vpImageIo::read(I, "cube.pgm");
  
#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Klt Tracker");
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
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbKltTracker::loadConfigFile()
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
#include <visp/vpMbKltTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImageIo.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbKltTracker tracker; // Create a model based tracker via Klt Points.
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
  
#if defined VISP_HAVE_XML2
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode

  Finally it can be used not to track an object but just to display a model at a
  given pose:

\code
#include <visp/vpMbKltTracker.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbKltTracker tracker; // Create a model based tracker via Klt Points.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used to display the model. 
  vpCameraParameters cam;
  
  // Acquire an image
  vpImageIo::read(I, "cube.pgm");
  
#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Klt Tracker");
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
  
#if defined VISP_HAVE_XML2
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode
*/
class VISP_EXPORT vpMbKltTracker: virtual public vpMbTracker
{
protected:
  //! Temporary OpenCV image for fast conversion.
  IplImage* cur;
  //! Initial pose.
  vpHomogeneousMatrix c0Mo;
  //! Angle used to detect a face appearance
  double angleAppears;
  //! Angle used to detect a face disappearance
  double angleDisappears;
  //! If true, compute the interaction matrix at each iteration of the minimization. Otherwise, compute it only on the first iteration.
  bool compute_interaction;
  //! Flag to specify whether the init method is called the first or not (specific calls to realize in this case).
  bool firstInitialisation;
  //! Erosion of the mask
  unsigned int maskBorder;
  //! The gain of the virtual visual servoing stage. 
  double lambda;
  //! The maximum iteration of the virtual visual servoing stage. 
  unsigned int  maxIter;
  //! Threshold below which the weight associated to a point to consider this one as an outlier.
  double threshold_outlier;
  //! Percentage of good points, according to the initial number, that must have the tracker.
  double percentGood;
  //! Use Ogre3d for visibility tests
  bool useOgre;
  //! The estimated displacement of the pose between the current instant and the initial position.
  vpHomogeneousMatrix ctTc0;
  //! Points tracker.
  vpKltOpencv tracker;
  //! Set of faces describing the object. 
  vpMbHiddenFaces<vpMbtKltPolygon> faces;
  //! First track() called
  bool firstTrack;
  //! Distance for near clipping
  double distNearClip;
  //! Distance for near clipping
  double distFarClip;
  //! Flags specifying which clipping to used
  unsigned int clippingFlag;
  
public:
  
            vpMbKltTracker();
  virtual   ~vpMbKltTracker();
  
  virtual void            display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo,
                                  const vpCameraParameters &cam, const vpColor& col, const unsigned int thickness=1,
                                  const bool displayFullModel = false);
  virtual void            display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                  const vpColor& col, const unsigned int thickness=1, const bool displayFullModel = false);

protected:
  virtual void            init(const vpImage<unsigned char>& I);
  virtual void            reinit(const vpImage<unsigned char>& I);
  
public:
  virtual void            loadConfigFile(const std::string& configFile);
          void            loadConfigFile(const char* configFile);
          
          /*! Return the angle used to test polygons appearance. */
  virtual inline  double  getAngleAppear() const { return angleAppears; }   
  
          /*! Return the angle used to test polygons disappearance. */
  virtual inline  double  getAngleDisappear() const { return angleDisappears; } 
  
          /*!
            Get the clipping used.
            
            \sa vpMbtPolygonClipping
            
            \return Clipping flags.
          */          
  virtual inline  unsigned int getClipping() const { return clippingFlag; } 
    
          /*! Return a reference to the faces structure. */
  inline  vpMbHiddenFaces<vpMbtKltPolygon>& getFaces() { return faces;}
          
          /*!
            Get the far distance for clipping.
            
            \return Far clipping value.
          */
  virtual inline  double  getFarClippingDistance() const { return distFarClip; }

          /*!
            Get the current list of KLT points.
            
            \return the list of KLT points through vpKltOpencv.
          */
  inline  CvPoint2D32f*   getKltPoints() {return tracker.getFeatures();}
  
          std::vector<vpImagePoint> getKltImagePoints() const;
          
          std::map<int, vpImagePoint> getKltImagePointsWithId() const;
          
          /*!
            Get the klt tracker at the current state.
            
            \return klt tracker.
          */
  inline  vpKltOpencv     getKltOpencv() const { return tracker; }
          
          /*!
            Get the value of the gain used to compute the control law.
            
            \return the value for the gain.
          */
  virtual inline  double  getLambda() const {return lambda;}
  
          /*!
            Get the erosion of the mask used on the Model faces.

            \return The erosion.
          */
  inline  unsigned int    getMaskBorder() const { return maskBorder; }
  
          /*!
            Get the maximum iteration of the virtual visual servoing stage.
            
            \return the number of iteration
          */
  virtual inline unsigned int getMaxIter() const {return maxIter;}
  
          /*!
            Get the current number of klt points.
            
            \return the number of features
          */
  inline  int             getNbKltPoints() const {return tracker.getNbFeatures();}
       
          /*!
            Get the near distance for clipping.
            
            \return Near clipping value.
          */
  virtual inline double   getNearClippingDistance() const { return distNearClip; }
  
          /*!
            Get the threshold for the acceptation of a point.

            \return threshold_outlier : Threshold for the weight below which a point is rejected.
          */
  inline  double          getThresholdAcceptation() const { return threshold_outlier;}
  
          void            resetTracker();
          
          /*! 
            Set the angle used to test polygons appearance.
            If the angle between the normal of the polygon and the line going
            from the camera to the polygon center has a value lower than
            this parameter, the polygon is considered as appearing.
            The polygon will then be tracked.
            
            \param a : new angle in radian.
          */
  virtual inline  void    setAngleAppear(const double &a) { angleAppears = a; }   
  
          /*! 
            Set the angle used to test polygons disappearance.
            If the angle between the normal of the polygon and the line going
            from the camera to the polygon center has a value greater than
            this parameter, the polygon is considered as disappearing.
            The tracking of the polygon will then be stopped.

            \param a : new angle in radian.
          */
  virtual inline  void    setAngleDisappear(const double &a) { angleDisappears = a; } 
  
          void            setCameraParameters(const vpCameraParameters& cam);
          
  virtual void            setClipping(const unsigned int &flags);
          
  virtual void            setFarClippingDistance(const double &dist);
          
          void            setKltOpencv(const vpKltOpencv& t);
          
          /*!
            Set the value of the gain used to compute the control law.
            
            \param gain : the desired value for the gain.
          */
  virtual inline  void    setLambda(const double gain) {this->lambda = gain;}
  
          /*!
            Set the erosion of the mask used on the Model faces.

            \param  e : The desired erosion.
          */
  inline  void            setMaskBorder(const unsigned int &e){ maskBorder = e; }
  
          /*!
            Set the maximum iteration of the virtual visual servoing stage.
            
            \param max : the desired number of iteration
          */
  virtual inline  void    setMaxIter(const unsigned int max) {maxIter = max;}
  
  virtual void            setNearClippingDistance(const double &dist);
  
  virtual void            setOgreVisibilityTest(const bool &v);
  
  virtual void            setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo);
  
          /*!
            Set the threshold for the acceptation of a point.

            \param th : Threshold for the weight below which a point is rejected.
          */
  inline  void            setThresholdAcceptation(const double th) {threshold_outlier = th;}
  
  virtual void            testTracking();
  virtual void            track(const vpImage<unsigned char>& I);
  
protected:
          void            computeVVS(const unsigned int &nbInfos, vpColVector &w);
          
  virtual void            initFaceFromCorners(const std::vector<vpPoint>& corners, const unsigned int indexFace = -1);
  virtual void            initCylinder(const vpPoint&, const vpPoint &, const double, const unsigned int ){};
  
          void            preTracking(const vpImage<unsigned char>& I, unsigned int &nbInfos, unsigned int &nbFaceUsed);
          bool            postTracking(const vpImage<unsigned char>& I, vpColVector &w);
};

#endif
#endif // VISP_HAVE_OPENCV
