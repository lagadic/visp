/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
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

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))

#include <visp3/mbt/vpMbTracker.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/mbt/vpMbtKltXmlParser.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpSubColVector.h>
#include <visp3/core/vpSubMatrix.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/mbt/vpMbtDistanceKltPoints.h>
#include <visp3/mbt/vpMbtDistanceCircle.h>
#include <visp3/mbt/vpMbtDistanceKltCylinder.h>

/*!
  \class vpMbKltTracker
  \ingroup group_mbt_trackers
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
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat cur;
#else
  IplImage *cur;
#endif
  //! Initial pose.
  vpHomogeneousMatrix c0Mo;
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
  //! The estimated displacement of the pose between the current instant and the initial position.
  vpHomogeneousMatrix ctTc0;
  //! Points tracker.
  vpKltOpencv tracker;
  //!
  std::list<vpMbtDistanceKltPoints*> kltPolygons;
  //!
  std::list<vpMbtDistanceKltCylinder*> kltCylinders;
  //! Vector of the circles used here only to display the full model.
  std::list<vpMbtDistanceCircle*> circles_disp;

public:
            vpMbKltTracker();
  virtual   ~vpMbKltTracker();
  
            void addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, const double r, const std::string &name="");
  virtual void            display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo,
                                  const vpCameraParameters &cam, const vpColor& col, const unsigned int thickness=1,
                                  const bool displayFullModel = false);
  virtual void            display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                  const vpColor& col, const unsigned int thickness=1, const bool displayFullModel = false);

protected:
  virtual void            init(const vpImage<unsigned char>& I);
  virtual void            reinit(const vpImage<unsigned char>& I);
  
public:
  /*! Return the address of the circle feature list. */
  std::list<vpMbtDistanceCircle*> &getFeaturesCircle() { return circles_disp; }
  /*! Return the address of the cylinder feature list. */
  std::list<vpMbtDistanceKltCylinder*> &getFeaturesKltCylinder() { return kltCylinders; }
  /*! Return the address of the Klt feature list. */
  std::list<vpMbtDistanceKltPoints*> &getFeaturesKlt() { return kltPolygons; }
  virtual void            loadConfigFile(const std::string& configFile);
          void            loadConfigFile(const char* configFile);
          
          /*!
            Get the current list of KLT points.
            
            \return the list of KLT points through vpKltOpencv.
          */
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
          inline  std::vector<cv::Point2f> getKltPoints() {return tracker.getFeatures();}
#else
          inline  CvPoint2D32f*   getKltPoints() {return tracker.getFeatures();}
#endif
  
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
            Get the threshold for the acceptation of a point.

            \return threshold_outlier : Threshold for the weight below which a point is rejected.
          */
  inline  double          getThresholdAcceptation() const { return threshold_outlier;}
  
  void reInitModel(const vpImage<unsigned char>& I, const std::string &cad_name, const vpHomogeneousMatrix& cMo_,
		  const bool verbose=false);
  void reInitModel(const vpImage<unsigned char>& I, const char* cad_name, const vpHomogeneousMatrix& cMo,
		  const bool verbose=false);
          void            resetTracker();
            
          void            setCameraParameters(const vpCameraParameters& cam);

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
  inline  void            setMaskBorder(const unsigned int &e)
                          {
                            maskBorder = e;
                            //if(useScanLine)
                            faces.getMbScanLineRenderer().setMaskBorder(maskBorder);
                          }
  
          /*!
            Set the maximum iteration of the virtual visual servoing stage.
            
            \param max : the desired number of iteration
          */
  virtual inline  void    setMaxIter(const unsigned int max) {maxIter = max;}

          /*!
            Use Ogre3D for visibility tests

            \warning This function has to be called before the initialization of the tracker.

            \param v : True to use it, False otherwise
          */
  virtual         void    setOgreVisibilityTest(const bool &v){
      vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
      faces.getOgreContext()->setWindowName("MBT Klt");
#endif
  }

          /*!
            Use Scanline algorithm for visibility tests

            \param v : True to use it, False otherwise
          */
  virtual void setScanLineVisibilityTest(const bool &v){
    vpMbTracker::setScanLineVisibilityTest(v);

    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it)
      (*it)->useScanLine = v;
  }
  
  virtual void            setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo);

  /*!
    Set if the projection error criteria has to be computed.

    \param flag : True if the projection error criteria has to be computed, false otherwise
  */
  virtual void            setProjectionErrorComputation(const bool &flag) {
                if(flag)
                  vpCTRACE << "This option is not yet implemented in vpMbKltTracker, projection error computation set to false." << std::endl ; }
  
          /*!
            Set the threshold for the acceptation of a point.

            \param th : Threshold for the weight below which a point is rejected.
          */
  inline  void            setThresholdAcceptation(const double th) {threshold_outlier = th;}  

          void            setUseKltTracking(const std::string &name, const bool &useKltTracking);
  
  virtual void            testTracking();
  virtual void            track(const vpImage<unsigned char>& I);
  
protected:
          void            computeVVS(const unsigned int &nbInfos, vpColVector &w);
          
          virtual void            initFaceFromCorners(vpMbtPolygon &polygon);
          virtual void            initFaceFromLines(vpMbtPolygon &polygon);
          virtual void    initCircle(const vpPoint&, const vpPoint &, const vpPoint &, const double, const int,
              const std::string &name="");
          virtual void    initCylinder(const vpPoint&, const vpPoint &, const double, const int,
              const std::string &name="");

          void            preTracking(const vpImage<unsigned char>& I, unsigned int &nbInfos, unsigned int &nbFaceUsed);
          bool            postTracking(const vpImage<unsigned char>& I, vpColVector &w);
};

#endif
#endif // VISP_HAVE_OPENCV
