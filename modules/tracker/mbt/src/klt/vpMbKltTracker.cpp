/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#include <visp3/core/vpImageConvert.h>
#include <visp3/mbt/vpMbKltTracker.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/core/vpTrackingException.h>

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))

#if defined(__APPLE__) && defined(__MACH__) // Apple OSX and iOS (Darwin)
#  include <TargetConditionals.h> // To detect OSX or IOS using TARGET_OS_IPHONE or TARGET_OS_IOS macro
#endif

vpMbKltTracker::vpMbKltTracker()
  :
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cur(),
#else
    cur(NULL),
#endif
    c0Mo(), compute_interaction(true),
    firstInitialisation(true), maskBorder(5), lambda(0.8), maxIter(200), threshold_outlier(0.5),
    percentGood(0.6), ctTc0(), tracker(), kltPolygons(), kltCylinders(), circles_disp()
{  
  tracker.setTrackerId(1);
  tracker.setUseHarris(1);
  tracker.setMaxFeatures(10000);
  tracker.setWindowSize(5);
  tracker.setQuality(0.01);
  tracker.setMinDistance(5);
  tracker.setHarrisFreeParameter(0.01);
  tracker.setBlockSize(3);
  tracker.setPyramidLevels(3);
  
  angleAppears = vpMath::rad(65);
  angleDisappears = vpMath::rad(75);

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Klt");
#endif
}

/*!
  Basic destructor.

*/
vpMbKltTracker::~vpMbKltTracker()
{
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if(cur != NULL){
    cvReleaseImage(&cur);
    cur = NULL;
  }
#endif

  // delete the Klt Polygon features
  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if (kltpoly!=NULL){
      delete kltpoly ;
    }
    kltpoly = NULL ;
  }
  kltPolygons.clear();

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    if (kltPolyCylinder!=NULL){
      delete kltPolyCylinder ;
    }
    kltPolyCylinder = NULL ;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  vpMbtDistanceCircle *ci;
  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles_disp.begin(); it!=circles_disp.end(); ++it){
    ci = *it;
    if (ci!=NULL){
      delete ci ;
    }
    ci = NULL ;
  }

  circles_disp.clear();
}

void 
vpMbKltTracker::init(const vpImage<unsigned char>& I)
{
  if(!modelInitialised){
    throw vpException(vpException::fatalError, "model not initialized");
  }
  
 bool reInitialisation = false;
  if(!useOgre)
    faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
  else{
#ifdef VISP_HAVE_OGRE   
    if(!faces.isOgreInitialised()){
      faces.setBackgroundSizeOgre(I.getHeight(), I.getWidth());
      faces.setOgreShowConfigDialog(ogreShowConfigDialog);
      faces.initOgre(cam);
	  // Turn off Ogre config dialog display for the next call to this function
	  // since settings are saved in the ogre.cfg file and used during the next
	  // call 
	  ogreShowConfigDialog = false;
    }
    
    faces.setVisibleOgre(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
    
#else
    faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
  }
  reinit(I);
}

void 
vpMbKltTracker::reinit(const vpImage<unsigned char>& I)
{
  c0Mo = cMo;
  ctTc0.eye();

  vpImageConvert::convert(I, cur);

  cam.computeFov(I.getWidth(), I.getHeight());

  if(useScanLine){
    faces.computeClippedPolygons(cMo,cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
  }
  
  // mask
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat mask((int)I.getRows(), (int)I.getCols(), CV_8UC1, cv::Scalar(0));
#else
  IplImage* mask = cvCreateImage(cvSize((int)I.getWidth(), (int)I.getHeight()), IPL_DEPTH_8U, 1);
  cvZero(mask);
#endif

  vpMbtDistanceKltPoints *kltpoly;
  vpMbtDistanceKltCylinder *kltPolyCylinder;
  if(useScanLine){
    vpImageConvert::convert(faces.getMbScanLineRenderer().getMask(), mask);
  }
  else{
    unsigned char val = 255/* - i*15*/;
    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
      kltpoly = *it;
      if(kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2){
        //need to changeFrame when reinit() is called by postTracking
        //other solution is
        kltpoly->polygon->changeFrame(cMo);
        kltpoly->polygon->computePolygonClipped(cam); // Might not be necessary when scanline is activated
        kltpoly->updateMask(mask, val, maskBorder);
      }
    }

    for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
      kltPolyCylinder = *it;

      if(kltPolyCylinder->isTracked())
      {
        for(unsigned int k = 0 ; k < kltPolyCylinder->listIndicesCylinderBBox.size() ; k++)
        {
          unsigned int indCylBBox = (unsigned int)kltPolyCylinder->listIndicesCylinderBBox[k];
          if(faces[indCylBBox]->isVisible() && faces[indCylBBox]->getNbPoint() > 2u){
            faces[indCylBBox]->computePolygonClipped(cam); // Might not be necessary when scanline is activated
          }
        }

        kltPolyCylinder->updateMask(mask, val, maskBorder);
      }
    }
  }
  
  tracker.initTracking(cur, mask);
//  tracker.track(cur); // AY: Not sure to be usefull but makes sure that the points are valid for tracking and avoid too fast reinitialisations.
//  vpCTRACE << "init klt. detected " << tracker.getNbFeatures() << " points" << std::endl;

  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2){
      kltpoly->init(tracker);
    }
  }

  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;

    if(kltPolyCylinder->isTracked())
      kltPolyCylinder->init(tracker, cMo);
  }

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  cvReleaseImage(&mask);
#endif
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void            
vpMbKltTracker::resetTracker()
{
  cMo.eye();
  
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if(cur != NULL){
    cvReleaseImage(&cur);
    cur = NULL;
  }
#endif

  // delete the Klt Polygon features
  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if (kltpoly!=NULL){
      delete kltpoly ;
    }
    kltpoly = NULL ;
  }
  kltPolygons.clear();

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    if (kltPolyCylinder!=NULL){
      delete kltPolyCylinder ;
    }
    kltPolyCylinder = NULL ;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  vpMbtDistanceCircle *ci;
  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles_disp.begin(); it!=circles_disp.end(); ++it){
    ci = *it;
    if (ci!=NULL){
      delete ci ;
    }
    ci = NULL ;
  }

  circles_disp.clear();

  compute_interaction = true;
  firstInitialisation = true;
  computeCovariance = false;

  tracker.setTrackerId(1);
  tracker.setUseHarris(1);
  
  tracker.setMaxFeatures(10000);
  tracker.setWindowSize(5);
  tracker.setQuality(0.01);
  tracker.setMinDistance(5);
  tracker.setHarrisFreeParameter(0.01);
  tracker.setBlockSize(3);
  tracker.setPyramidLevels(3);
  
  angleAppears = vpMath::rad(65);
  angleDisappears = vpMath::rad(75);
  
  clippingFlag = vpPolygon3D::NO_CLIPPING;
  
  maskBorder = 5;
  threshold_outlier = 0.5;
  percentGood = 0.7;
  
  lambda = 0.8;
  maxIter = 200;

  faces.reset();

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  useScanLine = false;
  
#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif
}

/*!
  Get the current list of KLT points.
  
  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f. This function convert and copy the openCV KLT points into vpImagePoints.
  
  \return the list of KLT points through vpKltOpencv.
*/
std::vector<vpImagePoint> 
vpMbKltTracker::getKltImagePoints() const
{
  std::vector<vpImagePoint> kltPoints;
  for (unsigned int i = 0; i < static_cast<unsigned int>(tracker.getNbFeatures()); i ++){
    long id;
    float x_tmp, y_tmp;
    tracker.getFeature((int)i, id, x_tmp, y_tmp);
    kltPoints.push_back(vpImagePoint(y_tmp, x_tmp));
  }
  
  return kltPoints;
}

/*!
  Get the current list of KLT points and their id.
  
  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f. This function convert and copy the openCV KLT points into vpImagePoints.
  
  \return the list of KLT points and their id through vpKltOpencv.
*/
std::map<int, vpImagePoint> 
vpMbKltTracker::getKltImagePointsWithId() const
{
  std::map<int, vpImagePoint> kltPoints;
  for (unsigned int i = 0; i < static_cast<unsigned int>(tracker.getNbFeatures()); i ++){
    long id;
    float x_tmp, y_tmp;
    tracker.getFeature((int)i, id, x_tmp, y_tmp);
#if TARGET_OS_IPHONE
    kltPoints[(int)id] = vpImagePoint(y_tmp, x_tmp);
#else
    kltPoints[id] = vpImagePoint(y_tmp, x_tmp);
#endif
  }
  
  return kltPoints;
}

/*!
  Set the new value of the klt tracker.

  \param t : Klt tracker containing the new values.
*/
void            
vpMbKltTracker::setKltOpencv(const vpKltOpencv& t){
  tracker.setMaxFeatures(t.getMaxFeatures());
  tracker.setWindowSize(t.getWindowSize());
  tracker.setQuality(t.getQuality());
  tracker.setMinDistance(t.getMinDistance());
  tracker.setHarrisFreeParameter(t.getHarrisFreeParameter());
  tracker.setBlockSize(t.getBlockSize());
  tracker.setPyramidLevels(t.getPyramidLevels());
}

/*!
  Set the camera parameters.

  \param camera : the new camera parameters.
*/
void
vpMbKltTracker::setCameraParameters(const vpCameraParameters& camera)
{
//  for (unsigned int i = 0; i < faces.size(); i += 1){
//    faces[i]->setCameraParameters(camera);
//  }

  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    kltpoly->setCameraParameters(camera);
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    kltPolyCylinder->setCameraParameters(camera);
  }

  this->cam = camera;
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track() function.
  This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders.

  \param I : image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void
vpMbKltTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo)
{
  if((int)(kltCylinders.size()) != 0)
  {
    std::cout << "WARNING: Cannot set pose when model contains cylinder(s). This feature is not implemented yet." << std::endl;
    std::cout << "Tracker will be reinitialized with the given pose." << std::endl;
    cMo = cdMo;
    init(I);
  }
  else
  {
    vpMbtDistanceKltPoints *kltpoly;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    std::vector<cv::Point2f> init_pts;
    std::vector<long> init_ids;
    std::vector<cv::Point2f> guess_pts;
#else
    unsigned int nbp = 0;
    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it) {
      kltpoly = *it;
      if(kltpoly->polygon->isVisible() && kltpoly->polygon->getNbPoint() > 2)
        nbp += (*it)->getCurrentNumberPoints();
    }

    CvPoint2D32f* init_pts = NULL;
    init_pts = (CvPoint2D32f*)cvAlloc(tracker.getMaxFeatures()*sizeof(init_pts[0]));
    long *init_ids = (long*)cvAlloc((unsigned int)tracker.getMaxFeatures()*sizeof(long));
    unsigned int iter_pts = 0;

    CvPoint2D32f* guess_pts = NULL;
    guess_pts = (CvPoint2D32f*)cvAlloc(tracker.getMaxFeatures()*sizeof(guess_pts[0]));
#endif

    vpHomogeneousMatrix cdMc = cdMo * cMo.inverse();
    vpHomogeneousMatrix cMcd = cdMc.inverse();

    vpRotationMatrix cdRc;
    vpTranslationVector cdtc;

    cdMc.extract(cdRc);
    cdMc.extract(cdtc);

    unsigned int nbCur = 0;

    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it) {
      kltpoly = *it;

      if(kltpoly->polygon->isVisible() && kltpoly->polygon->getNbPoint() > 2) {
        kltpoly->polygon->changeFrame(cdMo);

        //Get the normal to the face at the current state cMo
        vpPlane plan(kltpoly->polygon->p[0], kltpoly->polygon->p[1], kltpoly->polygon->p[2]);
        plan.changeFrame(cMcd);

        vpColVector Nc = plan.getNormal();
        Nc.normalize();

        double invDc = 1.0 / plan.getD();

        //Create the homography
        vpMatrix cdHc;
        vpGEMM(cdtc, Nc, -invDc, cdRc, 1.0, cdHc, VP_GEMM_B_T);
        cdHc /= cdHc[2][2];

        //Create the 2D homography
        vpMatrix cdGc = cam.get_K() * cdHc * cam.get_K_inverse();

        //Points displacement
        std::map<int, vpImagePoint>::const_iterator iter = kltpoly->getCurrentPoints().begin();
        nbCur+= (unsigned int)kltpoly->getCurrentPoints().size();
        for( ; iter != kltpoly->getCurrentPoints().end(); iter++){
          vpColVector cdp(3);
          cdp[0] = iter->second.get_j(); cdp[1] = iter->second.get_i(); cdp[2] = 1.0;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
          cv::Point2f p((float)cdp[0], (float)cdp[1]);
          init_pts.push_back(p);
#  if TARGET_OS_IPHONE
          init_ids.push_back((size_t)(kltpoly->getCurrentPointsInd())[(int)iter->first]);
#  else
          init_ids.push_back((size_t)(kltpoly->getCurrentPointsInd())[(size_t)iter->first]);
#  endif
#else
          init_pts[iter_pts].x = (float)cdp[0];
          init_pts[iter_pts].y = (float)cdp[1];
          init_ids[iter_pts] = (kltpoly->getCurrentPointsInd())[(size_t)iter->first];
#endif

          double p_mu_t_2 = cdp[0] * cdGc[2][0] + cdp[1] * cdGc[2][1] + cdGc[2][2];

          if( fabs(p_mu_t_2) < std::numeric_limits<double>::epsilon()){
            cdp[0] = 0.0;
            cdp[1] = 0.0;
            throw vpException(vpException::divideByZeroError, "the depth of the point is calculated to zero");
          }

          cdp[0] = (cdp[0] * cdGc[0][0] + cdp[1] * cdGc[0][1] + cdGc[0][2]) / p_mu_t_2;
          cdp[1] = (cdp[0] * cdGc[1][0] + cdp[1] * cdGc[1][1] + cdGc[1][2]) / p_mu_t_2;

          //Set value to the KLT tracker
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
          cv::Point2f p_guess((float)cdp[0], (float)cdp[1]);
          guess_pts.push_back(p_guess);
#else
          guess_pts[iter_pts].x = (float)cdp[0];
          guess_pts[iter_pts++].y = (float)cdp[1];
#endif
        }
      }
    }

    vpImageConvert::convert(I, cur);

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    tracker.setInitialGuess(init_pts, guess_pts, init_ids);
#else
    tracker.setInitialGuess(&init_pts, &guess_pts, init_ids, iter_pts);

    if(init_pts) cvFree(&init_pts);
    init_pts = NULL;

    if(guess_pts) cvFree(&guess_pts);
    guess_pts = NULL;

    if(init_ids)cvFree(&init_ids);
    init_ids = NULL;
#endif

    bool reInitialisation = false;
    if(!useOgre)
      faces.setVisible(I, cam, cdMo, angleAppears, angleDisappears, reInitialisation);
    else{
#ifdef VISP_HAVE_OGRE
      faces.setVisibleOgre(I, cam, cdMo, angleAppears, angleDisappears, reInitialisation);
#else
      faces.setVisible(I, cam, cdMo, angleAppears, angleDisappears, reInitialisation);
#endif
    }

    cam.computeFov(I.getWidth(), I.getHeight());

    if(useScanLine){
      faces.computeClippedPolygons(cdMo,cam);
      faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
    }

    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
      kltpoly = *it;
      if(kltpoly->polygon->isVisible() && kltpoly->polygon->getNbPoint() > 2){
        kltpoly->polygon->computePolygonClipped(cam);
        kltpoly->init(tracker);
      }
    }

    cMo = cdMo;
    c0Mo = cMo;
    ctTc0.eye();
  }
}

/*!
  Initialise a new face from the coordinates given in parameter.

  \param polygon : The polygon describing the set of lines that has to be tracked.
*/
void
vpMbKltTracker::initFaceFromCorners(vpMbtPolygon &polygon)
{
    vpMbtDistanceKltPoints *kltPoly = new vpMbtDistanceKltPoints();
    kltPoly->setCameraParameters(cam) ;
    kltPoly->polygon = &polygon;
    kltPoly->hiddenface = &faces ;
    kltPoly->useScanLine = useScanLine;
    kltPolygons.push_back(kltPoly);
}
/*!
  Initialise a new face from the coordinates given in parameter.

  \param polygon : The polygon describing the set of lines that has to be tracked.
*/
void
vpMbKltTracker::initFaceFromLines(vpMbtPolygon &polygon)
{
    vpMbtDistanceKltPoints *kltPoly = new vpMbtDistanceKltPoints();
    kltPoly->setCameraParameters(cam) ;
    kltPoly->polygon = &polygon;
    kltPoly->hiddenface = &faces ;
    kltPoly->useScanLine = useScanLine;
    kltPolygons.push_back(kltPoly);
}

/*!
  Achieve the tracking of the KLT features and associate the features to the faces.

  \param I : The input image.
  \param nbInfos : Size of the features.
  \param nbFaceUsed : Number of face used for the tracking.
*/
void
vpMbKltTracker::preTracking(const vpImage<unsigned char>& I, unsigned int &nbInfos, unsigned int &nbFaceUsed)
{
  vpImageConvert::convert(I, cur);
  tracker.track(cur);
  
  nbInfos = 0;
  nbFaceUsed = 0;
  vpMbtDistanceKltPoints *kltpoly;
//  for (unsigned int i = 0; i < faces.size(); i += 1){
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2){
      kltpoly->computeNbDetectedCurrent(tracker);
//       faces[i]->ransac();
      if(kltpoly->hasEnoughPoints()){
        nbInfos += kltpoly->getCurrentNumberPoints();
        nbFaceUsed++;
      }
    }
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;

    if(kltPolyCylinder->isTracked())
    {
      kltPolyCylinder->computeNbDetectedCurrent(tracker);
      if(kltPolyCylinder->hasEnoughPoints()){
        nbInfos += kltPolyCylinder->getCurrentNumberPoints();
        nbFaceUsed++;
      }
    }
  }
}

/*!
  Realize the post tracking operations. Mostly visibility tests
*/
bool
vpMbKltTracker::postTracking(const vpImage<unsigned char>& I, vpColVector &w)
{
  // # For a better Post Tracking, tracker should reinitialize if so faces don't have enough points but are visible.
  // # Here we are not doing it for more speed performance.
  bool reInitialisation = false;
  
  unsigned int initialNumber = 0;
  unsigned int currentNumber = 0;
  unsigned int shift = 0;
  vpMbtDistanceKltPoints *kltpoly;
//  for (unsigned int i = 0; i < faces.size(); i += 1){
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2){
      initialNumber += kltpoly->getInitialNumberPoint();
      if(kltpoly->hasEnoughPoints()){
        vpSubColVector sub_w(w, shift, 2*kltpoly->getCurrentNumberPoints());
        shift += 2*kltpoly->getCurrentNumberPoints();
        kltpoly->removeOutliers(sub_w, threshold_outlier);
        
        currentNumber += kltpoly->getCurrentNumberPoints();
      }
//       else{
//         reInitialisation = true;
//         break;
//       }
    }
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;

    if(kltPolyCylinder->isTracked())
    {
      initialNumber += kltPolyCylinder->getInitialNumberPoint();
      if(kltPolyCylinder->hasEnoughPoints()){
        vpSubColVector sub_w(w, shift, 2*kltPolyCylinder->getCurrentNumberPoints());
        shift += 2*kltPolyCylinder->getCurrentNumberPoints();
        kltPolyCylinder->removeOutliers(sub_w, threshold_outlier);

        currentNumber += kltPolyCylinder->getCurrentNumberPoints();
      }
    }
  }
  
//   if(!reInitialisation){
    double value = percentGood * (double)initialNumber;
    if((double)currentNumber < value){
//     std::cout << "Too many point disappear : " << initialNumber << "/" << currentNumber << std::endl;
      reInitialisation = true;
    }
    else{
      if(!useOgre)
        faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
      else{
#ifdef VISP_HAVE_OGRE    
        faces.setVisibleOgre(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#else
        faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
      }
    }
//   }
  
  if(reInitialisation)
    return true;
  
  return false;
}

/*!
  Realize the VVS loop for the tracking

  \param nbInfos : Size of the features
  \param w : weight of the features after M-Estimation.
*/
void
vpMbKltTracker::computeVVS(const unsigned int &nbInfos, vpColVector &w)
{
  vpMatrix L;     // interaction matrix
  vpColVector R;  // residu
  vpMatrix L_true;     // interaction matrix
  vpMatrix LVJ_true;
  //vpColVector R_true;  // residu
  vpColVector v;  // "speed" for VVS
  vpHomography H;
  vpColVector w_true;
  vpRobust robust(2*nbInfos);

  vpMatrix LTL;
  vpColVector LTR;
  vpHomogeneousMatrix cMoPrev;
  vpHomogeneousMatrix ctTc0_Prev;
  vpColVector error_prev(2*nbInfos);
  double mu = 0.01;
  
  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  R.resize(2*nbInfos);
  L.resize(2*nbInfos, 6, 0);

  while( ((int)((normRes - normRes_1)*1e8) != 0 )  && (iter<maxIter) ){
    
    unsigned int shift = 0;

    computeVVSInteractionMatrixAndResidu(shift, R, L, H, kltPolygons, kltCylinders, ctTc0);

    bool reStartFromLastIncrement = false;

    computeVVSCheckLevenbergMarquardtKlt(iter, nbInfos, cMoPrev, error_prev, ctTc0_Prev, mu, reStartFromLastIncrement);

    if(!reStartFromLastIncrement){
      computeVVSWeights(iter, nbInfos, R, w_true, w, robust);

      computeVVSPoseEstimation(iter, L, w, L_true, LVJ_true, normRes, normRes_1, w_true, R, LTL, LTR,
          error_prev, v, mu, cMoPrev, ctTc0_Prev);
    } // endif(!reStartFromLastIncrement)
    
    iter++;
  }
  
  if(computeCovariance){
    computeVVSCovariance(w_true, cMoPrev, L_true, LVJ_true);
  }
}

void
vpMbKltTracker::computeVVSCheckLevenbergMarquardtKlt(const unsigned int iter, const unsigned int nbInfos,
    const vpHomogeneousMatrix &cMoPrev, const vpColVector &error_prev, const vpHomogeneousMatrix &ctTc0_Prev,
    double &mu, bool &reStartFromLastIncrement) {
  if(iter != 0 && m_optimizationMethod == vpMbTracker::LEVENBERG_MARQUARDT_OPT){
    if(m_error.sumSquare()/(double)(2*nbInfos) > error_prev.sumSquare()/(double)(2*nbInfos)){
      mu *= 10.0;

      if(mu > 1.0)
        throw vpTrackingException(vpTrackingException::fatalError, "Optimization diverged");

      cMo = cMoPrev;
      m_error = error_prev;
      ctTc0 = ctTc0_Prev;
      reStartFromLastIncrement = true;
    }
  }
}

void
vpMbKltTracker::computeVVSCovariance(const vpColVector &w_true, const vpHomogeneousMatrix &cMoPrev,
    const vpMatrix &L_true, const vpMatrix &LVJ_true) {
  if(computeCovariance){
    vpMatrix D;
    D.diag(w_true);

    // Note that here the covariance is computed on cMoPrev for time computation efficiency
    if(isoJoIdentity){
        covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev,m_error,L_true,D);
    }
    else{
        covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev,m_error,LVJ_true,D);
    }
  }
}

void
vpMbKltTracker::computeVVSInteractionMatrixAndResidu(unsigned int shift, vpColVector &R, vpMatrix &L, vpHomography &H,
    std::list<vpMbtDistanceKltPoints*> &kltPolygons_, std::list<vpMbtDistanceKltCylinder*> &kltCylinders_,
    const vpHomogeneousMatrix &ctTc0_) {
  vpMbtDistanceKltPoints *kltpoly;
//  for (unsigned int i = 0; i < faces.size(); i += 1){
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it = kltPolygons_.begin(); it != kltPolygons_.end(); ++it){
    kltpoly = *it;
    if(kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2 &&
       kltpoly->hasEnoughPoints()){
      vpSubColVector subR(R, shift, 2*kltpoly->getCurrentNumberPoints());
      vpSubMatrix subL(L, shift, 0, 2*kltpoly->getCurrentNumberPoints(), 6);
      try{
        kltpoly->computeHomography(ctTc0_, H);
        kltpoly->computeInteractionMatrixAndResidu(subR, subL);
      }catch(...){
        throw vpTrackingException(vpTrackingException::fatalError, "Cannot compute interaction matrix");
      }

      shift += 2*kltpoly->getCurrentNumberPoints();
    }
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it = kltCylinders_.begin(); it != kltCylinders_.end(); ++it){
    kltPolyCylinder = *it;

    if(kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
    {
      vpSubColVector subR(R, shift, 2*kltPolyCylinder->getCurrentNumberPoints());
      vpSubMatrix subL(L, shift, 0, 2*kltPolyCylinder->getCurrentNumberPoints(), 6);
      try{
        kltPolyCylinder->computeInteractionMatrixAndResidu(ctTc0_,subR, subL);
      }catch(...){
        throw vpTrackingException(vpTrackingException::fatalError, "Cannot compute interaction matrix");
      }

      shift += 2*kltPolyCylinder->getCurrentNumberPoints();
    }
  }
}

void
vpMbKltTracker::computeVVSPoseEstimation(const unsigned int iter, vpMatrix &L,
    const vpColVector &w, vpMatrix &L_true, vpMatrix &LVJ_true, double &normRes, double &normRes_1, vpColVector &w_true,
    vpColVector &R, vpMatrix &LTL, vpColVector &LTR, vpColVector &error_prev, vpColVector &v, double &mu,
    vpHomogeneousMatrix &cMoPrev, vpHomogeneousMatrix &ctTc0_Prev) {
  m_error = R;
  if(computeCovariance){
    L_true = L;
    if(!isoJoIdentity){
       vpVelocityTwistMatrix cVo;
       cVo.buildFrom(cMo);
       LVJ_true = (L*cVo*oJo);
    }
  }

  normRes_1 = normRes;
  normRes = 0;
  for (unsigned int i = 0; i < static_cast<unsigned int>(R.getRows()); i += 1){
    w_true[i] = w[i];
    R[i] = R[i] * w[i];
    normRes += R[i];
  }

  if((iter == 0) || compute_interaction){
    for(unsigned int i=0; i<static_cast<unsigned int>(R.getRows()); i++){
      for(unsigned int j=0; j<6; j++){
        L[i][j] *= w[i];
      }
    }
  }

  if(isoJoIdentity){
      LTL = L.AtA();
      computeJTR(L, R, LTR);

      switch(m_optimizationMethod){
      case vpMbTracker::LEVENBERG_MARQUARDT_OPT:
      {
        vpMatrix LMA(LTL.getRows(), LTL.getCols());
        LMA.eye();
        vpMatrix LTLmuI = LTL + (LMA*mu);
        v = -lambda*LTLmuI.pseudoInverse(LTLmuI.getRows()*std::numeric_limits<double>::epsilon())*LTR;

        if(iter != 0)
          mu /= 10.0;

        error_prev = m_error;
        break;
      }
      case vpMbTracker::GAUSS_NEWTON_OPT:
      default:
        v = -lambda * LTL.pseudoInverse(LTL.getRows()*std::numeric_limits<double>::epsilon()) * LTR;
      }
  }
  else{
      vpVelocityTwistMatrix cVo;
      cVo.buildFrom(cMo);
      vpMatrix LVJ = (L*cVo*oJo);
      vpMatrix LVJTLVJ = (LVJ).AtA();
      vpColVector LVJTR;
      computeJTR(LVJ, R, LVJTR);

      switch(m_optimizationMethod){
      case vpMbTracker::LEVENBERG_MARQUARDT_OPT:
      {
        vpMatrix LMA(LVJTLVJ.getRows(), LVJTLVJ.getCols());
        LMA.eye();
        vpMatrix LTLmuI = LVJTLVJ + (LMA*mu);
        v = -lambda*LTLmuI.pseudoInverse(LTLmuI.getRows()*std::numeric_limits<double>::epsilon())*LVJTR;
        v = cVo * v;

        if(iter != 0)
          mu /= 10.0;

        error_prev = m_error;
        break;
      }
      case vpMbTracker::GAUSS_NEWTON_OPT:
      default:
      {
        v = -lambda*LVJTLVJ.pseudoInverse(LVJTLVJ.getRows()*std::numeric_limits<double>::epsilon())*LVJTR;
        v = cVo * v;
        break;
      }
      }
  }

  cMoPrev = cMo;
  ctTc0_Prev = ctTc0;
  ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
  cMo = ctTc0 * c0Mo;
}

void
vpMbKltTracker::computeVVSWeights(const unsigned int iter, const unsigned int nbInfos, const vpColVector &R,
    vpColVector &w_true, vpColVector &w, vpRobust &robust) {
  if(iter == 0){
    w_true.resize(2*nbInfos);
    w.resize(2*nbInfos);
    w = 1;
    w_true = 1;
  }
  robust.setIteration(iter);
  robust.setThreshold(2/cam.get_px());
  robust.MEstimator( vpRobust::TUKEY, R, w);
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I : the input image
*/
void
vpMbKltTracker::track(const vpImage<unsigned char>& I)
{   
  unsigned int nbInfos = 0;
  unsigned int nbFaceUsed = 0;

  try{
    preTracking(I, nbInfos, nbFaceUsed);
  }
  catch(vpException &e){
    throw e;
  }
  
  if(nbInfos < 4 || nbFaceUsed == 0){
    vpERROR_TRACE("\n\t\t Error-> not enough data") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  }

  //vpColVector w;
  computeVVS(nbInfos, m_w);

  if(postTracking(I, m_w))
    reinit(I);
}

/*!
  Load the xml configuration file. An example of such a file is provided in loadConfigFile(const char*) documentation.
  From the configuration file initialize the parameters corresponding to the objects: KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param configFile : full name of the xml file.

  \sa loadConfigFile(const char*), vpXmlParser::cleanup()
*/
void 
vpMbKltTracker::loadConfigFile(const std::string& configFile)
{
  vpMbKltTracker::loadConfigFile(configFile.c_str());
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the objects: KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data).

  \param configFile : full name of the xml file.

  The XML configuration file has the following form:
  \code
<?xml version="1.0"?>
<conf>
  <camera>
    <width>640</width>
    <height>480</height>
    <u0>320</u0>
    <v0>240</v0>
    <px>686.24</px>
    <py>686.24</py>
  </camera>
  <face>
    <angle_appear>65</angle_appear>
    <angle_disappear>85</angle_disappear>
    <near_clipping>0.01</near_clipping>
    <far_clipping>0.90</far_clipping>
    <fov_clipping>1</fov_clipping>
  </face>
  <klt>
    <mask_border>10</mask_border>
    <max_features>10000</max_features>
    <window_size>5</window_size>
    <quality>0.02</quality>
    <min_distance>10</min_distance>
    <harris>0.02</harris>
    <size_block>3</size_block>
    <pyramid_lvl>3</pyramid_lvl>
  </klt>
</conf>
  \endcode

  \sa loadConfigFile(const std::string&), vpXmlParser::cleanup()
*/
void
vpMbKltTracker::loadConfigFile(const char* configFile)
{
#ifdef VISP_HAVE_XML2
  vpMbtKltXmlParser xmlp;
  
  xmlp.setMaxFeatures(10000);
  xmlp.setWindowSize(5);
  xmlp.setQuality(0.01);
  xmlp.setMinDistance(5);
  xmlp.setHarrisParam(0.01);
  xmlp.setBlockSize(3);
  xmlp.setPyramidLevels(3);
  xmlp.setMaskBorder(maskBorder);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));
  
  try{
    std::cout << " *********** Parsing XML for MBT KLT Tracker ************ " << std::endl;
    xmlp.parse(configFile);
  }
  catch(...){
    vpERROR_TRACE("Can't open XML file \"%s\"\n ", configFile);
    throw vpException(vpException::ioError, "problem to parse configuration file.");
  }

  vpCameraParameters camera;
  xmlp.getCameraParameters(camera);
  setCameraParameters(camera);
  
  tracker.setMaxFeatures((int)xmlp.getMaxFeatures());
  tracker.setWindowSize((int)xmlp.getWindowSize());
  tracker.setQuality(xmlp.getQuality());
  tracker.setMinDistance(xmlp.getMinDistance());
  tracker.setHarrisFreeParameter(xmlp.getHarrisParam());
  tracker.setBlockSize((int)xmlp.getBlockSize());
  tracker.setPyramidLevels((int)xmlp.getPyramidLevels());
  maskBorder = xmlp.getMaskBorder();
  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  //if(useScanLine)
  faces.getMbScanLineRenderer().setMaskBorder(maskBorder);
  
  if(xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());
  
  if(xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());
  
  if(xmlp.getFovClipping())
    setClipping(clippingFlag = clippingFlag | vpPolygon3D::FOV_CLIPPING);

  useLodGeneral = xmlp.getLodState();
  minLineLengthThresholdGeneral = xmlp.getMinLineLengthThreshold();
  minPolygonAreaThresholdGeneral = xmlp.getMinPolygonAreaThreshold();

  applyLodSettingInConfig = false;
  if(this->getNbPolygon() > 0) {
    applyLodSettingInConfig = true;
    setLod(useLodGeneral);
    setMinLineLengthThresh(minLineLengthThresholdGeneral);
    setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);
  }

#else
  vpTRACE("You need the libXML2 to read the config file %s", configFile);
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : Boolean to say if all the model has to be displayed, even the faces that are visible.
*/
void
vpMbKltTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters & camera,
                        const vpColor& col, const unsigned int thickness, const bool displayFullModel)
{
  vpCameraParameters c = camera;

  if(clippingFlag > 3) // Contains at least one FOV constraint
    c.computeFov(I.getWidth(), I.getHeight());

  vpMbtDistanceKltPoints *kltpoly;
  vpMbtDistanceKltCylinder *kltPolyCylinder;

  // Previous version 12/08/2015
//  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
//    kltpoly = *it;
//    kltpoly->polygon->changeFrame(cMo_);
//    kltpoly->polygon->computePolygonClipped(c);
//  }
  faces.computeClippedPolygons(cMo_,c);

  if(useScanLine && !displayFullModel)
    faces.computeScanLineRender(cam,I.getWidth(), I.getHeight());

  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;

    kltpoly->display(I,cMo_,camera,col,thickness,displayFullModel);

    if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->polygon->isVisible() && kltpoly->isTracked()) {
      kltpoly->displayPrimitive(I);
//         faces[i]->displayNormal(I);
    }
  }

  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;

    kltPolyCylinder->display(I,cMo_,camera,col,thickness,displayFullModel);

    if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
      kltPolyCylinder->displayPrimitive(I);
  }

  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles_disp.begin(); it!=circles_disp.end(); ++it){
    (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
  }

#ifdef VISP_HAVE_OGRE
  if(useOgre)
    faces.displayOgre(cMo_);
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The color image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : Boolean to say if all the model has to be displayed, even the faces that are not visible.
*/
void
vpMbKltTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters & camera,
                        const vpColor& col , const unsigned int thickness, const bool displayFullModel)
{
  vpCameraParameters c = camera;
  
  if(clippingFlag > 3) // Contains at least one FOV constraint
    c.computeFov(I.getWidth(), I.getHeight());

  vpMbtDistanceKltPoints *kltpoly;
  vpMbtDistanceKltCylinder *kltPolyCylinder;

  // Previous version 12/08/2015
//  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
//    kltpoly = *it;
//    kltpoly->polygon->changeFrame(cMo_);
//    kltpoly->polygon->computePolygonClipped(c);
//  }
  faces.computeClippedPolygons(cMo_,c);

  if(useScanLine && !displayFullModel)
    faces.computeScanLineRender(cam,I.getWidth(), I.getHeight());

  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;

    kltpoly->display(I,cMo_,camera,col,thickness,displayFullModel);

    if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->polygon->isVisible() && kltpoly->isTracked()) {
      kltpoly->displayPrimitive(I);
//         faces[i]->displayNormal(I);
    }
  }

  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;

    kltPolyCylinder->display(I,cMo_,camera,col,thickness,displayFullModel);

    if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
      kltPolyCylinder->displayPrimitive(I);
  }

  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles_disp.begin(); it!=circles_disp.end(); ++it){
    (*it)->display(I, cMo_, camera, col, thickness);
  }

#ifdef VISP_HAVE_OGRE
  if(useOgre)
    faces.displayOgre(cMo_);
#endif
}

/*!
  Test the quality of the tracking.
  The tracking is supposed to fail if less than 10 points are tracked.

  \todo Find a efficient way to test the quality.

  \throw vpTrackingException::fatalError  if the test fails.
*/
void
vpMbKltTracker::testTracking()
{
  unsigned int nbTotalPoints = 0;
  vpMbtDistanceKltPoints *kltpoly;
//  for (unsigned int i = 0; i < faces.size(); i += 1){
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2 && kltpoly->hasEnoughPoints()){
      nbTotalPoints += kltpoly->getCurrentNumberPoints();
    }
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    if(kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
      nbTotalPoints += kltPolyCylinder->getCurrentNumberPoints();
  }

  if(nbTotalPoints < 10){
    std::cerr << "test tracking failed (too few points to realize a good tracking)." << std::endl;
    throw vpTrackingException(vpTrackingException::fatalError,
          "test tracking failed (too few points to realize a good tracking).");
  }
}

/*!
  Add a cylinder to display (not for tracking) from two points on the axis (defining the length of
  the cylinder) and its radius.

  \param p1 : First point on the axis.
  \param p2 : Second point on the axis.
  \param radius : Radius of the cylinder.
  \param idFace : Identifier of the polygon representing the revolution axis of the cylinder.
  \param name : The optional name of the cylinder.
*/
void
vpMbKltTracker::initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace,
    const std::string &/*name*/)
{
  vpMbtDistanceKltCylinder *kltPoly = new vpMbtDistanceKltCylinder();
  kltPoly->setCameraParameters(cam) ;

  kltPoly->buildFrom(p1,p2,radius);

  // Add the Cylinder BBox to the list of polygons
  kltPoly->listIndicesCylinderBBox.push_back(idFace+1);
  kltPoly->listIndicesCylinderBBox.push_back(idFace+2);
  kltPoly->listIndicesCylinderBBox.push_back(idFace+3);
  kltPoly->listIndicesCylinderBBox.push_back(idFace+4);

  kltPoly->hiddenface = &faces ;
  kltPoly->useScanLine = useScanLine;
  kltCylinders.push_back(kltPoly);
}

/*!
  Add a circle to display (not for tracking) from its center, 3 points (including the center) defining the plane that contain
  the circle and its radius.

  \param p1 : Center of the circle.
  \param p2,p3 : Two points on the plane containing the circle. With the center of the circle we have 3 points
  defining the plane that contains the circle.
  \param radius : Radius of the circle.
  \param name : The optional name of the circle.
*/
void
vpMbKltTracker::initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
    const int /*idFace*/, const std::string &name)
{
  addCircle(p1, p2, p3, radius, name);
}

/*!
  Add a circle to the list of circles.

  \param P1 : Center of the circle.
  \param P2,P3 : Two points on the plane containing the circle. With the center of the circle we have 3 points
  defining the plane that contains the circle.
  \param r : Radius of the circle.
  \param name : Name of the circle.
*/
void
vpMbKltTracker::addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, const double r, const std::string &name)
{
  bool already_here = false ;
  vpMbtDistanceCircle *ci ;

//  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles_disp.begin(); it!=circles_disp[i].end(); ++it){
//    ci = *it;
//    if((samePoint(*(ci->p1),P1) && samePoint(*(ci->p2),P2) && samePoint(*(ci->p3),P3)) ||
//       (samePoint(*(ci->p1),P1) && samePoint(*(ci->p2),P3) && samePoint(*(ci->p3),P2)) ){
//      already_here = (std::fabs(ci->radius - r) < std::numeric_limits<double>::epsilon() * vpMath::maximum(ci->radius, r));
//    }
//  }

  if (!already_here){
    ci = new vpMbtDistanceCircle ;

    ci->setCameraParameters(cam);
    ci->setName(name);
    ci->buildFrom(P1, P2, P3, r);
    circles_disp.push_back(ci);
  }
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new model
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void
vpMbKltTracker::reInitModel(const vpImage<unsigned char>& I, const std::string &cad_name,
                            const vpHomogeneousMatrix& cMo_, const bool verbose)
{
  reInitModel(I, cad_name.c_str(), cMo_, verbose);
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new model
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void
vpMbKltTracker::reInitModel(const vpImage<unsigned char>& I, const char* cad_name,
                            const vpHomogeneousMatrix& cMo_, const bool verbose)
{
  this->cMo.eye();

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if(cur != NULL){
    cvReleaseImage(&cur);
    cur = NULL;
  }
#endif

  firstInitialisation = true;


  // delete the Klt Polygon features
  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if (kltpoly!=NULL){
      delete kltpoly ;
    }
    kltpoly = NULL ;
  }
  kltPolygons.clear();

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    if (kltPolyCylinder!=NULL){
      delete kltPolyCylinder ;
    }
    kltPolyCylinder = NULL ;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  vpMbtDistanceCircle *ci;
  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles_disp.begin(); it!=circles_disp.end(); ++it){
    ci = *it;
    if (ci!=NULL){
      delete ci ;
    }
    ci = NULL ;
  }


  faces.reset();

  loadModel(cad_name, verbose);
  initFromPose(I, cMo_);
}

/*!
  Set if the polygons that have the given name have to be considered during the tracking phase.

  \param name : name of the polygon(s).
  \param useKltTracking : True if it has to be considered, False otherwise.
*/
void
vpMbKltTracker::setUseKltTracking(const std::string &name, const bool &useKltTracking)
{
  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(kltpoly->polygon->getName() == name){
      kltpoly->setTracked(useKltTracking);
    }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbKltTracker.cpp.o) has no symbols
void dummy_vpMbKltTracker() {};
#endif //VISP_HAVE_OPENCV
