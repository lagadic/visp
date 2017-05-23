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
 * Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

//#define VP_DEBUG_MODE 1 // Activate debug level 1

#include <visp3/core/vpDebug.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))

vpMbEdgeKltTracker::vpMbEdgeKltTracker()
  : thresholdKLT(2.), thresholdMBT(2.), m_maxIterKlt(30),
    w_mbt(), w_klt(), m_error_hybrid(), m_w_hybrid()
{
  computeCovariance = false;

  angleAppears = vpMath::rad(65);
  angleDisappears = vpMath::rad(75);

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Hybrid");
#endif

  m_lambda = 0.8;
  m_maxIter = 200;
}

/*!
  Basic constructor

*/
vpMbEdgeKltTracker::~vpMbEdgeKltTracker()
{
}

/*!
  Initialization of the tracker using a known initial pose.
  The 3D model must first have been loaded.

  \param I : Input image.
*/
void
vpMbEdgeKltTracker::init(const vpImage<unsigned char>& I)
{
  vpMbKltTracker::init(I);

  initPyramid(I, Ipyramid);

  vpMbEdgeTracker::resetMovingEdge();

  unsigned int i = (unsigned int)scales.size();
  do {
    i--;
    if(scales[i]){
      downScale(i);
      initMovingEdge(*Ipyramid[i], cMo);
      upScale(i);
    }
  } while(i != 0);

  cleanPyramid(Ipyramid);
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track() function.
  This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders.

  \param I : image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void
vpMbEdgeKltTracker::setPose( const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo)
{
    vpMbKltTracker::setPose(I, cdMo);

    resetMovingEdge();

    if(useScanLine){
      cam.computeFov(I.getWidth(), I.getHeight());
      faces.computeClippedPolygons(cMo,cam);
      faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
    }

    initPyramid(I, Ipyramid);

    unsigned int i = (unsigned int)scales.size();
    do {
      i--;
      if(scales[i]){
        downScale(i);
        initMovingEdge(*Ipyramid[i], cMo);
        upScale(i);
      }
    } while(i != 0);

    cleanPyramid(Ipyramid);
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void
vpMbEdgeKltTracker::resetTracker()
{
  vpMbEdgeTracker::resetTracker();
  vpMbKltTracker::resetTracker();
}

unsigned int
vpMbEdgeKltTracker::initMbtTracking(const unsigned int lvl)
{
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  if(lvl  >= scales.size() || !scales[lvl]){
    throw vpException(vpException::dimensionError, "lvl not used.");
  }

  unsigned int nbrow  = 0;
  for(std::list<vpMbtDistanceLine*>::iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    l = *it;

    if(l->isTracked()){
      l->initInteractionMatrixError();
      nbrow += l->nbFeatureTotal;
    }
  }

  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    cy = *it;

    if(cy->isTracked()){
      cy->initInteractionMatrixError();
      nbrow += cy->nbFeature;
    }
  }

  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[lvl].begin(); it!=circles[lvl].end(); ++it){
    ci = *it;

    if(ci ->isTracked()){
      ci->initInteractionMatrixError();
      nbrow += ci->nbFeature;
    }
  }

  return nbrow;
}

/*!
  Load the xml configuration file. An example of such a file is provided in loadConfigFile(const char*) documentation.
  From the configuration file initialize the parameters corresponding to the objects: moving-edges, KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param configFile : full name of the xml file.

  \sa loadConfigFile(const char*), vpXmlParser::cleanup()
*/
void
vpMbEdgeKltTracker::loadConfigFile(const std::string& configFile)
{
  vpMbEdgeKltTracker::loadConfigFile(configFile.c_str());
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the objects: moving-edges, KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data).

  \param configFile : full name of the xml file.

  The XML configuration file has the following form:
  \code
<?xml version="1.0"?>
<conf>
  <ecm>
    <mask>
      <size>5</size>
      <nb_mask>180</nb_mask>
    </mask>
    <range>
      <tracking>10</tracking>
    </range>
    <contrast>
      <edge_threshold>7000</edge_threshold>
      <mu1>0.5</mu1>
      <mu2>0.5</mu2>
    </contrast>
    <sample>
      <step>4</step>
    </sample>
  </ecm>
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
vpMbEdgeKltTracker::loadConfigFile(const char* configFile)
{
#ifdef VISP_HAVE_XML2
  vpMbtEdgeKltXmlParser xmlp;

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  xmlp.setMovingEdge(me);

  xmlp.setMaxFeatures(10000);
  xmlp.setWindowSize(5);
  xmlp.setQuality(0.01);
  xmlp.setMinDistance(5);
  xmlp.setHarrisParam(0.01);
  xmlp.setBlockSize(3);
  xmlp.setPyramidLevels(3);
  xmlp.setMaskBorder(maskBorder);

  try{
    std::cout << " *********** Parsing XML for Mb Edge Tracker ************ " << std::endl;
    xmlp.parse(configFile);
  }
  catch(...){
    vpERROR_TRACE("Can't open XML file \"%s\"\n ", configFile);
    throw vpException(vpException::ioError, "problem to parse configuration file.");
  }

  vpCameraParameters camera;
  xmlp.getCameraParameters(camera);
  setCameraParameters(camera);

  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  if(xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());

  if(xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());

  if(xmlp.getFovClipping()){
    setClipping(vpMbEdgeTracker::clippingFlag | vpPolygon3D::FOV_CLIPPING);
  }

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

  vpMe meParser;
  xmlp.getMe(meParser);
  vpMbEdgeTracker::setMovingEdge(meParser);

  tracker.setMaxFeatures((int)xmlp.getMaxFeatures());
  tracker.setWindowSize((int)xmlp.getWindowSize());
  tracker.setQuality(xmlp.getQuality());
  tracker.setMinDistance(xmlp.getMinDistance());
  tracker.setHarrisFreeParameter(xmlp.getHarrisParam());
  tracker.setBlockSize((int)xmlp.getBlockSize());
  tracker.setPyramidLevels((int)xmlp.getPyramidLevels());
  maskBorder = xmlp.getMaskBorder();

  //if(useScanLine)
  faces.getMbScanLineRenderer().setMaskBorder(maskBorder);

#else
  vpTRACE("You need the libXML2 to read the config file %s", configFile);
#endif
}

/*!
  Realize the post tracking operations. Mostly visibility tests
*/
bool
vpMbEdgeKltTracker::postTracking(const vpImage<unsigned char>& I, vpColVector &w_mbt, vpColVector &w_klt,
                                 const unsigned int lvl)
{

  postTrackingMbt(w_mbt,lvl);

  if (displayFeatures)
  {
    if(lvl == 0){
      vpMbtDistanceLine* l;
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
        l = *it;
        if (l->isVisible() && l->isTracked()){
          l->displayMovingEdges(I);
        }
      }

      vpMbtDistanceCylinder *cy ;
      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
        cy = *it;
        // A cylinder is always visible: #FIXME AY: Still valid?
        if(cy->isTracked())
          cy->displayMovingEdges(I);
      }

      vpMbtDistanceCircle *ci ;
      for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[lvl].begin(); it!=circles[lvl].end(); ++it){
        ci = *it;
        if (ci->isVisible() && ci->isTracked()){
          ci->displayMovingEdges(I);
        }
      }
    }
  }

  bool reInit = vpMbKltTracker::postTracking(I, w_klt);

  if(useScanLine){
    cam.computeFov(I.getWidth(), I.getHeight());
    faces.computeClippedPolygons(cMo,cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
  }

  vpMbEdgeTracker::updateMovingEdge(I);

  vpMbEdgeTracker::initMovingEdge(I, cMo) ;
  vpMbEdgeTracker::reinitMovingEdge(I, cMo);

  if(computeProjError)
    vpMbEdgeTracker::computeProjectionError(I);

  if(reInit)
    return true;

  return false;
}

/*!
  Post tracking computation. Compute the mean weight of a line and, check the
  weight associated to a site (to eventually remove an outlier) and eventually
  set a flag to re-initialize the line.

  \warning level parameter not yet implemented.

  \param w : Vector of weight associated to the residual.
  \param lvl : Optional parameter to specify the level to track.
*/
void
vpMbEdgeKltTracker::postTrackingMbt(vpColVector &w, const unsigned int lvl)
{
  if(lvl  >= scales.size() || !scales[lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }

  unsigned int n =0 ;
  vpMbtDistanceLine* l;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    if((*it)->isTracked()){
      l = *it;
      unsigned int indexLine = 0;
      double wmean = 0 ;

      for(unsigned int a = 0 ; a < l->meline.size() ; a++)
      {
        std::list<vpMeSite>::iterator itListLine;
        if (l->nbFeature[a] > 0) itListLine = l->meline[a]->getMeList().begin();

        for (unsigned int i=0 ; i < l->nbFeature[a] ; i++){
          wmean += w[n+indexLine] ;
          vpMeSite p = *itListLine;
          if (w[n+indexLine] < 0.5){
            p.setState(vpMeSite::M_ESTIMATOR);
            *itListLine = p;
          }

          ++itListLine;
          indexLine++;
        }
      }

      n+= l->nbFeatureTotal ;

      if (l->nbFeatureTotal!=0)
        wmean /= l->nbFeatureTotal ;
      else
        wmean = 1;

      l->setMeanWeight(wmean);

      if (wmean < 0.8)
        l->Reinit = true;
    }
  }

  // Same thing with cylinders as with lines
  vpMbtDistanceCylinder *cy ;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    if((*it)->isTracked()){
      cy = *it;
      double wmean = 0 ;
      std::list<vpMeSite>::iterator itListCyl1;
      std::list<vpMeSite>::iterator itListCyl2;
      if (cy->nbFeature > 0){
        itListCyl1 = cy->meline1->getMeList().begin();
        itListCyl2 = cy->meline2->getMeList().begin();
      }

      wmean = 0;
      for(unsigned int i=0 ; i < cy->nbFeaturel1 ; i++){
        wmean += w[n+i] ;
        vpMeSite p = *itListCyl1;
        if (w[n+i] < 0.5){
          p.setState(vpMeSite::M_ESTIMATOR);

          *itListCyl1 = p;
        }

        ++itListCyl1;
      }

      if (cy->nbFeaturel1!=0)
        wmean /= cy->nbFeaturel1 ;
      else
        wmean = 1;

      cy->setMeanWeight1(wmean);

      if (wmean < 0.8){
        cy->Reinit = true;
      }

      wmean = 0;
      for(unsigned int i=cy->nbFeaturel1 ; i < cy->nbFeature ; i++){
        wmean += w[n+i] ;
        vpMeSite p = *itListCyl2;
        if (w[n+i] < 0.5){
          p.setState(vpMeSite::M_ESTIMATOR);

          *itListCyl2 = p;
        }

        ++itListCyl2;
      }

      if (cy->nbFeaturel2!=0)
        wmean /= cy->nbFeaturel2 ;
      else
        wmean = 1;

      cy->setMeanWeight2(wmean);

      if (wmean < 0.8){
        cy->Reinit = true;
      }

      n+= cy->nbFeature ;
    }
  }

  // Same thing with circles as with lines
  vpMbtDistanceCircle *ci;
  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[lvl].begin(); it!=circles[lvl].end(); ++it){
    if((*it)->isTracked()){
      ci = *it;
      double wmean = 0 ;
      std::list<vpMeSite>::iterator itListCir;

      if (ci->nbFeature > 0){
        itListCir = ci->meEllipse->getMeList().begin();
      }

      wmean = 0;
      for(unsigned int i=0 ; i < ci->nbFeature ; i++){
        wmean += w[n+i] ;
        vpMeSite p = *itListCir;
        if (w[n+i] < 0.5){
          p.setState(vpMeSite::M_ESTIMATOR);

          *itListCir = p;
        }

        ++itListCir;
      }

      if (ci->nbFeature!=0)
        wmean /= ci->nbFeature ;
      else
        wmean = 1;

      ci->setMeanWeight(wmean);

      if (wmean < 0.8){
        ci->Reinit = true;
      }

      n+= ci->nbFeature ;
    }
  }
}

/*!
  Realize the VVS loop for the tracking

  \param I : current image.
  \param nbInfos : Size of the features (KLT).
  \param nbrow : Size of the features (Edge).
  \param lvl : level of the pyramid.
*/
void
vpMbEdgeKltTracker::computeVVS(const vpImage<unsigned char>& I, const unsigned int &nbInfos, unsigned int &nbrow, const unsigned int lvl)
{
  vpColVector factor;
  nbrow = trackFirstLoop(I, factor, lvl);

  if(nbrow < 4 && nbInfos < 4){
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  }
  else if(nbrow < 4)
    nbrow = 0;

  unsigned int totalNbRows = nbrow + 2*nbInfos;
  double residu = 0;
  double residu_1 = -1;
  unsigned int iter = 0;

  vpMatrix L(totalNbRows, 6);
  vpMatrix L_mbt, L_klt;     // interaction matrix
  vpColVector weighted_error(totalNbRows);
  vpColVector R_mbt, R_klt;  // residu
  vpMatrix L_true;
  vpMatrix LVJ_true;

  if(nbrow != 0){
    L_mbt.resize(nbrow, 6, false);
    R_mbt.resize(nbrow, false);
  }

  if(nbInfos != 0){
    L_klt.resize(2*nbInfos, 6, false);
    R_klt.resize(2*nbInfos, false);
  }

  vpColVector v;  // "speed" for VVS
  vpRobust robust_mbt(0), robust_klt(0);
  vpHomography H;

  vpMatrix LTL;
  vpColVector LTR;

  double factorMBT = 1.0;
  double factorKLT = 1.0;

  //More efficient weight repartition for hybrid tracker should come soon...
//   factorMBT = 1.0 - (double)nbrow / (double)(nbrow + nbInfos);
//   factorKLT = 1.0 - factorMBT;
  factorMBT = 0.35;
  factorKLT = 0.65;

  if (nbrow < 4)
    factorKLT = 1.;
  if (nbInfos < 4)
    factorMBT = 1.;

  double residuMBT = 0;
  double residuKLT = 0;

  vpHomogeneousMatrix cMoPrev;
  vpHomogeneousMatrix ctTc0_Prev;
  vpColVector m_error_prev;
  vpColVector m_w_prev;

  //Init size
  m_error_hybrid.resize(totalNbRows, false);
  m_w_hybrid.resize(totalNbRows, false);

  if (nbrow != 0) {
    w_mbt.resize(nbrow, false);
    w_mbt = 1; //needed in vpRobust::psiTukey()
    robust_mbt.resize(nbrow);
  }

  if (nbInfos != 0) {
    w_klt.resize(2*nbInfos, false);
    w_klt = 1; //needed in vpRobust::psiTukey()
    robust_klt.resize(2*nbInfos);
  }

  double mu = m_initialMu;

  while( ((int)((residu - residu_1)*1e8) !=0 )  && (iter < m_maxIter) ){
    if (nbrow >= 4)
      trackSecondLoop(I, L_mbt, R_mbt, cMo, lvl);

    if (nbInfos >= 4) {
      unsigned int shift = 0;
      vpMbtDistanceKltPoints *kltpoly;

      for (std::list<vpMbtDistanceKltPoints*>::const_iterator it=vpMbKltTracker::kltPolygons.begin(); it!=vpMbKltTracker::kltPolygons.end(); ++it) {
        kltpoly = *it;
        if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->hasEnoughPoints()) {
          vpSubColVector subR(R_klt, shift, 2*kltpoly->getCurrentNumberPoints());
          vpSubMatrix subL(L_klt, shift, 0, 2*kltpoly->getCurrentNumberPoints(), 6);
          kltpoly->computeHomography(ctTc0, H);
          kltpoly->computeInteractionMatrixAndResidu(subR, subL);
          shift += 2*kltpoly->getCurrentNumberPoints();
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for (std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it) {
        kltPolyCylinder = *it;

        if(kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
        {
          vpSubColVector subR(R_klt, shift, 2*kltPolyCylinder->getCurrentNumberPoints());
          vpSubMatrix subL(L_klt, shift, 0, 2*kltPolyCylinder->getCurrentNumberPoints(), 6);
          try {
            kltPolyCylinder->computeInteractionMatrixAndResidu(ctTc0,subR, subL);
          } catch(...) {
            throw vpTrackingException(vpTrackingException::fatalError, "Cannot compute interaction matrix");
          }

          shift += 2*kltPolyCylinder->getCurrentNumberPoints();
        }
      }
    }

    /* residuals */
    if (nbrow > 3) {
      m_error_hybrid.insert(0, R_mbt);
    }

    if (nbInfos > 3) {
      m_error_hybrid.insert(nbrow, R_klt);
    }

    unsigned int cpt = 0;
    while (cpt< (nbrow+2*nbInfos)) {
      if (cpt<(unsigned)nbrow) {
        m_w_hybrid[cpt] = ((w_mbt[cpt] * factor[cpt]) * factorMBT);
      } else {
        m_w_hybrid[cpt] = (w_klt[cpt-nbrow] * factorKLT);
      }
      cpt++;
    }

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_hybrid, m_error_prev, cMoPrev, mu, reStartFromLastIncrement, &m_w_prev);
    if (reStartFromLastIncrement) {
      ctTc0 = ctTc0_Prev;
    }

    if (!reStartFromLastIncrement) {
        /* robust */
      if (nbrow > 3) {
        residuMBT = 0;
        for(unsigned int i = 0; i < R_mbt.getRows(); i++)
          residuMBT += fabs(R_mbt[i]);
        residuMBT /= R_mbt.getRows();

        robust_mbt.setThreshold(thresholdMBT/cam.get_px());
        robust_mbt.MEstimator( vpRobust::TUKEY, R_mbt, w_mbt);

        L.insert(L_mbt, 0, 0);
      }

      if (nbInfos > 3) {
        residuKLT = 0;
        for(unsigned int i = 0; i < R_klt.getRows(); i++)
          residuKLT += fabs(R_klt[i]);
        residuKLT /= R_klt.getRows();

        robust_klt.setThreshold(thresholdKLT/cam.get_px());
        robust_klt.MEstimator( vpRobust::TUKEY, R_klt, w_klt);

        L.insert(L_klt, nbrow, 0);
      }

      unsigned int cpt = 0;
      while (cpt< (nbrow+2*nbInfos)) {
        if (cpt<(unsigned)nbrow) {
          m_w_hybrid[cpt] = ((w_mbt[cpt] * factor[cpt]) * factorMBT);
        } else {
          m_w_hybrid[cpt] = (w_klt[cpt-nbrow] * factorKLT);
        }
        cpt++;
      }

      if (computeCovariance) {
        L_true = L;
        if (!isoJoIdentity) {
           vpVelocityTwistMatrix cVo;
           cVo.buildFrom(cMo);
           LVJ_true = (L*cVo*oJo);
        }
      }

      residu_1 = residu;
      residu = 0;
      double num = 0;
      double den = 0;

      for (unsigned int i = 0; i < weighted_error.getRows(); i++) {
        num += m_w_hybrid[i]*vpMath::sqr(m_error_hybrid[i]);
        den += m_w_hybrid[i];

        weighted_error[i] = m_error_hybrid[i] * m_w_hybrid[i];
        if (m_computeInteraction) {
          for (unsigned int j = 0; j < 6; j += 1) {
            L[i][j] *= m_w_hybrid[i];
          }
        }
      }

      residu = sqrt(num/den);

      computeVVSPoseEstimation(isoJoIdentity, iter, L, LTL, weighted_error, m_error_hybrid, m_error_prev, LTR, mu, v, &m_w_hybrid, &m_w_prev);

      cMoPrev = cMo;
      ctTc0_Prev = ctTc0;
      ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
      cMo = ctTc0 * c0Mo;
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity, m_w_hybrid, cMoPrev, L_true, LVJ_true, m_error_hybrid);
}

void
vpMbEdgeKltTracker::computeVVSInit() {
  throw vpException(vpException::fatalError, "vpMbEdgeKltTracker::computeVVSInit() should not be called!");
}

void
vpMbEdgeKltTracker::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbEdgeKltTracker::computeVVSInteractionMatrixAndResidu() should not be called!");
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed.

  \param I : the input image.
*/
void
vpMbEdgeKltTracker::track(const vpImage<unsigned char>& I)
{
  try{
    vpMbKltTracker::preTracking(I);
  }
  catch(...){}

  if(m_nbInfos >= 4) {
    unsigned int old_maxIter = m_maxIter;
    m_maxIter = m_maxIterKlt;
    vpMbKltTracker::computeVVS();
    m_maxIter = old_maxIter;
  }
  else{
    m_nbInfos = 0;
    // std::cout << "[Warning] Unable to init with KLT" << std::endl;
  }

  vpMbEdgeTracker::trackMovingEdge(I);

  unsigned int nbrow = 0;
  computeVVS(I, m_nbInfos, nbrow);

  if(postTracking(I, w_mbt, w_klt)){
    vpMbKltTracker::reinit(I);

    // AY : Removed as edge tracked, if necessary, is reinitialized in postTracking()

//    initPyramid(I, Ipyramid);

//    unsigned int i = (unsigned int)scales.size();
//    do {
//      i--;
//      if(scales[i]){
//        downScale(i);
//        initMovingEdge(*Ipyramid[i], cMo);
//        upScale(i);
//      }
//    } while(i != 0);

//    cleanPyramid(Ipyramid);
  }
}

unsigned int
vpMbEdgeKltTracker::trackFirstLoop(const vpImage<unsigned char>& I, vpColVector &factor, const unsigned int lvl)
{
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  if(lvl >= scales.size() || !scales[lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }

  unsigned int nbrow = initMbtTracking(lvl);

  if (nbrow==0){
//     throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features in the interaction matrix...");
      return nbrow;
  }

  factor.resize(nbrow);
  factor = 1;

  unsigned int n = 0;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    if((*it)->isTracked()){
      l = *it;
      l->computeInteractionMatrixError(cMo);

      double fac = 1;
      for(std::list<int>::const_iterator itindex = l->Lindex_polygon.begin(); itindex!=l->Lindex_polygon.end(); ++itindex){
        int index = *itindex;
        if (l->hiddenface->isAppearing((unsigned int)index)) {
          fac = 0.2;
          break;
        }
        if(l->closeToImageBorder(I, 10)){
          fac = 0.1;
          break;
        }
      }

      unsigned int indexFeature = 0;
      for(unsigned int a = 0 ; a < l->meline.size(); a++){
        std::list<vpMeSite>::const_iterator itListLine;
        if (l->meline[a] != NULL)
        {
          itListLine = l->meline[a]->getMeList().begin();

          for (unsigned int i=0 ; i < l->nbFeature[a] ; i++){
              factor[n+i] = fac;
              vpMeSite site = *itListLine;
              if (site.getState() != vpMeSite::NO_SUPPRESSION) factor[n+i] = 0.2;
              ++itListLine;
              indexFeature++;
          }
          n+= l->nbFeature[a];
        }
      }
    }
  }

  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    if((*it)->isTracked()){
      cy = *it;
      cy->computeInteractionMatrixError(cMo, I);
      double fac = 1.0;

      std::list<vpMeSite>::const_iterator itCyl1;
      std::list<vpMeSite>::const_iterator itCyl2;
      if ((cy->meline1 != NULL || cy->meline2 != NULL)){
        itCyl1 = cy->meline1->getMeList().begin();
        itCyl2 = cy->meline2->getMeList().begin();
      }

      for(unsigned int i=0 ; i < cy->nbFeature ; i++){
        factor[n+i] = fac;
        vpMeSite site;
        if(i<cy->nbFeaturel1) {
          site= *itCyl1;
          ++itCyl1;
        }
        else{
          site= *itCyl2;
          ++itCyl2;
        }
        if (site.getState() != vpMeSite::NO_SUPPRESSION) factor[n+i] = 0.2;
      }

      n+= cy->nbFeature;
    }
  }

  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[lvl].begin(); it!=circles[lvl].end(); ++it){
    if((*it)->isTracked()){
      ci = *it;
      ci->computeInteractionMatrixError(cMo);
      double fac = 1.0;

      std::list<vpMeSite>::const_iterator itCir;
      if (ci->meEllipse != NULL) {
        itCir = ci->meEllipse->getMeList().begin();
      }

      for(unsigned int i=0 ; i < ci->nbFeature ; i++){
        factor[n+i] = fac;
        vpMeSite site = *itCir;
        if (site.getState() != vpMeSite::NO_SUPPRESSION) factor[n+i] = 0.2;
        ++itCir;
      }

      n+= ci->nbFeature;
    }
  }

  return nbrow;
}

void
vpMbEdgeKltTracker::trackSecondLoop(const vpImage<unsigned char>& I,  vpMatrix &L, vpColVector &error,
                                    vpHomogeneousMatrix& cMo_, const unsigned int lvl)
{
  vpMbtDistanceLine* l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  unsigned int n = 0 ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    if((*it)->isTracked()){
      l = *it;
      l->computeInteractionMatrixError(cMo_) ;
      for (unsigned int i=0 ; i < l->nbFeatureTotal ; i++){
        for (unsigned int j=0; j < 6 ; j++){
          L[n+i][j] = l->L[i][j];
          error[n+i] = l->error[i];
        }
      }
      n+= l->nbFeatureTotal;
    }
  }

  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    if((*it)->isTracked()){
      cy = *it;
      cy->computeInteractionMatrixError(cMo_, I) ;
      for(unsigned int i=0 ; i < cy->nbFeature ; i++){
        for(unsigned int j=0; j < 6 ; j++){
          L[n+i][j] = cy->L[i][j];
          error[n+i] = cy->error[i];
        }
      }
      n+= cy->nbFeature;
    }
  }
  for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[lvl].begin(); it!=circles[lvl].end(); ++it){
    if((*it)->isTracked()){
      ci = *it;
      ci->computeInteractionMatrixError(cMo_) ;
      for(unsigned int i=0 ; i < ci->nbFeature ; i++){
        for(unsigned int j=0; j < 6 ; j++){
          L[n+i][j] = ci->L[i][j];
          error[n+i] = ci->error[i];
        }
      }

      n+= ci->nbFeature;
    }
  }
}

/*!
  Set the camera parameters

  \param camera : the new camera parameters
*/
void
vpMbEdgeKltTracker::setCameraParameters(const vpCameraParameters& camera)
{
  this->cam = camera;

  vpMbEdgeTracker::setCameraParameters(cam);
  vpMbKltTracker::setCameraParameters(cam);
}

/*!
  Initialise a new face from the coordinates given in parameter.

  \param polygon : The polygon describing the set of lines that has to be tracked.
*/
void
vpMbEdgeKltTracker::initFaceFromCorners(vpMbtPolygon &polygon)
{
  vpMbEdgeTracker::initFaceFromCorners(polygon);
  vpMbKltTracker::initFaceFromCorners(polygon);
}
/*!
  Initialise a new face from the coordinates given in parameter.

  \param polygon : The polygon describing the set of lines that has to be tracked.
*/
void
vpMbEdgeKltTracker::initFaceFromLines(vpMbtPolygon &polygon)
{
  vpMbEdgeTracker::initFaceFromLines(polygon);
  vpMbKltTracker::initFaceFromLines(polygon);
}

/*!
  Add a circle to track from its center, 3 points (including the center) defining the plane that contain
  the circle and its radius.

  \param p1 : Center of the circle.
  \param p2,p3 : Two points on the plane containing the circle. With the center of the circle we have 3 points
  defining the plane that contains the circle.
  \param radius : Radius of the circle.
  \param idFace : Id of the face associated to the circle.
  \param name : The optional name of the circle.
*/
void
vpMbEdgeKltTracker::initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
    const int idFace, const std::string &name)
{
  vpMbEdgeTracker::initCircle(p1, p2, p3, radius, idFace, name);
}

/*!
  Add a cylinder to track from tow points on the axis (defining the length of
  the cylinder) and its radius.

  \param p1 : First point on the axis.
  \param p2 : Second point on the axis.
  \param radius : Radius of the cylinder.
  \param idFace : Id of the face associated to the cylinder.
  \param name : The optional name of the cylinder.
*/
void
vpMbEdgeKltTracker::initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace,
    const std::string &name)
{
  vpMbEdgeTracker::initCylinder(p1, p2, radius, idFace, name);
  vpMbKltTracker::initCylinder(p1, p2, radius, idFace, name);
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : boolean to say if all the model has to be displayed, even the faces that are not visible.
*/
void
vpMbEdgeKltTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &camera,
                            const vpColor& col, const unsigned int thickness, const bool displayFullModel)
{
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
        (*it)->display(I,cMo_, camera, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
        (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[scaleLevel].begin(); it!=circles[scaleLevel].end(); ++it){
        (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
      }

      break ; //displaying model on one scale only
    }
  }

  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
        kltpoly->displayPrimitive(I);
    }
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
      kltPolyCylinder->displayPrimitive(I);
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
  \param displayFullModel : boolean to say if all the model has to be displayed, even the faces that are not visible.
*/
void
vpMbEdgeKltTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &camera,
                            const vpColor& col , const unsigned int thickness, const bool displayFullModel)
{
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
        (*it)->display(I,cMo_, camera, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
        (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[scaleLevel].begin(); it!=circles[scaleLevel].end(); ++it){
        (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
      }

      break ; //displaying model on one scale only
    }
  }

  vpMbtDistanceKltPoints *kltpoly;
  for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
    kltpoly = *it;
    if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
        kltpoly->displayPrimitive(I);
    }
  }

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
    kltPolyCylinder = *it;
    if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
      kltPolyCylinder->displayPrimitive(I);
  }

#ifdef VISP_HAVE_OGRE
  if(useOgre)
    faces.displayOgre(cMo_);
#endif
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
vpMbEdgeKltTracker::reInitModel(const vpImage<unsigned char>& I, const std::string &cad_name,
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
vpMbEdgeKltTracker::reInitModel(const vpImage<unsigned char>& I, const char* cad_name,
                                const vpHomogeneousMatrix& cMo_, const bool verbose)
{
  // Reinit klt
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

  firstInitialisation = true;

  // Reinit edge
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;

  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
        l = *it;
        if (l!=NULL) delete l ;
        l = NULL ;
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
        cy = *it;
        if (cy!=NULL) delete cy;
        cy = NULL;
      }

      for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[i].begin(); it!=circles[i].end(); ++it){
        ci = *it;
        if (ci!=NULL) delete ci;
        ci = NULL;
      }

      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }

  //compute_interaction=1;
  nline = 0;
  ncylinder = 0;
  ncircle = 0;
  //lambda = 1;
  nbvisiblepolygone = 0;

  // Reinit common parts
  faces.reset();

  loadModel(cad_name, verbose);

  this->cMo = cMo_;
  init(I);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbEdgeKltTracker.cpp.o) has no symbols
void dummy_vpMbEdgeKltTracker() {};
#endif //VISP_HAVE_OPENCV
