/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Model based tracker using only KLT
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp/vpMbKltTracker.h>

#ifdef VISP_HAVE_OPENCV

vpMbKltTracker::vpMbKltTracker()
{
  cur = NULL;
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
  
  maskBorder = 5;
  threshold_outlier = 0.5;
  percentGood = 0.7;
  
  lambda = 0.8;
  maxIter = 200;
  
  faces = new vpMbHiddenFaces<vpMbtKltPolygon>();

#ifdef VISP_HAVE_OGRE
  faces->getOgreContext()->setWindowName("MBT KLT");
  useOgre = false;
#endif
}

/*!
  Basic destructor.

*/
vpMbKltTracker::~vpMbKltTracker()
{
  if(cur != NULL){
    cvReleaseImage(&cur);
    cur = NULL;
  }
}

void 
vpMbKltTracker::init(const vpImage<unsigned char>& _I)
{
  if(!modelInitialised){
    throw vpException(vpException::fatalError, "model not initialised");
  }
  if(!cameraInitialised){
    throw vpException(vpException::fatalError, "camera not initialised");
  }
  
 bool reInitialisation = false;
  if(!useOgre)
    faces->setVisible(_I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
  else{
#ifdef VISP_HAVE_OGRE   
    if(!faces->isOgreInitialised())
      faces->initOgre(cam);
    
    faces->setVisibleOgre(_I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
    
#else
    vpTRACE("Warning, ViSP doesn't have Ogre3D");
    faces->setVisible(_I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
  }
  
  reinit(_I);
}

void 
vpMbKltTracker::reinit(const vpImage<unsigned char>& _I)
{  
  c0Mo = cMo;
  ctTc0.setIdentity();

  vpImageConvert::convert(_I,cur);
  
  // mask
  IplImage* mask = cvCreateImage(cvSize((int)_I.getWidth(), (int)_I.getHeight()), IPL_DEPTH_8U, 1);
  cvZero(mask);
  
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if((*faces)[i]->isVisible())
        (*faces)[i]->updateMask(mask, 255 - i*15, maskBorder);
  }
  
  tracker.initTracking(cur, mask);
  
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if((*faces)[i]->isVisible()){
      (*faces)[i]->init(tracker);
    }
  }

  cvReleaseImage(&mask);
}

/*!
  Get the current list of KLT points.
  
  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f. This function convert and copy the openCV KLT points into vpImagePoints.
  
  \return the list of KLT points through vpKltOpencv.
*/
std::vector<vpImagePoint> 
vpMbKltTracker::getKltImagePoints()
{
  std::vector<vpImagePoint> kltPoints;
  for (unsigned int i = 0; i < static_cast<unsigned int>(tracker.getNbFeatures()); i ++){
    int id;
    float x_tmp, y_tmp;
    tracker.getFeature((int)i, id, x_tmp, y_tmp);
    kltPoints.push_back(vpImagePoint(y_tmp, x_tmp));
  }
  
  return kltPoints;
}

/*!
  Set the camera parameters

  \param _cam : the new camera parameters
*/
void
vpMbKltTracker::setCameraParameters(const vpCameraParameters& _cam)
{
  for (unsigned int i = 0; i < faces->size(); i += 1){
    (*faces)[i]->setCameraParameters(_cam);
  }
  this->cam = _cam;
  this->cameraInitialised = true;
}

/*!
  set the current pose.

  \param _cMo : the current pose.
*/
void vpMbKltTracker::setPose(const vpHomogeneousMatrix &_cMo)
{
  cMo = _cMo;
}
          
          
/*!
  Initialise a new face from the coordinates given in parameter.

  \param _corners : Coordinates of the corners of the face in the object frame.
  \param _indexFace : index of the face (depends on the vrml file organisation).
*/
void
vpMbKltTracker::initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace)
{  
  vpMbtKltPolygon *polygon = new vpMbtKltPolygon;
//   polygon->setCameraParameters(cam);
  polygon->setNbPoint(_corners.size());
  polygon->setIndex((int)_indexFace);
  for(unsigned int j = 0; j < _corners.size(); j++) {
    polygon->addPoint(j, _corners[j]);
  }
  faces->addPolygon(polygon);
  faces->getPolygon().back()->setCameraParameters(cam);

  delete polygon;
  polygon = NULL;
}

/*!
  Realise the pre tracking operations

  \param _I : The input image.
  \param nbInfos : Size of the features.
  \param nbFaceUsed : Number of face used for the tracking.
*/
void
vpMbKltTracker::preTracking(const vpImage<unsigned char>& _I, unsigned int &nbInfos, unsigned int &nbFaceUsed)
{
  vpImageConvert::convert(_I,cur);
  tracker.track(cur);
  
  nbInfos = 0;  
  nbFaceUsed = 0;
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if((*faces)[i]->isVisible()){
      (*faces)[i]->computeNbDetectedCurrent(tracker);
          
//       (*faces)[i]->ransac();
      if((*faces)[i]->hasEnoughPoints()){
        nbInfos += (*faces)[i]->getNbPointsCur();
        nbFaceUsed++;
      }
    }
  }
}

/*!
  Realise the post tracking operations. Mostly visibility tests
*/
bool
vpMbKltTracker::postTracking(const vpImage<unsigned char>& _I, vpColVector &w)
{
  // # For a better Post Tracking, tracker should reinitialise if so faces don't have enough points but are visible.
  // # Here we are not doing it for more spee performance.
  bool reInitialisation = false;
  
  unsigned int initialNumber = 0;
  unsigned int currentNumber = 0;
  unsigned int shift = 0;
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if((*faces)[i]->isVisible()){
      initialNumber += (*faces)[i]->getInitialNumberPoint();
      if((*faces)[i]->hasEnoughPoints()){    
        vpSubColVector sub_w(w, shift, 2*(*faces)[i]->getNbPointsCur());
        (*faces)[i]->removeOutliers(sub_w, threshold_outlier);
        shift += 2*(*faces)[i]->getNbPointsCur();
        
        currentNumber += (*faces)[i]->getNbPointsCur();
      }
//       else{
//         reInitialisation = true;
//         break;
//       }
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
        faces->setVisible(_I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
      else{
#ifdef VISP_HAVE_OGRE    
        faces->setVisibleOgre(_I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#else
        faces->setVisible(_I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
      }
    }
//   }
  
  if(reInitialisation)
    return true;
  
  return false;
}

/*!
  Realise the VVS loop for the tracking

  \param nbInfos : Size of the features
  \param w : weight of the features after M-Estimation.
*/
void
vpMbKltTracker::computeVVS(const unsigned int &nbInfos, vpColVector &w)
{
  vpMatrix J;     // interaction matrix
  vpColVector R;  // residu
  vpMatrix J_true;     // interaction matrix
  vpColVector R_true;  // residu
  vpColVector v;  // "speed" for VVS
  vpHomography H;
  vpColVector w_true;
  vpRobust robust(2*nbInfos);

  vpMatrix JTJ, JTR;
  
  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  R.resize(2*nbInfos);
  J.resize(2*nbInfos, 6, 0);
  
  while( ((int)((normRes - normRes_1)*1e8) != 0 )  && (iter<maxIter) ){
    
    unsigned int shift = 0;
    for (unsigned int i = 0; i < faces->size(); i += 1){
      if((*faces)[i]->isVisible() && (*faces)[i]->hasEnoughPoints()){
        vpSubColVector subR(R, shift, 2*(*faces)[i]->getNbPointsCur());
        vpSubMatrix subJ(J, shift, 0, 2*(*faces)[i]->getNbPointsCur(), 6);
        try{
          (*faces)[i]->computeHomography(ctTc0, H);
          (*faces)[i]->computeInteractionMatrixAndResidu(subR, subJ);
        }catch(...){
          std::cerr << "exception while tracking face " << i << std::endl;
          throw ;
        }

        shift += 2*(*faces)[i]->getNbPointsCur();
      }
    }

      /* robust */
    if(iter == 0){
      w_true.resize(2*nbInfos);
      w.resize(2*nbInfos);
      w = 1;
    }
    robust.setIteration(iter);
    robust.setThreshold(2/cam.get_px());
    robust.MEstimator( vpRobust::TUKEY, R, w);
    
    if(computeCovariance){
      R_true = R;
      J_true = J;
    }

    normRes_1 = normRes;
    normRes = 0;
    for (unsigned int i = 0; i < static_cast<unsigned int>(R.getRows()); i += 1){
      w_true = w[i] * w[i];
      R[i] = R[i] * w[i];
      normRes += R[i];
    }

    if((iter == 0) || compute_interaction){
      for(unsigned int i=0; i<static_cast<unsigned int>(R.getRows()); i++){
        for(unsigned int j=0; j<6; j++){
          J[i][j] *= w[i];
        }
      }
    }
    
    JTJ = J.AtA();
    computeJTR(J, R, JTR);
    v = -lambda * JTJ.pseudoInverse(1e-16) * JTR;
    
    ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
    
    iter++;
  }
  
  if(computeCovariance){
    vpMatrix D;
    D.diag(w_true);
    covarianceMatrix = vpMatrix::computeCovarianceMatrix(J_true,v,-lambda*R_true,D);
  }
  
  cMo = ctTc0 * c0Mo;
}

/*!
  Realise the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param _I : the input image
*/
void
vpMbKltTracker::track(const vpImage<unsigned char>& _I)
{
  
  unsigned int nbInfos;
  unsigned int nbFaceUsed;
  preTracking(_I, nbInfos, nbFaceUsed);
  
  if(nbInfos < 4 || nbFaceUsed == 0){
    vpERROR_TRACE("\n\t\t Error-> not enough data") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  }

  vpColVector w;
  computeVVS(nbInfos, w);  

  if(postTracking(_I, w))
    reinit(_I);
}

/*!
  Load the xml configuration file.
  From the configuration file parameters write initialize the corresponding objects (Ecm, camera).

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param _filename : full name of the xml file.
*/
void 
vpMbKltTracker::loadConfigFile(const std::string& _filename)
{
  vpMbKltTracker::loadConfigFile(_filename.c_str());
}

/*!
  Load the xml configuration file.
  From the configuration file parameters initialize the corresponding objects (Ecm, camera).

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data). 

  \param filename : full name of the xml file.

  \sa vpXmlParser::cleanup()
*/
void
vpMbKltTracker::loadConfigFile(const char* filename)
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
    
    xmlp.parse(filename);
  }
  catch(...){
    vpERROR_TRACE("Can't open XML file \"%s\"\n ",filename);
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
#else
  vpTRACE("You need the libXML2 to read the config file %s", filename);
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param _I : The image .
  \param _cMo : Pose used to project the 3D model into the image.
  \param _cam : The camera parameters.
  \param _col : The desired color.
  \param _l : The thickness of the lines.
  \param displayFullModel : Boolean to say if all the model has to be displayed.
*/
void
vpMbKltTracker::display(const vpImage<unsigned char>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters & _cam, const vpColor& _col , const unsigned int _l, const bool displayFullModel)
{
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if(displayFullModel || (*faces)[i]->isVisible())
    {
      (*faces)[i]->changeFrame(_cMo);
      std::vector<vpImagePoint> roi = (*faces)[i]->getRoi(_cam);
      for (unsigned int j = 0; j < (*faces)[i]->getNbPoint(); j += 1){
        vpImagePoint ip1, ip2;
        ip1 = roi[j];
        ip2 = roi[(j+1)%(*faces)[i]->getNbPoint()];
        vpDisplay::displayLine (_I, ip1, ip2, _col, _l);
      }
      
      if(displayFeatures && (*faces)[i]->hasEnoughPoints())
        (*faces)[i]->displayPrimitive(_I);
      
//       if(facesTracker[i].hasEnoughPoints())
//         (*faces)[i]->displayNormal(_I);
    }
  }

#ifdef VISP_HAVE_OGRE
  if(useOgre)
    faces->displayOgre(_cMo);
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param _I : The color image .
  \param _cMo : Pose used to project the 3D model into the image.
  \param _cam : The camera parameters.
  \param _col : The desired color.
  \param _l : The thickness of the lines.
  \param displayFullModel : Boolean to say if all the model has to be displayed.
*/
void
vpMbKltTracker::display(const vpImage<vpRGBa>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters & _cam, const vpColor& _col , const unsigned int _l, const bool displayFullModel)
{
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if(displayFullModel || (*faces)[i]->isVisible())
    {
      (*faces)[i]->changeFrame(_cMo);
      std::vector<vpImagePoint> roi = (*faces)[i]->getRoi(_cam);
      for (unsigned int j = 0; j < (*faces)[i]->getNbPoint(); j += 1){
        vpImagePoint ip1, ip2;
        ip1 = roi[j];
        ip2 = roi[(j+1)%(*faces)[i]->getNbPoint()];
        vpDisplay::displayLine (_I, ip1, ip2, _col, _l);
      }
      
      if(displayFeatures && (*faces)[i]->hasEnoughPoints())
        (*faces)[i]->displayPrimitive(_I);
      
//       if(facesTracker[i].hasEnoughPoints())
//         (*faces)[i]->displayNormal(_I);
    }
  }
  
#ifdef VISP_HAVE_OGRE
  if(useOgre)
    faces->displayOgre(_cMo);
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
  for (unsigned int i = 0; i < faces->size(); i += 1){
    if((*faces)[i]->isVisible()){
      nbTotalPoints += (*faces)[i]->getNbPointsCur();
    }
  }

  if(nbTotalPoints < 10){
    std::cerr << "test tracking failed (too few points to realise a good tracking)." << std::endl;
    throw vpTrackingException(vpTrackingException::fatalError,
          "test tracking failed (too few points to realise a good tracking).");
  }
}

#endif //VISP_HAVE_OPENCV
