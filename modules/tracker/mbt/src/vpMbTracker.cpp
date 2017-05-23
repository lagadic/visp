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
 * Generic model based tracker
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbTracker.cpp
  \brief Generic model based tracker
*/

#include <iostream>
#include <limits>
#include <algorithm>
#include <map>

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpDisplay.h>
#ifdef VISP_HAVE_MODULE_GUI
#  include <visp3/gui/vpDisplayOpenCV.h>
#  include <visp3/gui/vpDisplayX.h>
#  include <visp3/gui/vpDisplayGDI.h>
#endif
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpException.h>
#ifdef VISP_HAVE_MODULE_IO
#  include <visp3/io/vpImageIo.h>
#endif
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTrackingException.h>

#ifdef VISP_HAVE_COIN3D
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
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#endif



#ifndef DOXYGEN_SHOULD_SKIP_THIS

namespace {
  /*!
    Structure to store info about segment in CAO model files.
   */
  struct SegmentInfo {
    SegmentInfo() : extremities(), name(), useLod(false), minLineLengthThresh(0.) {}

    std::vector<vpPoint> extremities;
    std::string name;
    bool useLod;
    double minLineLengthThresh;
  };

  /*!
    Structure to store info about a polygon face represented by a vpPolygon and by a list of vpPoint
    representing the corners of the polygon face in 3D.
   */
  struct PolygonFaceInfo {
    PolygonFaceInfo(const double dist, const vpPolygon &poly, const std::vector<vpPoint> &corners)
      : distanceToCamera(dist), polygon(poly), faceCorners(corners) {}

    bool operator<(const PolygonFaceInfo &pfi) const {
      return distanceToCamera < pfi.distanceToCamera;
    }

    double distanceToCamera;
    vpPolygon polygon;
    std::vector<vpPoint> faceCorners;
  };
}
#endif // DOXYGEN_SHOULD_SKIP_THIS

/*!
  Basic constructor.
  Set default values.

*/
vpMbTracker::vpMbTracker()
: cam(), cMo(), oJo(6,6), isoJoIdentity(true), modelFileName(), modelInitialised(false),
  poseSavingFilename(), computeCovariance(false), covarianceMatrix(), computeProjError(false),
  projectionError(90.0), displayFeatures(false), m_optimizationMethod(vpMbTracker::GAUSS_NEWTON_OPT),
  faces(), angleAppears( vpMath::rad(89) ), angleDisappears( vpMath::rad(89) ), distNearClip(0.001),
  distFarClip(100), clippingFlag(vpPolygon3D::NO_CLIPPING), useOgre(false), ogreShowConfigDialog(false), useScanLine(false),
  nbPoints(0), nbLines(0), nbPolygonLines(0), nbPolygonPoints(0), nbCylinders(0), nbCircles(0),
  useLodGeneral(false), applyLodSettingInConfig(false), minLineLengthThresholdGeneral(50.0), minPolygonAreaThresholdGeneral(2500.0),
  mapOfParameterNames(), m_computeInteraction(true), m_lambda(1.0), m_maxIter(30), m_stopCriteriaEpsilon(1e-8), m_initialMu(0.01)
{
    oJo.eye();
    //Map used to parse additional information in CAO model files,
    //like name of faces or LOD setting
    mapOfParameterNames["name"] = "string";
    mapOfParameterNames["minPolygonAreaThreshold"] = "number";
    mapOfParameterNames["minLineLengthThreshold"] = "number";
    mapOfParameterNames["useLod"] = "boolean";
}

/*!
  Basic destructor that doest nothing.
*/
vpMbTracker::~vpMbTracker()
{
}

#ifdef VISP_HAVE_MODULE_GUI
/*!
  Initialise the tracker by clicking in the image on the pixels that correspond to the
  3D points whose coordinates are extracted from a file. In this file, comments starting
  with # character are allowed. Notice that 3D point coordinates are expressed in meter
  in the object frame with their X, Y and Z values.

  The structure of this file is the following:

  \code
  # 3D point coordinates
  4                 # Number of points in the file (minimum is four)
  0.01 0.01 0.01    # \
  ...               #  | 3D coordinates in the object frame (X, Y, Z)
  0.01 -0.01 -0.01  # /
  \endcode

  \param I : Input image where the user has to click.
  \param initFile : File containing the coordinates of at least 4 3D points the user has
  to click in the image. This file should have .init extension (ie teabox.init).
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.

*/
void
vpMbTracker::initClick(const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp)
{
  vpHomogeneousMatrix last_cMo;
  vpPoseVector init_pos;
  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

  std::string ext = ".init";
  std::string str_pose = "";
  size_t pos = (unsigned int)initFile.rfind(ext);

  // Load the last poses from files
  std::fstream finitpos ;
  std::fstream finit ;
  char s[FILENAME_MAX];
  if (poseSavingFilename.empty()) {
    if ( pos == initFile.size()-ext.size() && pos != 0)
      str_pose = initFile.substr(0,pos) + ".0.pos";
    else
      str_pose =  initFile + ".0.pos";

    finitpos.open(str_pose.c_str() ,std::ios::in) ;
    sprintf(s, "%s", str_pose.c_str());
  } else {
    finitpos.open(poseSavingFilename.c_str() ,std::ios::in) ;
    sprintf(s, "%s", poseSavingFilename.c_str());
  }
  if (finitpos.fail() ){
    std::cout << "cannot read " << s << std::endl << "cMo set to identity" << std::endl;
    last_cMo.eye();
  }
  else {
    for (unsigned int i = 0; i < 6; i += 1){
      finitpos >> init_pos[i];
    }

    finitpos.close();
    last_cMo.buildFrom(init_pos) ;

    std::cout <<"last_cMo : "<<std::endl << last_cMo <<std::endl;

    vpDisplay::display(I);
    display(I, last_cMo, cam, vpColor::green, 1, true);
    vpDisplay::displayFrame(I, last_cMo, cam, 0.05, vpColor::green);
    vpDisplay::flush(I);

    std::cout << "No modification : left click " << std::endl;
    std::cout << "Modify initial pose : right click " << std::endl ;

    vpDisplay::displayText(I, 15, 10,
                           "left click to validate, right click to modify initial pose",
                           vpColor::red);

    vpDisplay::flush(I) ;

    while (!vpDisplay::getClick(I, ip, button)) ;
  }


  if (!finitpos.fail() && button == vpMouseButton::button1){
    cMo = last_cMo ;
  }
  else
  {
    vpDisplay *d_help = NULL;

    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;

    vpPose pose ;

    pose.clearPoint() ;

    // file parser
    // number of points
    // X Y Z
    // X Y Z
    if( pos == initFile.size()-ext.size() && pos != 0)
      sprintf(s,"%s", initFile.c_str());
    else
      sprintf(s,"%s.init", initFile.c_str());

    std::cout << "Load 3D points from: " << s << std::endl ;
    finit.open(s,std::ios::in) ;
    if (finit.fail()){
      std::cout << "cannot read " << s << std::endl;
      throw vpException(vpException::ioError, "Cannot open model-based tracker init file %s", s);
    }

#ifdef VISP_HAVE_MODULE_IO
    //Display window creation and initialisation
    try{
      if(displayHelp){
        std::string dispF;
        if( pos == initFile.size()-ext.size() && pos != 0)
          dispF = initFile.substr(0,pos) + ".ppm";
        else
          dispF = initFile + ".ppm";

        if (vpIoTools::checkFilename(dispF)) {
          std::cout << "Load image to help initialization: " << dispF << std::endl;
#if defined VISP_HAVE_X11
          d_help = new vpDisplayX ;
#elif defined VISP_HAVE_GDI
          d_help = new vpDisplayGDI;
#elif defined VISP_HAVE_OPENCV
          d_help = new vpDisplayOpenCV;
#endif

          vpImage<vpRGBa> Iref ;
          vpImageIo::read(Iref, dispF) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
          d_help->init(Iref, I.display->getWindowXPosition()+(int)I.getWidth()+80, I.display->getWindowYPosition(),
                       "Where to initialize...")  ;
          vpDisplay::display(Iref) ;
          vpDisplay::flush(Iref);
#endif
        }
      }
    }
    catch(...) {
      if(d_help != NULL) {
        delete d_help;
        d_help = NULL;
      }
    }
#else //#ifdef VISP_HAVE_MODULE_IO
    (void)(displayHelp);
#endif //#ifdef VISP_HAVE_MODULE_IO
    char c;
    // skip lines starting with # as comment
    finit.get(c);
    while (!finit.fail() && (c == '#')) {
      finit.ignore(256, '\n');
      finit.get(c);
    }
    finit.unget();

    unsigned int n3d;
    finit >> n3d ;
    finit.ignore(256, '\n'); // skip the rest of the line
    std::cout << "Number of 3D points  " << n3d << std::endl ;
    if (n3d > 100000) {
      throw vpException(vpException::badValue, "In %s file, the number of 3D points exceed the max allowed", s);
    }

    vpPoint *P = new vpPoint [n3d]  ;
    for (unsigned int i=0 ; i < n3d ; i++){
      // skip lines starting with # as comment
      finit.get(c);
      while (!finit.fail() && (c == '#')) {
        finit.ignore(256, '\n');
        finit.get(c);
      }
      finit.unget();
      double X,Y,Z;

      finit >> X;
      finit >> Y;
      finit >> Z;
      finit.ignore(256, '\n'); // skip the rest of the line

      std::cout << "Point " << i+1 << " with 3D coordinates: " << X << " " << Y << " " << Z << std::endl;
      P[i].setWorldCoordinates(X,Y,Z) ; // (X,Y,Z)
    }

    finit.close();

    bool isWellInit = false;
    while(!isWellInit)
    {
      std::vector<vpImagePoint> mem_ip;
      for(unsigned int i=0 ; i< n3d ; i++)
      {
        std::ostringstream text;
        text << "Click on point " << i+1;
        vpDisplay::display(I);
        vpDisplay::displayText(I, 15, 10, text.str(), vpColor::red);
        for (unsigned int k=0; k<mem_ip.size(); k++) {
          vpDisplay::displayCross(I, mem_ip[k], 10, vpColor::green, 2) ;
        }
        vpDisplay::flush(I) ;

        std::cout << "Click on point " << i+1 << " ";
        double x=0,y=0;
        vpDisplay::getClick(I, ip) ;
        mem_ip.push_back(ip);
        vpDisplay::flush(I) ;
        vpPixelMeterConversion::convertPoint(cam, ip, x, y);
        P[i].set_x(x);
        P[i].set_y(y);

        std::cout << "with 2D coordinates: " << ip << std::endl;

        pose.addPoint(P[i]) ; // and added to the pose computation point list
      }
      vpDisplay::flush(I) ;
      vpDisplay::display(I) ;

      vpHomogeneousMatrix cMo1, cMo2;
      pose.computePose(vpPose::LAGRANGE, cMo1) ;
      double d1 = pose.computeResidual(cMo1);
      pose.computePose(vpPose::DEMENTHON, cMo2) ;
      double d2 = pose.computeResidual(cMo2);

      if(d1 < d2){
        cMo = cMo1;
      }
      else{
        cMo = cMo2;
      }
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      display(I, cMo, cam, vpColor::green, 1, true);
      vpDisplay::displayText(I, 15, 10,
                             "left click to validate, right click to re initialize object",
                             vpColor::red);

      vpDisplay::flush(I);

      button = vpMouseButton::button1;
      while (!vpDisplay::getClick(I, ip, button));


      if (button == vpMouseButton::button1)
      {
        isWellInit = true;
      }
      else
      {
        pose.clearPoint() ;
        vpDisplay::display(I) ;
        vpDisplay::flush(I) ;
      }
    }
    vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::red);

    delete [] P;

    //save the pose into file
    if(poseSavingFilename.empty())
      savePose(str_pose);
    else
      savePose(poseSavingFilename);

    if(d_help != NULL) {
      delete d_help;
      d_help = NULL;
    }
  }

  std::cout << "cMo : "<< std::endl << cMo <<std::endl;

  init(I);
}

/*!
  Initialise the tracker by clicking in the image on the pixels that correspond to the
  3D points whose coordinates are given in \e points3D_list.

  \param I : Input image where the user has to click.
  \param points3D_list : List of at least 4 3D points with coordinates expressed in meters in the object frame.
  \param displayFile : Path to the image used to display the help. This image may be used to show where to click.
  This functionality is only available if visp_io module is used.
*/
void vpMbTracker::initClick(const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
                            const std::string &displayFile)
{
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;
  vpDisplay *d_help = NULL;

  vpPose pose ;
  std::vector<vpPoint> P;
  for (unsigned int i=0 ; i < points3D_list.size() ; i++)
    P.push_back( vpPoint(points3D_list[i].get_oX(), points3D_list[i].get_oY(), points3D_list[i].get_oZ()) );

#ifdef VISP_HAVE_MODULE_IO
  vpImage<vpRGBa> Iref ;
  //Display window creation and initialisation
  if(vpIoTools::checkFilename(displayFile)){
    try{
      std::cout << "Load image to help initialization: " << displayFile << std::endl;
#if defined VISP_HAVE_X11
      d_help = new vpDisplayX ;
#elif defined VISP_HAVE_GDI
      d_help = new vpDisplayGDI;
#elif defined VISP_HAVE_OPENCV
      d_help = new vpDisplayOpenCV;
#endif

      vpImageIo::read(Iref, displayFile) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
      d_help->init(Iref, I.display->getWindowXPosition()+(int)I.getWidth()+80, I.display->getWindowYPosition(),
                   "Where to initialize...")  ;
      vpDisplay::display(Iref) ;
      vpDisplay::flush(Iref);
#endif
    }
    catch(...) {
      if(d_help != NULL) {
        delete d_help;
        d_help = NULL;
      }
    }
  }
#else //#ifdef VISP_HAVE_MODULE_IO
    (void)(displayFile);
#endif //#ifdef VISP_HAVE_MODULE_IO

  vpImagePoint ip;
  bool isWellInit = false;
  while(!isWellInit)
  {
    for(unsigned int i=0 ; i< points3D_list.size() ; i++)
    {
      std::cout << "Click on point " << i+1 << std::endl ;
      double x=0,y=0;
      vpDisplay::getClick(I, ip) ;
      vpDisplay::displayCross(I, ip, 5,vpColor::green) ;
      vpDisplay::flush(I) ;
      vpPixelMeterConversion::convertPoint(cam, ip, x, y);
      P[i].set_x(x);
      P[i].set_y(y);

      std::cout << "Click on point " << ip << std::endl;

      vpDisplay::displayPoint (I, ip, vpColor::green); //display target point
      pose.addPoint(P[i]) ; // and added to the pose computation point list
    }
    vpDisplay::flush(I) ;

    vpHomogeneousMatrix cMo1, cMo2;
    pose.computePose(vpPose::LAGRANGE, cMo1) ;
    double d1 = pose.computeResidual(cMo1);
    pose.computePose(vpPose::DEMENTHON, cMo2) ;
    double d2 = pose.computeResidual(cMo2);

    if(d1 < d2){
      cMo = cMo1;
    }
    else{
      cMo = cMo2;
    }
    pose.computePose(vpPose::VIRTUAL_VS, cMo);

    display(I, cMo, cam, vpColor::green, 1, true);
    vpDisplay::displayText(I, 15, 10,
        "left click to validate, right click to re initialize object",
        vpColor::red);

    vpDisplay::flush(I) ;

    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
    while (!vpDisplay::getClick(I, ip, button)) {};

    if (button == vpMouseButton::button1)
    {
      isWellInit = true;
    }
    else
    {
      pose.clearPoint() ;
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
  }

  vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::red);

  if(d_help != NULL) {
    delete d_help;
    d_help = NULL;
  }

  init(I);
}
#endif //#ifdef VISP_HAVE_MODULE_GUI

/*!
  Initialise the tracker by reading 3D point coordinates and the corresponding 2D image point coordinates
  from a file. Comments starting with # character are allowed.
  3D point coordinates are expressed in meter in the object frame with X, Y and Z values.
  2D point coordinates are expressied in pixel coordinates, with first the line and then the column of the pixel in the image.
  The structure of this file is the following.
  \code
  # 3D point coordinates
  4                 # Number of 3D points in the file (minimum is four)
  0.01 0.01 0.01    #  \
  ...               #  | 3D coordinates in meters in the object frame
  0.01 -0.01 -0.01  # /
  # corresponding 2D point coordinates
  4                 # Number of image points in the file (has to be the same as the number of 3D points)
  100 200           #  \
  ...               #  | 2D coordinates in pixel in the image
  50 10  		        #  /
  \endcode

  \param I : Input image
  \param initFile : Path to the file containing all the points.
*/
void vpMbTracker::initFromPoints( const vpImage<unsigned char>& I, const std::string& initFile )
{
  char s[FILENAME_MAX];
  std::fstream finit ;

  std::string ext = ".init";
  size_t pos = initFile.rfind(ext);

  if( pos == initFile.size()-ext.size() && pos != 0)
    sprintf(s,"%s", initFile.c_str());
  else
    sprintf(s,"%s.init", initFile.c_str());

  std::cout << "Load 2D/3D points from: " << s << std::endl ;
  finit.open(s, std::ios::in) ;
  if (finit.fail()){
    std::cout << "cannot read " << s << std::endl;
    throw vpException(vpException::ioError, "Cannot open model-based tracker init file %s", s);
  }

  //********
  // Read 3D points coordinates
  //********
  char c;
  // skip lines starting with # as comment
  finit.get(c);
  while (!finit.fail() && (c == '#')) {
    finit.ignore(256, '\n');
    finit.get(c);
  }
  finit.unget();

  unsigned int n3d;
  finit >> n3d;
  finit.ignore(256, '\n'); // skip the rest of the line
  std::cout << "Number of 3D points  " << n3d << std::endl;
  if (n3d > 100000) {
    throw vpException(vpException::badValue, "In %s file, the number of 3D points exceed the max allowed", s);
  }

  vpPoint *P = new vpPoint [n3d];
  for (unsigned int i=0 ; i < n3d ; i++){
    // skip lines starting with # as comment
    finit.get(c);
    while (!finit.fail() && (c == '#')) {
      finit.ignore(256, '\n');
      finit.get(c);
    }
    finit.unget();
    double X, Y, Z;
    finit >> X ;
    finit >> Y ;
    finit >> Z ;
    finit.ignore(256, '\n'); // skip the rest of the line

    std::cout << "Point " << i+1 << " with 3D coordinates: " << X << " " << Y << " " << Z << std::endl;
    P[i].setWorldCoordinates(X, Y, Z) ; // (X,Y,Z)
  }

  //********
  // Read 3D points coordinates
  //********
  // skip lines starting with # as comment
  finit.get(c);
  while (!finit.fail() && (c == '#')) {
    finit.ignore(256, '\n');
    finit.get(c);
  }
  finit.unget();

  unsigned int n2d;
  finit >> n2d;
  finit.ignore(256, '\n'); // skip the rest of the line
  std::cout << "Number of 2D points  " << n2d << std::endl;
  if (n2d > 100000) {
    delete [] P;
    throw vpException(vpException::badValue, "In %s file, the number of 2D points exceed the max allowed", s);
  }

  if(n3d != n2d) {
    delete [] P;
    throw vpException(vpException::badValue, "In %s file, number of 2D points %d and number of 3D points %d are not equal", s, n2d, n3d);
  }

  vpPose pose ;
  for(unsigned int i=0 ; i< n2d ; i++)
  {
    // skip lines starting with # as comment
    finit.get(c);
    while (!finit.fail() && (c == '#')) {
      finit.ignore(256, '\n');
      finit.get(c);
    }
    finit.unget();
    double u, v, x=0, y=0;
    finit >> v;
    finit >> u;
    finit.ignore(256, '\n'); // skip the rest of the line

    vpImagePoint ip(v, u);
    std::cout << "Point " << i+1 << " with 2D coordinates: " << ip << std::endl;
    vpPixelMeterConversion::convertPoint(cam, ip, x, y);
    P[i].set_x(x);
    P[i].set_y(y);
    pose.addPoint(P[i]);
  }

  finit.close();

  vpHomogeneousMatrix cMo1, cMo2;
  pose.computePose(vpPose::LAGRANGE, cMo1) ;
  double d1 = pose.computeResidual(cMo1);
  pose.computePose(vpPose::DEMENTHON, cMo2) ;
  double d2 = pose.computeResidual(cMo2);

  if(d1 < d2)
    cMo = cMo1;
  else
    cMo = cMo2;

  pose.computePose(vpPose::VIRTUAL_VS, cMo);

  delete [] P;

  init(I);
}

/*!
  Initialise the tracking with the list of image points (points2D_list) and
  the list of corresponding 3D points (object frame) (points3D_list).

  \param I : Input image
  \param points2D_list : List of image points.
  \param points3D_list : List of 3D points (object frame).
*/
void vpMbTracker::initFromPoints( const vpImage<unsigned char>& I, const std::vector<vpImagePoint> &points2D_list,
                                  const std::vector<vpPoint> &points3D_list )
{
  if(points2D_list.size() != points3D_list.size())
    vpERROR_TRACE( "vpMbTracker::initFromPoints(), Number of 2D points different to the number of 3D points." );

  size_t size = points3D_list.size();
  std::vector<vpPoint> P;
  vpPose pose ;

  for(size_t i=0 ; i< size ; i++)
  {
    P.push_back( vpPoint(points3D_list[i].get_oX(), points3D_list[i].get_oY(), points3D_list[i].get_oZ()) );
    double x=0,y=0;
    vpPixelMeterConversion::convertPoint(cam, points2D_list[i], x, y);
    P[i].set_x(x);
    P[i].set_y(y);
    pose.addPoint(P[i]);
  }

  vpHomogeneousMatrix cMo1, cMo2;
  pose.computePose(vpPose::LAGRANGE, cMo1) ;
  double d1 = pose.computeResidual(cMo1);
  pose.computePose(vpPose::DEMENTHON, cMo2) ;
  double d2 = pose.computeResidual(cMo2);

  if(d1 < d2)
    cMo = cMo1;
  else
    cMo = cMo2;

  pose.computePose(vpPose::VIRTUAL_VS, cMo);

  init(I);
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read in the file initFile.
  The structure of this file is (without the comments):
  \code
  // The six value of the pose vector
  0.0000    //  \
  0.0000    //  |
  1.0000    //  | Exemple of value for the pose vector where Z = 1 meter
  0.0000    //  |
  0.0000    //  |
  0.0000    //  /
  \endcode

  Where the three firsts lines refer to the translation and the three last to the rotation in thetaU parametrisation (see vpThetaUVector).
  \param I : Input image
  \param initFile : Path to the file containing the pose.
*/
void vpMbTracker::initFromPose(const vpImage<unsigned char>& I, const std::string &initFile)
{
  char s[FILENAME_MAX];
  std::fstream finit ;
  vpPoseVector init_pos;

  std::string ext = ".pos";
  size_t pos =  initFile.rfind(ext);

  if( pos == initFile.size()-ext.size() && pos != 0)
    sprintf(s,"%s", initFile.c_str());
  else
    sprintf(s,"%s.pos", initFile.c_str());

  finit.open(s,std::ios::in) ;
  if (finit.fail()){
    std::cout << "cannot read " << s << std::endl;
    throw vpException(vpException::ioError, "cannot read init file");
  }

  for (unsigned int i = 0; i < 6; i += 1){
    finit >> init_pos[i];
  }

  cMo.buildFrom(init_pos);

  init(I);
}

/*!
  Initialise the tracking thanks to the pose.

  \param I : Input image
  \param cMo_ : Pose matrix.
*/
void vpMbTracker::initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_)
{
  this->cMo = cMo_;
  init(I);
}

/*!
  Initialise the tracking thanks to the pose vector.

  \param I : Input image
  \param cPo : Pose vector.
*/
void vpMbTracker::initFromPose (const vpImage<unsigned char>& I, const vpPoseVector &cPo)
{
  vpHomogeneousMatrix _cMo(cPo);
  initFromPose(I, _cMo);
}

/*!
  Save the pose in the given filename

  \param filename : Path to the file used to save the pose.
*/
void vpMbTracker::savePose(const std::string &filename) const
{
  vpPoseVector init_pos;
  std::fstream finitpos ;
  char s[FILENAME_MAX];

  sprintf(s,"%s", filename.c_str());
  finitpos.open(s, std::ios::out) ;

  init_pos.buildFrom(cMo);
  finitpos << init_pos;
  finitpos.close();
}


void vpMbTracker::addPolygon(const std::vector<vpPoint>& corners, const int idFace, const std::string &polygonName,
    const bool useLod, const double minPolygonAreaThreshold, const double minLineLengthThreshold)
{
  std::vector<vpPoint> corners_without_duplicates;
  corners_without_duplicates.push_back(corners[0]);
  for (unsigned int i=0; i < corners.size()-1; i++) {
    if (std::fabs(corners[i].get_oX() - corners[i+1].get_oX()) > std::fabs(corners[i].get_oX())*std::numeric_limits<double>::epsilon()
        || std::fabs(corners[i].get_oY() - corners[i+1].get_oY()) > std::fabs(corners[i].get_oY())*std::numeric_limits<double>::epsilon()
        || std::fabs(corners[i].get_oZ() - corners[i+1].get_oZ()) > std::fabs(corners[i].get_oZ())*std::numeric_limits<double>::epsilon()) {
      corners_without_duplicates.push_back(corners[i+1]);
    }
  }

  vpMbtPolygon polygon;
  polygon.setNbPoint((unsigned int)corners_without_duplicates.size());
  polygon.setIndex((int)idFace);
  polygon.setName(polygonName);
  polygon.setLod(useLod);

//  //if(minPolygonAreaThreshold != -1.0) {
//  if(std::fabs(minPolygonAreaThreshold + 1.0) > std::fabs(minPolygonAreaThreshold)*std::numeric_limits<double>::epsilon()) {
//    polygon.setMinPolygonAreaThresh(minPolygonAreaThreshold);
//  }
//
//  //if(minLineLengthThreshold != -1.0) {
//  if(std::fabs(minLineLengthThreshold + 1.0) > std::fabs(minLineLengthThreshold)*std::numeric_limits<double>::epsilon()) {
//    polygon.setMinLineLengthThresh(minLineLengthThreshold);
//  }

  polygon.setMinPolygonAreaThresh(minPolygonAreaThreshold);
  polygon.setMinLineLengthThresh(minLineLengthThreshold);

  for(unsigned int j = 0; j < corners_without_duplicates.size(); j++) {
    polygon.addPoint(j, corners_without_duplicates[j]);
  }

  faces.addPolygon(&polygon);

  if(clippingFlag != vpPolygon3D::NO_CLIPPING)
    faces.getPolygon().back()->setClipping(clippingFlag);

  if((clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
    faces.getPolygon().back()->setNearClippingDistance(distNearClip);

  if((clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
    faces.getPolygon().back()->setFarClippingDistance(distFarClip);
}

void vpMbTracker::addPolygon(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
    const int idFace, const std::string &polygonName, const bool useLod, const double minPolygonAreaThreshold)
{
    vpMbtPolygon polygon;
    polygon.setNbPoint(4);
    polygon.setName(polygonName);
    polygon.setLod(useLod);

//    //if(minPolygonAreaThreshold != -1.0) {
//    if(std::fabs(minPolygonAreaThreshold + 1.0) > std::fabs(minPolygonAreaThreshold)*std::numeric_limits<double>::epsilon()) {
//      polygon.setMinPolygonAreaThresh(minPolygonAreaThreshold);
//    }
    polygon.setMinPolygonAreaThresh(minPolygonAreaThreshold);
    //Non sense to set minLineLengthThreshold for circle
    //but used to be coherent when applying LOD settings for all polygons
    polygon.setMinLineLengthThresh(minLineLengthThresholdGeneral);

    {
      // Create the 4 points of the circle bounding box
      vpPlane plane(p1, p2, p3, vpPlane::object_frame);

      // Matrice de passage entre world et circle frame
      double norm_X = sqrt(vpMath::sqr(p2.get_oX()-p1.get_oX())
                           + vpMath::sqr(p2.get_oY()-p1.get_oY())
                           + vpMath::sqr(p2.get_oZ()-p1.get_oZ()));
      double norm_Y = sqrt(vpMath::sqr(plane.getA())
                           + vpMath::sqr(plane.getB())
                           + vpMath::sqr(plane.getC()));
      vpRotationMatrix wRc;
      vpColVector x(3),y(3),z(3);
      // X axis is P2-P1
      x[0] = (p2.get_oX()-p1.get_oX()) / norm_X;
      x[1] = (p2.get_oY()-p1.get_oY()) / norm_X;
      x[2] = (p2.get_oZ()-p1.get_oZ()) / norm_X;
      // Y axis is the normal of the plane
      y[0] = plane.getA() / norm_Y;
      y[1] = plane.getB() / norm_Y;
      y[2] = plane.getC() / norm_Y;
      // Z axis = X ^ Y
      z = vpColVector::crossProd(x, y);
      for (unsigned int i=0; i< 3; i++) {
        wRc[i][0] = x[i];
        wRc[i][1] = y[i];
        wRc[i][2] = z[i];
      }

      vpTranslationVector wtc(p1.get_oX(), p1.get_oY(), p1.get_oZ());
      vpHomogeneousMatrix wMc(wtc, wRc);

      vpColVector c_p(4); // A point in the circle frame that is on the bbox
      c_p[0] = radius;
      c_p[1] = 0;
      c_p[2] = radius;
      c_p[3] = 1;

      // Matrix to rotate a point by 90 deg around Y in the circle frame
      for(unsigned int i=0; i<4; i++) {
        vpColVector w_p(4); // A point in the word frame
        vpHomogeneousMatrix cMc_90(vpTranslationVector(), vpRotationMatrix(0,vpMath::rad(90*i), 0));
        w_p = wMc * cMc_90 * c_p;

        vpPoint w_P;
        w_P.setWorldCoordinates(w_p[0], w_p[1], w_p[2]);

        polygon.addPoint(i,w_P);
      }
    }

    polygon.setIndex(idFace);
    faces.addPolygon(&polygon);

    if(clippingFlag != vpPolygon3D::NO_CLIPPING)
      faces.getPolygon().back()->setClipping(clippingFlag);

    if((clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
      faces.getPolygon().back()->setNearClippingDistance(distNearClip);

    if((clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
      faces.getPolygon().back()->setFarClippingDistance(distFarClip);
}

void vpMbTracker::addPolygon(const vpPoint& p1, const vpPoint &p2, const int idFace, const std::string &polygonName,
    const bool useLod, const double minLineLengthThreshold)
{
  // A polygon as a single line that corresponds to the revolution axis of the cylinder
  vpMbtPolygon polygon;
  polygon.setNbPoint(2);

  polygon.addPoint(0, p1);
  polygon.addPoint(1, p2);

  polygon.setIndex(idFace) ;
  polygon.setName(polygonName);
  polygon.setLod(useLod);

//  //if(minLineLengthThreshold != -1.0) {
//  if(std::fabs(minLineLengthThreshold + 1.0) > std::fabs(minLineLengthThreshold)*std::numeric_limits<double>::epsilon()) {
//    polygon.setMinLineLengthThresh(minLineLengthThreshold);
//  }
  polygon.setMinLineLengthThresh(minLineLengthThreshold);
  //Non sense to set minPolygonAreaThreshold for cylinder
  //but used to be coherent when applying LOD settings for all polygons
  polygon.setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);

  faces.addPolygon(&polygon) ;

  if(clippingFlag != vpPolygon3D::NO_CLIPPING)
    faces.getPolygon().back()->setClipping(clippingFlag);

  if((clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
    faces.getPolygon().back()->setNearClippingDistance(distNearClip);

  if((clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
    faces.getPolygon().back()->setFarClippingDistance(distFarClip);
}

void vpMbTracker::addPolygon(const std::vector<std::vector<vpPoint> > &listFaces, const int idFace, const std::string &polygonName,
    const bool useLod, const double minLineLengthThreshold)
{
    int id = idFace;
    for(unsigned int i = 0 ; i < listFaces.size() ; i++)
    {
        vpMbtPolygon polygon;
        polygon.setNbPoint((unsigned int)listFaces[i].size());
        for(unsigned int j = 0 ; j < listFaces[i].size() ; j++)
            polygon.addPoint(j, listFaces[i][j]);

        polygon.setIndex(id) ;
        polygon.setName(polygonName);
        polygon.setIsPolygonOriented(false);
        polygon.setLod(useLod);
        polygon.setMinLineLengthThresh(minLineLengthThreshold);
        polygon.setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);

        faces.addPolygon(&polygon) ;

        if(clippingFlag != vpPolygon3D::NO_CLIPPING)
          faces.getPolygon().back()->setClipping(clippingFlag);

        if((clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
          faces.getPolygon().back()->setNearClippingDistance(distNearClip);

        if((clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
          faces.getPolygon().back()->setFarClippingDistance(distFarClip);

        id++;
    }
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the
  loadCAOModel() method.

  \warning When this class is called to load a vrml model, remember that you
  have to call Call SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao.

  \param modelFile : the file containing the the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void
vpMbTracker::loadModel(const char *modelFile, const bool verbose)
{
  loadModel( std::string(modelFile), verbose );
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the
  loadCAOModel() method.

  \warning When this class is called to load a vrml model, remember that you
  have to call Call SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao.

  \param modelFile : the file containing the the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void
vpMbTracker::loadModel(const std::string& modelFile, const bool verbose)
{
  std::string::const_iterator it;

  if(vpIoTools::checkFilename(modelFile)) {
    it = modelFile.end();
    if((*(it-1) == 'o' && *(it-2) == 'a' && *(it-3) == 'c' && *(it-4) == '.') ||
       (*(it-1) == 'O' && *(it-2) == 'A' && *(it-3) == 'C' && *(it-4) == '.') ){
      std::vector<std::string> vectorOfModelFilename;
      int startIdFace = (int)faces.size();
      nbPoints = 0;
      nbLines = 0;
      nbPolygonLines = 0;
      nbPolygonPoints = 0;
      nbCylinders = 0;
      nbCircles = 0;
      loadCAOModel(modelFile, vectorOfModelFilename, startIdFace, verbose, true);
    }
    else if((*(it-1) == 'l' && *(it-2) == 'r' && *(it-3) == 'w' && *(it-4) == '.') ||
            (*(it-1) == 'L' && *(it-2) == 'R' && *(it-3) == 'W' && *(it-4) == '.') ){
      loadVRMLModel(modelFile);
    }
    else{
      throw vpException(vpException::ioError, "Error: File %s doesn't contain a cao or wrl model", modelFile.c_str());
    }
  }
  else{
    throw vpException(vpException::ioError, "Error: File %s doesn't exist", modelFile.c_str());
  }

  this->modelInitialised = true;
  this->modelFileName = modelFile;
}


/*!
  Load the 3D model of the object from a vrml file. Only LineSet and FaceSet are
  extracted from the vrml file.

  \warning When this class is called, remember that you have to call Call
  SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
  SoDB::finish();
#endif
}
  \endcode

  \warning The cylinders extracted using this method do not use the Cylinder
  keyword of vrml since vrml exporter such as Blender or AC3D consider a
  cylinder as an IndexedFaceSet. To test whether an indexedFaceSet is a cylinder
  or not, the name of the geometry is read. If the name begins with "cyl" then
  the faceset is supposed to be a cylinder. For example, the line
  \code
geometry DEF cyl_cylinder1 IndexedFaceSet
  \endcode
  defines a cylinder named cyl_cylinder1.

  \throw vpException::fatalError if the file cannot be open.

  \param modelFile : The full name of the file containing the 3D model.
*/
void
vpMbTracker::loadVRMLModel(const std::string& modelFile)
{
#ifdef VISP_HAVE_COIN3D
  SoDB::init(); // Call SoDB::finish() before ending the program.

  SoInput in;
  SbBool ok = in.openFile(modelFile.c_str());
  SoVRMLGroup  *sceneGraphVRML2;

  if (!ok) {
    vpERROR_TRACE("can't open file to load model");
    throw vpException(vpException::fatalError, "can't open file to load model");
  }

  if(!in.isFileVRML2())
  {
    SoSeparator  *sceneGraph = SoDB::readAll(&in);
    if (sceneGraph == NULL) { /*return -1;*/ }
    sceneGraph->ref();

    SoToVRML2Action tovrml2;
    tovrml2.apply(sceneGraph);

    sceneGraphVRML2 =tovrml2.getVRML2SceneGraph();
    sceneGraphVRML2->ref();
    sceneGraph->unref();
  }
  else
  {
    sceneGraphVRML2	= SoDB::readAllVRML(&in);
    if (sceneGraphVRML2 == NULL) { /*return -1;*/ }
    sceneGraphVRML2->ref();
  }

  in.closeFile();

  vpHomogeneousMatrix transform;
  int indexFace = (int)faces.size();
  extractGroup(sceneGraphVRML2, transform, indexFace);

  sceneGraphVRML2->unref();
#else
  vpERROR_TRACE("coin not detected with ViSP, cannot load model : %s", modelFile.c_str());
  throw vpException(vpException::fatalError, "coin not detected with ViSP, cannot load model");
#endif
}

void vpMbTracker::removeComment(std::ifstream& fileId) {
  char c;

  fileId.get(c);
  while (!fileId.fail() && (c == '#')) {
    fileId.ignore(256, '\n');
    fileId.get(c);
  }
  if (fileId.fail()) {
    throw(vpException(vpException::ioError, "Reached end of file"));
  }
  fileId.unget();
}

std::map<std::string, std::string> vpMbTracker::parseParameters(std::string& endLine) {
  std::map<std::string, std::string> mapOfParams;

  bool exit = false;
  while (!endLine.empty() && !exit) {
    exit = true;

    for (std::map<std::string, std::string>::const_iterator it =
        mapOfParameterNames.begin(); it != mapOfParameterNames.end();
        ++it) {
      endLine = trim(endLine);
      //          std::cout << "endLine=" << endLine << std::endl;
      std::string param(it->first + "=");

      //Compare with a potential parameter
      if (endLine.compare(0, param.size(), param) == 0) {
        exit = false;
        endLine = endLine.substr(param.size());

        bool parseQuote = false;
        if (it->second == "string") {
          //Check if the string is between quotes
          if (endLine.size() > 2 && endLine[0] == '"') {
            parseQuote = true;
            endLine = endLine.substr(1);
            size_t pos = endLine.find_first_of('"');

            if (pos != std::string::npos) {
              mapOfParams[it->first] = endLine.substr(0, pos);
              endLine = endLine.substr(pos + 1);
            } else {
              parseQuote = false;
            }
          }
        }

        if (!parseQuote) {
          //Deal with space or tabulation after parameter value to substring
          // to the next sequence
          size_t pos1 = endLine.find_first_of(' ');
          size_t pos2 = endLine.find_first_of('\t');
          size_t pos = pos1 < pos2 ? pos1 : pos2;

          mapOfParams[it->first] = endLine.substr(0, pos);
          endLine = endLine.substr(pos + 1);
        }
      }
    }
  }

  return mapOfParams;
}

/*!
  Load a 3D model contained in a *.cao file.

  Since ViSP 2.9.1, lines starting with # character are considered as comments.
  It is also possible to add comment at the end of the lines. No specific character is requested before the comment.
  In the following example we use "//" but it could be an other character.

  Since ViSP 2.9.1, circles are supported.

  The structure of the file is :
  \code
  V1
  # Number of points describing the object
  8
  0.01 0.01 0.01  // point with index 0 \
  ...             // ...                 | coordinates of the points in the object frame (in m.)
  0.01 0.01 0.01  // point with index 7 /
  # Number of lines to track.
  3
  0 2 // line with index 0 \
  1 4 //                    | Index of the points representing the extremities of the lines
  1 5 // line with index 2 /
  # Number of polygon (face) to track using the line previously described
  1
  3 0 1 2 // Face described as follow : nbLine indexLine1 indexLine2 ... indexLineN
  # Number of polygon (face) to track using the points previously described
  3
  4 0 2 3 4 // Face described as follow : nbPoint IndexPoint1 IndexPoint2 ... IndexPointN
  4 1 3 5 7
  3 1 5 6
  # Number of cylinder
  1
  6 7 0.05 // Index of the limits points on the axis (used to know the 'height' of the cylinder) and radius of the cyclinder (in m.)
  # Number of circle
  1
  0.5 0 1 2 // radius, index center point, index 2 other points on the plane containing the circle
  \endcode

  \param modelFile : Full name of the main *.cao file containing the model.
  \param vectorOfModelFilename : A vector of *.cao files.
  \param startIdFace : Current Id of the face.
  \param verbose : If true, will print additional information with CAO model files which include other CAO model files.
  \param parent : This parameter is set to true when parsing a parent CAO model file, and false when parsing an included
  CAO model file.
*/
void
vpMbTracker::loadCAOModel(const std::string& modelFile,
                          std::vector<std::string>& vectorOfModelFilename, int& startIdFace,
                          const bool verbose, const bool parent) {
  std::ifstream fileId;
  fileId.exceptions(std::ifstream::failbit | std::ifstream::eofbit);
  fileId.open(modelFile.c_str(), std::ifstream::in);
  if (fileId.fail()) {
    std::cout << "cannot read CAO model file: " << modelFile << std::endl;
    throw vpException(vpException::ioError, "cannot read CAO model file");
  }

  if(verbose) {
      std::cout << "Model file : " << modelFile << std::endl;
  }
  vectorOfModelFilename.push_back(modelFile);

  try {
    char c;
    // Extraction of the version (remove empty line and commented ones (comment
    // line begin with the #)).
    //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
    removeComment(fileId);

    //////////////////////////Read CAO Version (V1, V2, ...)//////////////////////////
    int caoVersion;
    fileId.get(c);
    if (c == 'V') {
      fileId >> caoVersion;
      fileId.ignore(256, '\n'); // skip the rest of the line
    } else {
      std::cout
          << "in vpMbTracker::loadCAOModel() -> Bad parameter header file : use V0, V1, ...";
      throw vpException(vpException::badValue,
                        "in vpMbTracker::loadCAOModel() -> Bad parameter header file : use V0, V1, ...");
    }

    removeComment(fileId);


    //////////////////////////Read the header part if present//////////////////////////
    std::string line;
    std::string prefix = "load";

    fileId.get(c);
    fileId.unget();
    bool header = false;
    while(c == 'l' || c == 'L') {
      header = true;

      getline(fileId, line);
      if(!line.compare(0, prefix.size(), prefix)) {

        //Get the loaded model pathname
        std::string headerPathRead = line.substr(6);
        size_t firstIndex = headerPathRead.find_first_of("\")");
        headerPathRead = headerPathRead.substr(0, firstIndex);

        std::string headerPath = headerPathRead;
        if(!vpIoTools::isAbsolutePathname(headerPathRead)) {
          std::string parentDirectory = vpIoTools::getParent(modelFile);
          headerPath = vpIoTools::createFilePath(parentDirectory, headerPathRead);
        }

        //Normalize path
        headerPath = vpIoTools::path(headerPath);

        //Get real path
        headerPath = vpIoTools::getAbsolutePathname(headerPath);

        bool cyclic = false;
        for (std::vector<std::string>::const_iterator it = vectorOfModelFilename.begin();
             it != vectorOfModelFilename.end() && !cyclic; ++it) {
          if (headerPath == *it) {
            cyclic = true;
          }
        }

        if (!cyclic) {
          if (vpIoTools::checkFilename(headerPath)) {
            loadCAOModel(headerPath, vectorOfModelFilename, startIdFace, verbose, false);
          } else {
            throw vpException(vpException::ioError, "file cannot be open");
          }
        } else {
          std::cout << "WARNING Cyclic dependency detected with file "
                    << headerPath << " declared in " << modelFile << std::endl;
        }
      } else {
        header = false;
      }

      removeComment(fileId);
      fileId.get(c);
      fileId.unget();
    }


    //////////////////////////Read the point declaration part//////////////////////////
    unsigned int caoNbrPoint;
    fileId >> caoNbrPoint;
    fileId.ignore(256, '\n'); // skip the rest of the line

    nbPoints += caoNbrPoint;
    if(verbose || vectorOfModelFilename.size() == 1) {
      std::cout << "> " << caoNbrPoint << " points" << std::endl;
    }

    if (caoNbrPoint > 100000) {
      throw vpException(vpException::badValue,
                        "Exceed the max number of points in the CAO model.");
    }

    if (caoNbrPoint == 0 && !header) {
      throw vpException(vpException::badValue,
                        "in vpMbTracker::loadCAOModel() -> no points are defined");
    }
    vpPoint *caoPoints = new vpPoint[caoNbrPoint];

    double x; // 3D coordinates
    double y;
    double z;

    int i;    // image coordinate (used for matching)
    int j;

    for (unsigned int k = 0; k < caoNbrPoint; k++) {
      removeComment(fileId);

      fileId >> x;
      fileId >> y;
      fileId >> z;

      if (caoVersion == 2) {
        fileId >> i;
        fileId >> j;
      }

      fileId.ignore(256, '\n'); // skip the rest of the line

      caoPoints[k].setWorldCoordinates(x, y, z);
    }


    removeComment(fileId);


    //////////////////////////Read the segment declaration part//////////////////////////
    //Store in a map the potential segments to add
    std::map<std::pair<unsigned int, unsigned int>, SegmentInfo > segmentTemporaryMap;
    unsigned int caoNbrLine;
    fileId >> caoNbrLine;
    fileId.ignore(256, '\n'); // skip the rest of the line

    nbLines += caoNbrLine;
    unsigned int *caoLinePoints = NULL;
    if(verbose || vectorOfModelFilename.size() == 1) {
      std::cout << "> " << caoNbrLine << " lines" << std::endl;
    }

    if (caoNbrLine > 100000) {
      delete[] caoPoints;
      throw vpException(vpException::badValue,
                        "Exceed the max number of lines in the CAO model.");
    }

    if (caoNbrLine > 0)
      caoLinePoints = new unsigned int[2 * caoNbrLine];

    unsigned int index1, index2;
    //Initialization of idFace with startIdFace for dealing with recursive load in header
    int idFace = startIdFace;

    for (unsigned int k = 0; k < caoNbrLine; k++) {
      removeComment(fileId);

      fileId >> index1;
      fileId >> index2;

      //////////////////////////Read the parameter value if present//////////////////////////
      //Get the end of the line
      char buffer[256];
      fileId.getline(buffer, 256);
      std::string endLine(buffer);
      std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

      std::string segmentName = "";
      double minLineLengthThresh = !applyLodSettingInConfig ? minLineLengthThresholdGeneral : 50.0;
      bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
      if(mapOfParams.find("name") != mapOfParams.end()) {
        segmentName = mapOfParams["name"];
      }
      if(mapOfParams.find("minLineLengthThreshold") != mapOfParams.end()) {
        minLineLengthThresh = std::atof(mapOfParams["minLineLengthThreshold"].c_str());
      }
      if(mapOfParams.find("useLod") != mapOfParams.end()) {
        useLod = parseBoolean(mapOfParams["useLod"]);
      }

      SegmentInfo segmentInfo;
      segmentInfo.name = segmentName;
      segmentInfo.useLod = useLod;
      segmentInfo.minLineLengthThresh = minLineLengthThresh;

      caoLinePoints[2 * k] = index1;
      caoLinePoints[2 * k + 1] = index2;

      if (index1 < caoNbrPoint && index2 < caoNbrPoint) {
        std::vector<vpPoint> extremities;
        extremities.push_back(caoPoints[index1]);
        extremities.push_back(caoPoints[index2]);
        segmentInfo.extremities = extremities;

        std::pair<unsigned int, unsigned int> key(index1, index2);

        segmentTemporaryMap[key] = segmentInfo;
      } else {
        vpTRACE(" line %d has wrong coordinates.", k);
      }
    }

    removeComment(fileId);


    //////////////////////////Read the face segment declaration part//////////////////////////
    /* Load polygon from the lines extracted earlier (the first point of the line is used)*/
    //Store in a vector the indexes of the segments added in the face segment case
    std::vector<std::pair<unsigned int, unsigned int> > faceSegmentKeyVector;
    unsigned int caoNbrPolygonLine;
    fileId >> caoNbrPolygonLine;
    fileId.ignore(256, '\n'); // skip the rest of the line

    nbPolygonLines += caoNbrPolygonLine;
    if(verbose || vectorOfModelFilename.size() == 1) {
      std::cout << "> " << caoNbrPolygonLine << " polygon lines" << std::endl;
    }

    if (caoNbrPolygonLine > 100000) {
      delete[] caoPoints;
      delete[] caoLinePoints;
      throw vpException(vpException::badValue,
                        "Exceed the max number of polygon lines.");
    }

    unsigned int index;
    for (unsigned int k = 0; k < caoNbrPolygonLine; k++) {
      removeComment(fileId);

      unsigned int nbLinePol;
      fileId >> nbLinePol;
      std::vector<vpPoint> corners;
      if (nbLinePol > 100000) {
        throw vpException(vpException::badValue, "Exceed the max number of lines.");
      }

      for (unsigned int n = 0; n < nbLinePol; n++) {
        fileId >> index;

        if(index >= caoNbrLine) {
          throw vpException(vpException::badValue, "Exceed the max number of lines.");
        }
        corners.push_back(caoPoints[caoLinePoints[2 * index]]);
        corners.push_back(caoPoints[caoLinePoints[2 * index + 1]]);

        std::pair<unsigned int, unsigned int> key(caoLinePoints[2 * index], caoLinePoints[2 * index + 1]);
        faceSegmentKeyVector.push_back(key);
      }


      //////////////////////////Read the parameter value if present//////////////////////////
      //Get the end of the line
      char buffer[256];
      fileId.getline(buffer, 256);
      std::string endLine(buffer);
      std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

      std::string polygonName = "";
      bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
      double minPolygonAreaThreshold = !applyLodSettingInConfig ? minPolygonAreaThresholdGeneral : 2500.0;
      if(mapOfParams.find("name") != mapOfParams.end()) {
        polygonName = mapOfParams["name"];
      }
      if(mapOfParams.find("minPolygonAreaThreshold") != mapOfParams.end()) {
        minPolygonAreaThreshold = std::atof(mapOfParams["minPolygonAreaThreshold"].c_str());
      }
      if(mapOfParams.find("useLod") != mapOfParams.end()) {
        useLod = parseBoolean(mapOfParams["useLod"]);
      }

      addPolygon(corners, idFace++, polygonName, useLod, minPolygonAreaThreshold, minLineLengthThresholdGeneral);
      initFaceFromLines(*(faces.getPolygon().back())); // Init from the last polygon that was added
    }

    //Add the segments which were not already added in the face segment case
    for(std::map<std::pair<unsigned int, unsigned int>, SegmentInfo >::const_iterator it =
        segmentTemporaryMap.begin(); it != segmentTemporaryMap.end(); ++it) {
      if(std::find(faceSegmentKeyVector.begin(), faceSegmentKeyVector.end(), it->first) == faceSegmentKeyVector.end()) {
        addPolygon(it->second.extremities, idFace++, it->second.name, it->second.useLod, minPolygonAreaThresholdGeneral,
                   it->second.minLineLengthThresh);
        initFaceFromCorners(*(faces.getPolygon().back())); // Init from the last polygon that was added
      }
    }

    removeComment(fileId);


    //////////////////////////Read the face point declaration part//////////////////////////
    /* Extract the polygon using the point coordinates (top of the file) */
    unsigned int caoNbrPolygonPoint;
    fileId >> caoNbrPolygonPoint;
    fileId.ignore(256, '\n'); // skip the rest of the line

    nbPolygonPoints += caoNbrPolygonPoint;
    if(verbose || vectorOfModelFilename.size() == 1) {
      std::cout << "> " << caoNbrPolygonPoint << " polygon points"
                << std::endl;
    }

    if (caoNbrPolygonPoint > 100000) {
      throw vpException(vpException::badValue,
                        "Exceed the max number of polygon point.");
    }

    for (unsigned int k = 0; k < caoNbrPolygonPoint; k++) {
      removeComment(fileId);

      unsigned int nbPointPol;
      fileId >> nbPointPol;
      if (nbPointPol > 100000) {
        throw vpException(vpException::badValue,
                          "Exceed the max number of points.");
      }
      std::vector<vpPoint> corners;
      for (unsigned int n = 0; n < nbPointPol; n++) {
        fileId >> index;
        if (index > caoNbrPoint - 1) {
          throw vpException(vpException::badValue,
                            "Exceed the max number of points.");
        }
        corners.push_back(caoPoints[index]);
      }


      //////////////////////////Read the parameter value if present//////////////////////////
      //Get the end of the line
      char buffer[256];
      fileId.getline(buffer, 256);
      std::string endLine(buffer);
      std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

      std::string polygonName = "";
      bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
      double minPolygonAreaThreshold = !applyLodSettingInConfig ? minPolygonAreaThresholdGeneral : 2500.0;
      if(mapOfParams.find("name") != mapOfParams.end()) {
        polygonName = mapOfParams["name"];
      }
      if(mapOfParams.find("minPolygonAreaThreshold") != mapOfParams.end()) {
        minPolygonAreaThreshold = std::atof(mapOfParams["minPolygonAreaThreshold"].c_str());
      }
      if(mapOfParams.find("useLod") != mapOfParams.end()) {
        useLod = parseBoolean(mapOfParams["useLod"]);
      }


      addPolygon(corners, idFace++, polygonName, useLod, minPolygonAreaThreshold, minLineLengthThresholdGeneral);
      initFaceFromCorners(*(faces.getPolygon().back())); // Init from the last polygon that was added
    }

    //////////////////////////Read the cylinder declaration part//////////////////////////
    unsigned int caoNbCylinder;
    try {
      removeComment(fileId);

      if (fileId.eof()) { // check if not at the end of the file (for old style files)
        delete[] caoPoints;
        delete[] caoLinePoints;
        return;
      }

      /* Extract the cylinders */
      fileId >> caoNbCylinder;
      fileId.ignore(256, '\n'); // skip the rest of the line

      nbCylinders += caoNbCylinder;
      if(verbose || vectorOfModelFilename.size() == 1) {
        std::cout << "> " << caoNbCylinder << " cylinders" << std::endl;
      }

      if (caoNbCylinder > 100000) {
        throw vpException(vpException::badValue,
                          "Exceed the max number of cylinders.");
      }

      for (unsigned int k = 0; k < caoNbCylinder; ++k) {
        removeComment(fileId);

        double radius;
        unsigned int indexP1, indexP2;
        fileId >> indexP1;
        fileId >> indexP2;
        fileId >> radius;

        //////////////////////////Read the parameter value if present//////////////////////////
        //Get the end of the line
        char buffer[256];
        fileId.getline(buffer, 256);
        std::string endLine(buffer);
        std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

        std::string polygonName = "";
        bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
        double minLineLengthThreshold = !applyLodSettingInConfig ? minLineLengthThresholdGeneral : 50.0;
        if(mapOfParams.find("name") != mapOfParams.end()) {
          polygonName = mapOfParams["name"];
        }
        if(mapOfParams.find("minLineLengthThreshold") != mapOfParams.end()) {
          minLineLengthThreshold = std::atof(mapOfParams["minLineLengthThreshold"].c_str());
        }
        if(mapOfParams.find("useLod") != mapOfParams.end()) {
          useLod = parseBoolean(mapOfParams["useLod"]);
        }

        int idRevolutionAxis = idFace;
        addPolygon(caoPoints[indexP1], caoPoints[indexP2], idFace++, polygonName, useLod, minLineLengthThreshold);

        std::vector<std::vector<vpPoint> > listFaces;
        createCylinderBBox(caoPoints[indexP1], caoPoints[indexP2], radius,listFaces);
        addPolygon(listFaces, idFace, polygonName, useLod, minLineLengthThreshold);
        idFace+=4;

        initCylinder(caoPoints[indexP1], caoPoints[indexP2], radius, idRevolutionAxis, polygonName);
      }

    } catch (...) {
      std::cerr << "Cannot get the number of cylinders. Defaulting to zero."
                << std::endl;
      caoNbCylinder = 0;
    }


    //////////////////////////Read the circle declaration part//////////////////////////
    unsigned int caoNbCircle;
    try {
      removeComment(fileId);

      if (fileId.eof()) { // check if not at the end of the file (for old style files)
        delete[] caoPoints;
        delete[] caoLinePoints;
        return;
      }

      /* Extract the circles */
      fileId >> caoNbCircle;
      fileId.ignore(256, '\n'); // skip the rest of the line

      nbCircles += caoNbCircle;
      if(verbose || vectorOfModelFilename.size() == 1) {
        std::cout << "> " << caoNbCircle << " circles" << std::endl;
      }

      if (caoNbCircle > 100000) {
        throw vpException(vpException::badValue,
                          "Exceed the max number of cicles.");
      }

      for (unsigned int k = 0; k < caoNbCircle; ++k) {
        removeComment(fileId);

        double radius;
        unsigned int indexP1, indexP2, indexP3;
        fileId >> radius;
        fileId >> indexP1;
        fileId >> indexP2;
        fileId >> indexP3;

        //////////////////////////Read the parameter value if present//////////////////////////
        //Get the end of the line
        char buffer[256];
        fileId.getline(buffer, 256);
        std::string endLine(buffer);
        std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

        std::string polygonName = "";
        bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
        double minPolygonAreaThreshold = !applyLodSettingInConfig ? minPolygonAreaThresholdGeneral : 2500.0;
        if(mapOfParams.find("name") != mapOfParams.end()) {
          polygonName = mapOfParams["name"];
        }
        if(mapOfParams.find("minPolygonAreaThreshold") != mapOfParams.end()) {
          minPolygonAreaThreshold = std::atof(mapOfParams["minPolygonAreaThreshold"].c_str());
        }
        if(mapOfParams.find("useLod") != mapOfParams.end()) {
          useLod = parseBoolean(mapOfParams["useLod"]);
        }

        addPolygon(caoPoints[indexP1], caoPoints[indexP2],
                   caoPoints[indexP3], radius, idFace, polygonName, useLod, minPolygonAreaThreshold);

        initCircle(caoPoints[indexP1], caoPoints[indexP2],
                   caoPoints[indexP3], radius, idFace++, polygonName);
      }

    } catch (...) {
      std::cerr << "Cannot get the number of circles. Defaulting to zero."
                << std::endl;
      caoNbCircle = 0;
    }

    startIdFace = idFace;

    delete[] caoPoints;
    delete[] caoLinePoints;

    if(vectorOfModelFilename.size() > 1 && parent) {
      if(verbose) {
        std::cout << "Global information for " << vpIoTools::getName(modelFile) << " :" << std::endl;
        std::cout << "Total nb of points : " << nbPoints << std::endl;
        std::cout << "Total nb of lines : " << nbLines << std::endl;
        std::cout << "Total nb of polygon lines : " << nbPolygonLines << std::endl;
        std::cout << "Total nb of polygon points : " << nbPolygonPoints << std::endl;
        std::cout << "Total nb of cylinders : " << nbCylinders << std::endl;
        std::cout << "Total nb of circles : " << nbCircles << std::endl;
      } else {
        std::cout << "> " << nbPoints << " points" << std::endl;
        std::cout << "> " << nbLines << " lines" << std::endl;
        std::cout << "> " << nbPolygonLines << " polygon lines" << std::endl;
        std::cout << "> " << nbPolygonPoints << " polygon points" << std::endl;
        std::cout << "> " << nbCylinders << " cylinders" << std::endl;
        std::cout << "> " << nbCircles << " circles" << std::endl;
      }
    }
  } catch (...) {
    std::cerr << "Cannot read line!" << std::endl;
    throw vpException(vpException::ioError, "cannot read line");
  }
}

#ifdef VISP_HAVE_COIN3D
/*!
  Extract a VRML object Group.

  \param sceneGraphVRML2 : Current node (either Transform, or Group node).
  \param transform : Transformation matrix for this group.
  \param idFace : Index of the face.
*/
void
vpMbTracker::extractGroup(SoVRMLGroup *sceneGraphVRML2, vpHomogeneousMatrix &transform, int &idFace)
{
  vpHomogeneousMatrix transformCur;
  SoVRMLTransform *sceneGraphVRML2Trasnform = dynamic_cast<SoVRMLTransform *>(sceneGraphVRML2);
  if(sceneGraphVRML2Trasnform){
    float rx, ry, rz, rw;
    sceneGraphVRML2Trasnform->rotation.getValue().getValue(rx,ry,rz,rw);
    vpRotationMatrix rotMat(vpQuaternionVector(rx,ry,rz,rw));
//     std::cout << "Rotation: " << rx << " " << ry << " " << rz << " " << rw << std::endl;

    float tx, ty, tz;
    tx = sceneGraphVRML2Trasnform->translation.getValue()[0];
    ty = sceneGraphVRML2Trasnform->translation.getValue()[1];
    tz = sceneGraphVRML2Trasnform->translation.getValue()[2];
    vpTranslationVector transVec(tx,ty,tz);
//     std::cout << "Translation: " << tx << " " << ty << " " << tz << std::endl;

    float sx, sy, sz;
    sx = sceneGraphVRML2Trasnform->scale.getValue()[0];
    sy = sceneGraphVRML2Trasnform->scale.getValue()[1];
    sz = sceneGraphVRML2Trasnform->scale.getValue()[2];
//     std::cout << "Scale: " << sx << " " << sy << " " << sz << std::endl;

    for(unsigned int i = 0 ; i < 3 ; i++)
      rotMat[0][i] *= sx;
    for(unsigned int i = 0 ; i < 3 ; i++)
      rotMat[1][i] *= sy;
    for(unsigned int i = 0 ; i < 3 ; i++)
      rotMat[2][i] *= sz;

    transformCur = vpHomogeneousMatrix(transVec,rotMat);
    transform = transform * transformCur;
  }

  int nbShapes = sceneGraphVRML2->getNumChildren();
//   std::cout << sceneGraphVRML2->getTypeId().getName().getString() << std::endl;
//   std::cout << "Nb object in VRML : " << nbShapes << std::endl;

  SoNode * child;

  for (int i = 0; i < nbShapes; i++)
  {
    vpHomogeneousMatrix transform_recursive(transform);
    child = sceneGraphVRML2->getChild(i);

    if (child->getTypeId() == SoVRMLGroup::getClassTypeId()){
      extractGroup((SoVRMLGroup*)child, transform_recursive, idFace);
    }

    if (child->getTypeId() == SoVRMLTransform::getClassTypeId()){
      extractGroup((SoVRMLTransform*)child, transform_recursive, idFace);
    }

    if (child->getTypeId() == SoVRMLShape::getClassTypeId()){
      SoChildList * child2list = child->getChildren();
      std::string name = child->getName().getString();

      for (int j = 0; j < child2list->getLength(); j++)
      {
        if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedFaceSet::getClassTypeId())
        {
          SoVRMLIndexedFaceSet * face_set;
          face_set = (SoVRMLIndexedFaceSet*)child2list->get(j);
          if(!strncmp(face_set->getName().getString(),"cyl",3)){
            extractCylinders(face_set, transform, idFace, name);
          }else{
            extractFaces(face_set, transform, idFace, name);
          }
        }
        if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedLineSet::getClassTypeId())
        {
          SoVRMLIndexedLineSet * line_set;
          line_set = (SoVRMLIndexedLineSet*)child2list->get(j);
          extractLines(line_set, idFace, name);
        }
      }
    }
  }
}

/*!
  Extract a face of the object to track from the VMRL model. This method calls
  the initFaceFromCorners() method implemented in the child class.

  \param face_set : Pointer to the face in the vrml format.
  \param transform : Transformation matrix applied to the face.
  \param idFace : Face id.
  \param polygonName: Name of the polygon.
*/
void
vpMbTracker::extractFaces(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, int &idFace, const std::string &polygonName)
{
  std::vector<vpPoint> corners;
  corners.resize(0);

//  SoMFInt32 indexList = _face_set->coordIndex;
//  int indexListSize = indexList.getNum();
  int indexListSize = face_set->coordIndex.getNum();

  vpColVector pointTransformed(4);
  vpPoint pt;
  SoVRMLCoordinate *coord;

  for (int i = 0; i < indexListSize; i++)
  {
    if (face_set->coordIndex[i] == -1)
    {
      if(corners.size() > 1)
      {
        addPolygon(corners, idFace++, polygonName);
        initFaceFromCorners(*(faces.getPolygon().back())); // Init from the last polygon that was added
        corners.resize(0);
      }
    }
    else
    {
      coord = (SoVRMLCoordinate *)(face_set->coord.getValue());
      int index = face_set->coordIndex[i];
      pointTransformed[0]=coord->point[index].getValue()[0];
      pointTransformed[1]=coord->point[index].getValue()[1];
      pointTransformed[2]=coord->point[index].getValue()[2];
      pointTransformed[3] = 1.0;

      pointTransformed = transform * pointTransformed;

      pt.setWorldCoordinates(pointTransformed[0],pointTransformed[1],pointTransformed[2]);
      corners.push_back(pt);
    }
  }
}

/*!
  Extract a cylinder  to track from the VMRL model. This method calls
  the initCylinder() method implemented in the child class.

  \warning This method extract cylinder described using an indexed face set not
  a cylinder set since software such as AC3D or blender export a cylinder using
  this data type. the object name is used, if it begins with "cyl" then this
  method is called otherwise the extractFaces() is used.

  \param face_set : Pointer to the cylinder in the vrml format.
  \param transform : Transformation matrix applied to the cylinder.
  \param idFace : Id of the face.
  \param polygonName: Name of the polygon.
*/
void
vpMbTracker::extractCylinders(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, int &idFace, const std::string &polygonName)
{
  std::vector<vpPoint> corners_c1, corners_c2;//points belonging to the first circle and to the second one.
  SoVRMLCoordinate* coords = (SoVRMLCoordinate *)face_set->coord.getValue();

  unsigned int indexListSize = (unsigned int)coords->point.getNum();

  if(indexListSize % 2 == 1){
    std::cout << "Not an even number of points when extracting a cylinder." << std::endl;
    throw vpException(vpException::dimensionError, "Not an even number of points when extracting a cylinder.");
  }
  corners_c1.resize(indexListSize / 2);
  corners_c2.resize(indexListSize / 2);
  vpColVector pointTransformed(4);
  vpPoint pt;


  // extract all points and fill the two sets.

  for(int i=0; i<coords->point.getNum(); ++i){
    pointTransformed[0]=coords->point[i].getValue()[0];
    pointTransformed[1]=coords->point[i].getValue()[1];
    pointTransformed[2]=coords->point[i].getValue()[2];
    pointTransformed[3] = 1.0;

    pointTransformed = transform * pointTransformed;

    pt.setWorldCoordinates(pointTransformed[0],pointTransformed[1],pointTransformed[2]);

    if(i < (int)corners_c1.size()){
      corners_c1[(unsigned int)i] = pt;
    }else{
      corners_c2[(unsigned int)i-corners_c1.size()] = pt;
    }
  }

  vpPoint p1 = getGravityCenter(corners_c1);
  vpPoint p2 = getGravityCenter(corners_c2);

  vpColVector dist(3);
  dist[0] = p1.get_oX() - corners_c1[0].get_oX();
  dist[1] = p1.get_oY() - corners_c1[0].get_oY();
  dist[2] = p1.get_oZ() - corners_c1[0].get_oZ();
  double radius_c1 = sqrt(dist.sumSquare());
  dist[0] = p2.get_oX() - corners_c2[0].get_oX();
  dist[1] = p2.get_oY() - corners_c2[0].get_oY();
  dist[2] = p2.get_oZ() - corners_c2[0].get_oZ();
  double radius_c2 = sqrt(dist.sumSquare());

  if(std::fabs(radius_c1 - radius_c2) > (std::numeric_limits<double>::epsilon() * vpMath::maximum(radius_c1, radius_c2))){
    std::cout << "Radius from the two circles of the cylinders are different." << std::endl;
    throw vpException(vpException::badValue, "Radius from the two circles of the cylinders are different.");
  }

  //addPolygon(p1, p2, idFace, polygonName);
  //initCylinder(p1, p2, radius_c1, idFace++);

  int idRevolutionAxis = idFace;
  addPolygon(p1, p2, idFace++, polygonName);

  std::vector<std::vector<vpPoint> > listFaces;
  createCylinderBBox(p1, p2, radius_c1, listFaces);
  addPolygon(listFaces, idFace, polygonName);
  idFace+=4;

  initCylinder(p1, p2, radius_c1, idRevolutionAxis, polygonName);
}

/*!
  Extract a line of the object to track from the VMRL model. This method calls
  the initFaceFromCorners() method implemented in the child class.

  \param line_set : Pointer to the line in the vrml format.
  \param idFace : Id of the face.
  \param polygonName: Name of the polygon.
*/
void
vpMbTracker::extractLines(SoVRMLIndexedLineSet* line_set, int &idFace, const std::string &polygonName)
{
  std::vector<vpPoint> corners;
  corners.resize(0);

  int indexListSize = line_set->coordIndex.getNum();

  SbVec3f point(0,0,0);
  vpPoint pt;
  SoVRMLCoordinate *coord;

  for (int i = 0; i < indexListSize; i++)
  {
    if (line_set->coordIndex[i] == -1)
    {
      if(corners.size() > 1)
      {
        addPolygon(corners, idFace++, polygonName);
        initFaceFromCorners(*(faces.getPolygon().back())); // Init from the last polygon that was added
        corners.resize(0);
      }
    }
    else
    {
      coord = (SoVRMLCoordinate *)(line_set->coord.getValue());
      int index = line_set->coordIndex[i];
      point[0]=coord->point[index].getValue()[0];
      point[1]=coord->point[index].getValue()[1];
      point[2]=coord->point[index].getValue()[2];

      pt.setWorldCoordinates(point[0],point[1],point[2]);
      corners.push_back(pt);
    }
  }
}

#endif // VISP_HAVE_COIN3D

/*!
  Compute the center of gravity of a set of point. This is used in the cylinder
  extraction to find the center of the circles.

  \throw vpException::dimensionError if the set is empty.

  \param pts : Set of point to extract the center of gravity.
  \return Center of gravity of the set.
*/
vpPoint
vpMbTracker::getGravityCenter(const std::vector<vpPoint>& pts) const
{
  if(pts.empty()){
    std::cout << "Cannot extract center of gravity of empty set." << std::endl;
    throw vpException(vpException::dimensionError, "Cannot extract center of gravity of empty set.");
  }
  double oX = 0;
  double oY = 0;
  double oZ = 0;
  vpPoint G;

  for(unsigned int i=0; i<pts.size(); ++i){
    oX += pts[i].get_oX();
    oY += pts[i].get_oY();
    oZ += pts[i].get_oZ();
  }

  G.setWorldCoordinates(oX/pts.size(), oY/pts.size(), oZ/pts.size());
  return G;
}

/*!
  Get the list of polygons faces (a vpPolygon representing the projection of the face in the image and a list of face corners
  in 3D), with the possibility to order by distance to the camera or to use the visibility check to consider if the polygon
  face must be retrieved or not.

  \param orderPolygons : If true, the resulting list is ordered from the nearest polygon faces to the farther.
  \param useVisibility : If true, only visible faces will be retrieved.
  \param clipPolygon : If true, the polygons will be clipped according to the clipping flags set in vpMbTracker.
  \return A pair object containing the list of vpPolygon and the list of face corners.
 */
std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > >
vpMbTracker::getPolygonFaces(const bool orderPolygons, const bool useVisibility, const bool clipPolygon)
{
  //Temporary variable to permit to order polygons by distance
  std::vector<vpPolygon> polygonsTmp;
  std::vector<std::vector<vpPoint> > roisPtTmp;

  //Pair containing the list of vpPolygon and the list of face corners
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pairOfPolygonFaces;

  for (unsigned int i = 0; i < faces.getPolygon().size(); i++) {
    //A face has at least 3 points
    if (faces.getPolygon()[i]->nbpt > 2) {
      if ( (useVisibility && faces.getPolygon()[i]->isvisible) || !useVisibility ) {
        std::vector<vpImagePoint> roiPts;

        if (clipPolygon) {
          faces.getPolygon()[i]->getRoiClipped(cam, roiPts, cMo);
        } else {
          roiPts = faces.getPolygon()[i]->getRoi(cam, cMo);
        }

        if (roiPts.size() <= 2) {
          continue;
        }

        polygonsTmp.push_back(vpPolygon(roiPts));

        std::vector<vpPoint> polyPts;
        if (clipPolygon) {
          faces.getPolygon()[i]->getPolygonClipped(polyPts);
        } else {
          for (unsigned int j = 0; j < faces.getPolygon()[i]->nbpt; j++) {
            polyPts.push_back(faces.getPolygon()[i]->p[j]);
          }
        }
        roisPtTmp.push_back(polyPts);
      }
    }
  }

  if(orderPolygons) {
    //Order polygons by distance (near to far)
    std::vector<PolygonFaceInfo> listOfPolygonFaces;
    for(unsigned int i = 0; i < polygonsTmp.size(); i++) {
      double x_centroid = 0.0, y_centroid = 0.0, z_centroid = 0.0;
      for(unsigned int j = 0; j < roisPtTmp[i].size(); j++) {
        x_centroid += roisPtTmp[i][j].get_X();
        y_centroid += roisPtTmp[i][j].get_Y();
        z_centroid += roisPtTmp[i][j].get_Z();
      }

      x_centroid /= roisPtTmp[i].size();
      y_centroid /= roisPtTmp[i].size();
      z_centroid /= roisPtTmp[i].size();

      double squared_dist = x_centroid*x_centroid + y_centroid*y_centroid + z_centroid*z_centroid;
      listOfPolygonFaces.push_back(PolygonFaceInfo(squared_dist, polygonsTmp[i], roisPtTmp[i]));
    }

    //Sort the list of polygon faces
    std::sort(listOfPolygonFaces.begin(), listOfPolygonFaces.end());

    polygonsTmp.resize(listOfPolygonFaces.size());
    roisPtTmp.resize(listOfPolygonFaces.size());

    size_t cpt = 0;
    for(std::vector<PolygonFaceInfo>::const_iterator it = listOfPolygonFaces.begin(); it != listOfPolygonFaces.end();
        ++it, cpt++) {
      polygonsTmp[cpt] = it->polygon;
      roisPtTmp[cpt] = it->faceCorners;
    }

    pairOfPolygonFaces.first = polygonsTmp;
    pairOfPolygonFaces.second = roisPtTmp;
  } else {
    pairOfPolygonFaces.first = polygonsTmp;
    pairOfPolygonFaces.second = roisPtTmp;
  }

  return pairOfPolygonFaces;
}

/*!
  Use Ogre3D for visibility tests

  \warning This function has to be called before the initialization of the tracker.

  \param v : True to use it, False otherwise
*/
void
vpMbTracker::setOgreVisibilityTest(const bool &v)
{
  useOgre = v;
  if(useOgre){
#ifndef VISP_HAVE_OGRE
    useOgre = false;
    std::cout << "WARNING: ViSP doesn't have Ogre3D, basic visibility test will be used. setOgreVisibilityTest() set to false." << std::endl;
#endif
  }
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.
*/
void
vpMbTracker::setFarClippingDistance(const double &dist)
{
  if( (clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING && dist <= distNearClip)
    vpTRACE("Far clipping value cannot be inferior than near clipping value. Far clipping won't be considered.");
  else if ( dist < 0 )
    vpTRACE("Far clipping value cannot be inferior than 0. Far clipping won't be considered.");
  else{
    clippingFlag = (clippingFlag | vpPolygon3D::FAR_CLIPPING);
    distFarClip = dist;
    for (unsigned int i = 0; i < faces.size(); i ++){
      faces[i]->setFarClippingDistance(distFarClip);
    }
#ifdef VISP_HAVE_OGRE
    faces.getOgreContext()->setFarClippingDistance(distFarClip);
#endif
  }
}

/*!
  Set the flag to consider if the level of detail (LOD) is used.

  \param useLod : true if the level of detail must be used, false otherwise. When true,
  two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
  \param name : name of the face we want to modify the LOD parameter.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
 */
void
vpMbTracker::setLod(const bool useLod, const std::string &name)
{
  for (unsigned int i = 0; i < faces.size(); i++)
  {
    if(name.empty() || faces[i]->name == name) {
      faces[i]->setLod(useLod);
    }
  }
}

/*!
    Set the threshold for the minimum line length to be considered as visible in the LOD case.

    \param minLineLengthThresh : threshold for the minimum line length in pixel.
    \param name : name of the face we want to modify the LOD threshold.

    \sa setLod(), setMinPolygonAreaThresh()
 */
void
vpMbTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name)
{
  for (unsigned int i = 0; i < faces.size(); i++)
  {
    if(name.empty() || faces[i]->name == name) {
      faces[i]->setMinLineLengthThresh(minLineLengthThresh);
    }
  }
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinLineLengthThresh()
 */
void
vpMbTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name)
{
  for (unsigned int i = 0; i < faces.size(); i++)
  {
    if(name.empty() || faces[i]->name == name) {
      faces[i]->setMinPolygonAreaThresh(minPolygonAreaThresh);
    }
  }
}

/*!
  Set the near distance for clipping.

  \param dist : Near clipping value.
*/
void
vpMbTracker::setNearClippingDistance(const double &dist)
{
  if( (clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING && dist >= distFarClip)
    vpTRACE("Near clipping value cannot be superior than far clipping value. Near clipping won't be considered.");
  else if ( dist < 0 )
    vpTRACE("Near clipping value cannot be inferior than 0. Near clipping won't be considered.");
  else{
    clippingFlag = (clippingFlag | vpPolygon3D::NEAR_CLIPPING);
    distNearClip = dist;
    for (unsigned int i = 0; i < faces.size(); i ++){
      faces[i]->setNearClippingDistance(distNearClip);
    }
#ifdef VISP_HAVE_OGRE
    faces.getOgreContext()->setNearClippingDistance(distNearClip);
#endif
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags : New clipping flags.
*/
void
vpMbTracker::setClipping(const unsigned int &flags)
{
  clippingFlag = flags;
  for (unsigned int i = 0; i < faces.size(); i ++)
    faces[i]->setClipping(clippingFlag);
}

void
vpMbTracker::computeCovarianceMatrixVVS(const bool isoJoIdentity_, const vpColVector &w_true, const vpHomogeneousMatrix &cMoPrev,
                                        const vpMatrix &L_true, const vpMatrix &LVJ_true, const vpColVector &error) {
  if (computeCovariance) {
    vpMatrix D;
    D.diag(w_true);

    // Note that here the covariance is computed on cMoPrev for time computation efficiency
    if (isoJoIdentity_) {
      covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev, error, L_true, D);
    } else{
      covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev, error, LVJ_true, D);
    }
  }
}

/*!
  Compute \f$ J^T R \f$, with J the interaction matrix and R the vector of
  residu.

  \throw vpMatrixException::incorrectMatrixSizeError if the sizes of the
  matrices do not allow the computation.

  \warning The JTR vector is resized.

  \param interaction : The interaction matrix (size Nx6).
  \param error : The residu vector (size Nx1).
  \param JTR : The resulting JTR column vector (size 6x1).

*/
void
vpMbTracker::computeJTR(const vpMatrix& interaction, const vpColVector& error, vpColVector& JTR) const
{
  if(interaction.getRows() != error.getRows() || interaction.getCols() != 6 ){
    throw vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
              "Incorrect matrices size in computeJTR.");
  }

  JTR.resize(6);
  const unsigned int N = interaction.getRows();

  for (unsigned int i = 0; i < 6; i += 1){
    double ssum = 0;
    for (unsigned int j = 0; j < N; j += 1){
      ssum += interaction[j][i] * error[j];
    }
    JTR[i] = ssum;
  }
}

void
vpMbTracker::computeVVSCheckLevenbergMarquardt(const unsigned int iter, vpColVector &error, const vpColVector &m_error_prev, const vpHomogeneousMatrix &cMoPrev,
                                               double &mu, bool &reStartFromLastIncrement, vpColVector * const w, const vpColVector * const m_w_prev) {
  if (iter != 0 && m_optimizationMethod == vpMbTracker::LEVENBERG_MARQUARDT_OPT) {
    if (error.sumSquare() / (double) error.getRows() > m_error_prev.sumSquare() / (double) m_error_prev.getRows()){
      mu *= 10.0;

      if(mu > 1.0)
        throw vpTrackingException(vpTrackingException::fatalError, "Optimization diverged");

      cMo = cMoPrev;
      error = m_error_prev;
      if (w != NULL && m_w_prev != NULL) {
        *w = *m_w_prev;
      }
      reStartFromLastIncrement = true;
    }
  }
}

void
vpMbTracker::computeVVSPoseEstimation(const bool isoJoIdentity_, const unsigned int iter, vpMatrix &L, vpMatrix &LTL, vpColVector &R,
                                      const vpColVector &error, vpColVector &error_prev, vpColVector &LTR, double &mu, vpColVector &v,
                                      const vpColVector * const w, vpColVector * const m_w_prev) {
  if (isoJoIdentity_) {
      LTL = L.AtA();
      computeJTR(L, R, LTR);

      switch (m_optimizationMethod) {
        case vpMbTracker::LEVENBERG_MARQUARDT_OPT:
          {
            vpMatrix LMA(LTL.getRows(), LTL.getCols());
            LMA.eye();
            vpMatrix LTLmuI = LTL + (LMA*mu);
            v = -m_lambda*LTLmuI.pseudoInverse(LTLmuI.getRows()*std::numeric_limits<double>::epsilon())*LTR;

            if(iter != 0)
              mu /= 10.0;

            error_prev = error;
            if (w != NULL && m_w_prev != NULL)
              *m_w_prev = *w;
            break;
          }

        case vpMbTracker::GAUSS_NEWTON_OPT:
        default:
          v = -m_lambda * LTL.pseudoInverse(LTL.getRows()*std::numeric_limits<double>::epsilon()) * LTR;
      }
  } else {
      vpVelocityTwistMatrix cVo;
      cVo.buildFrom(cMo);
      vpMatrix LVJ = (L*cVo*oJo);
      vpMatrix LVJTLVJ = (LVJ).AtA();
      vpColVector LVJTR;
      computeJTR(LVJ, R, LVJTR);

      switch (m_optimizationMethod) {
        case vpMbTracker::LEVENBERG_MARQUARDT_OPT:
          {
            vpMatrix LMA(LVJTLVJ.getRows(), LVJTLVJ.getCols());
            LMA.eye();
            vpMatrix LTLmuI = LVJTLVJ + (LMA*mu);
            v = -m_lambda*LTLmuI.pseudoInverse(LTLmuI.getRows()*std::numeric_limits<double>::epsilon())*LVJTR;
            v = cVo * v;

            if(iter != 0)
              mu /= 10.0;

            error_prev = error;
            if (w != NULL && m_w_prev != NULL)
              *m_w_prev = *w;
            break;
          }
        case vpMbTracker::GAUSS_NEWTON_OPT:
        default:
          {
            v = -m_lambda*LVJTLVJ.pseudoInverse(LVJTLVJ.getRows()*std::numeric_limits<double>::epsilon())*LVJTR;
            v = cVo * v;
            break;
          }
      }
  }
}

void
vpMbTracker::computeVVSWeights(vpRobust &robust, const vpColVector &error, vpColVector &w) {
  if (error.getRows() > 0)
    robust.MEstimator(vpRobust::TUKEY, error, w);
}

/*!
  Get a 1x6 vpColVector representing the estimated degrees of freedom.
  vpColVector[0] = 1 if translation on X is estimated, 0 otherwise;
  vpColVector[1] = 1 if translation on Y is estimated, 0 otherwise;
  vpColVector[2] = 1 if translation on Z is estimated, 0 otherwise;
  vpColVector[3] = 1 if rotation on X is estimated, 0 otherwise;
  vpColVector[4] = 1 if rotation on Y is estimated, 0 otherwise;
  vpColVector[5] = 1 if rotation on Z is estimated, 0 otherwise;

  \return 1x6 vpColVector representing the estimated degrees of freedom.
*/
vpColVector
vpMbTracker::getEstimatedDoF() const
{
    vpColVector v(6);
    for(unsigned int i = 0 ; i < 6 ; i++)
        v[i] = oJo[i][i];
    return v;
}

/*!
  Set a 6-dim column vector representing the degrees of freedom in the object frame
  that are estimated by the tracker. When set to 1, all the 6 dof are estimated.

  Below we give the correspondance between the index of the vector and the considered dof:
  - v[0] = 1 if translation along X is estimated, 0 otherwise;
  - v[1] = 1 if translation along Y is estimated, 0 otherwise;
  - v[2] = 1 if translation along Z is estimated, 0 otherwise;
  - v[3] = 1 if rotation along X is estimated, 0 otherwise;
  - v[4] = 1 if rotation along Y is estimated, 0 otherwise;
  - v[5] = 1 if rotation along Z is estimated, 0 otherwise;

*/
void
vpMbTracker::setEstimatedDoF(const vpColVector& v)
{
    if(v.getRows() == 6)
    {
        isoJoIdentity = true;
        for(unsigned int i = 0 ; i < 6 ; i++){
            // if(v[i] != 0){
            if(std::fabs(v[i]) > std::numeric_limits<double>::epsilon()){
              oJo[i][i] = 1.0;
            }
            else{
              oJo[i][i] = 0.0;
              isoJoIdentity = false;
            }
        }
    }
}


void
vpMbTracker::createCylinderBBox(const vpPoint& p1, const vpPoint &p2, const double &radius, std::vector<std::vector<vpPoint> > &listFaces)
{
    listFaces.clear();

//    std::vector<vpPoint> revolutionAxis;
//    revolutionAxis.push_back(p1);
//    revolutionAxis.push_back(p2);
//    listFaces.push_back(revolutionAxis);

    vpColVector axis(3);
    axis[0] = p1.get_oX() - p2.get_oX();
    axis[1] = p1.get_oY() - p2.get_oY();
    axis[2] = p1.get_oZ() - p2.get_oZ();

    vpColVector randomVec(3);
    randomVec = 0;

    vpColVector axisOrtho(3);

    randomVec[0] = 1.0;
    axisOrtho = vpColVector::crossProd(axis, randomVec);

    if(axisOrtho.euclideanNorm() < std::numeric_limits<double>::epsilon())
    {
        randomVec = 0;
        randomVec[1] = 1.0;
        axisOrtho = vpColVector::crossProd(axis, randomVec);
        if(axisOrtho.euclideanNorm() < std::numeric_limits<double>::epsilon())
        {
            randomVec = 0;
            randomVec[2] = 1.0;
            axisOrtho = vpColVector::crossProd(axis, randomVec);
            if(axisOrtho.euclideanNorm() < std::numeric_limits<double>::epsilon())
                throw vpMatrixException(vpMatrixException::badValue, "Problem in the cylinder definition");
        }
    }

    axisOrtho.normalize();

    vpColVector axisOrthoBis(3);
    axisOrthoBis = vpColVector::crossProd(axis, axisOrtho);
    axisOrthoBis.normalize();

    //First circle
    vpColVector p1Vec(3);
    p1Vec[0] = p1.get_oX();
    p1Vec[1] = p1.get_oY();
    p1Vec[2] = p1.get_oZ();
    vpColVector fc1 = p1Vec + axisOrtho*radius;
    vpColVector fc2 = p1Vec + axisOrthoBis*radius;
    vpColVector fc3 = p1Vec - axisOrtho*radius;
    vpColVector fc4 = p1Vec - axisOrthoBis*radius;

    vpColVector p2Vec(3);
    p2Vec[0] = p2.get_oX();
    p2Vec[1] = p2.get_oY();
    p2Vec[2] = p2.get_oZ();
    vpColVector sc1 = p2Vec + axisOrtho*radius;
    vpColVector sc2 = p2Vec + axisOrthoBis*radius;
    vpColVector sc3 = p2Vec - axisOrtho*radius;
    vpColVector sc4 = p2Vec - axisOrthoBis*radius;

    std::vector<vpPoint> pointsFace;
    pointsFace.push_back( vpPoint(fc1[0], fc1[1], fc1[2]) );
    pointsFace.push_back( vpPoint(sc1[0], sc1[1], sc1[2]) );
    pointsFace.push_back( vpPoint(sc2[0], sc2[1], sc2[2]) );
    pointsFace.push_back( vpPoint(fc2[0], fc2[1], fc2[2]) );
    listFaces.push_back(pointsFace);

    pointsFace.clear();
    pointsFace.push_back( vpPoint(fc2[0], fc2[1], fc2[2]) );
    pointsFace.push_back( vpPoint(sc2[0], sc2[1], sc2[2]) );
    pointsFace.push_back( vpPoint(sc3[0], sc3[1], sc3[2]) );
    pointsFace.push_back( vpPoint(fc3[0], fc3[1], fc3[2]) );
    listFaces.push_back(pointsFace);

    pointsFace.clear();
    pointsFace.push_back( vpPoint(fc3[0], fc3[1], fc3[2]) );
    pointsFace.push_back( vpPoint(sc3[0], sc3[1], sc3[2]) );
    pointsFace.push_back( vpPoint(sc4[0], sc4[1], sc4[2]) );
    pointsFace.push_back( vpPoint(fc4[0], fc4[1], fc4[2]) );
    listFaces.push_back(pointsFace);

    pointsFace.clear();
    pointsFace.push_back( vpPoint(fc4[0], fc4[1], fc4[2]) );
    pointsFace.push_back( vpPoint(sc4[0], sc4[1], sc4[2]) );
    pointsFace.push_back( vpPoint(sc1[0], sc1[1], sc1[2]) );
    pointsFace.push_back( vpPoint(fc1[0], fc1[1], fc1[2]) );
    listFaces.push_back(pointsFace);
}

