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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>

/*!
 \file vpMbtDistanceLine.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <visp3/mbt/vpMbtDistanceLine.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <stdlib.h>

void buildPlane(vpPoint &P, vpPoint &Q, vpPoint &R, vpPlane &plane);
void buildLine(vpPoint &P1, vpPoint &P2, vpPoint &P3, vpPoint &P4, vpLine &L);

/*!
  Basic constructor
*/
vpMbtDistanceLine::vpMbtDistanceLine()
  : name(), index(0), cam(), me(NULL), isTrackedLine(true), isTrackedLineWithVisibility(true),
    wmean(1), featureline(), poly(), useScanLine(false), meline(), line(NULL), p1(NULL), p2(NULL), L(),
    error(), nbFeature(), nbFeatureTotal(0), Reinit(false), hiddenface(NULL), Lindex_polygon(),
    Lindex_polygon_tracked(), isvisible(false)
{
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtDistanceLine::~vpMbtDistanceLine()
{
//	cout << "Deleting line " << index << endl ;
  if (line != NULL) delete line ;

  for(unsigned int i = 0 ; i < meline.size() ; i++)
    if (meline[i] != NULL) delete meline[i] ;

  meline.clear();
}

/*!
  Project the line and the two points corresponding to its extremities into the image.
  
  \param cMo : The pose of the camera used to project the line into the image.
*/
void
vpMbtDistanceLine::project(const vpHomogeneousMatrix &cMo)
{
  line->project(cMo) ;
  p1->project(cMo) ;
  p2->project(cMo) ;
}

/*!
  Build a 3D plane thanks to 3 points and stores it in \f$ plane \f$.
  
  \param P : The first point to define the plane
  \param Q : The second point to define the plane
  \param R : The third point to define the plane
  \param plane : The vpPlane instance used to store the computed plane equation.
*/
void
buildPlane(vpPoint &P, vpPoint &Q, vpPoint &R, vpPlane &plane)
{
  vpColVector a(3);
  vpColVector b(3);
  vpColVector n(3);
  //Calculate vector corresponding to PQ
  a[0]=P.get_oX()-Q.get_oX();
  a[1]=P.get_oY()-Q.get_oY();
  a[2]=P.get_oZ()-Q.get_oZ();

  //Calculate vector corresponding to PR
  b[0]=P.get_oX()-R.get_oX();
  b[1]=P.get_oY()-R.get_oY();
  b[2]=P.get_oZ()-R.get_oZ();

  //Calculate normal vector to plane PQ x PR
  n=vpColVector::cross(a,b);

  //Equation of the plane is given by:
  double A = n[0];
  double B = n[1];
  double C = n[2];
  double D=-(A*P.get_oX()+B*P.get_oY()+C*P.get_oZ());

  double norm =  sqrt(A*A+B*B+C*C) ;
  plane.setA(A/norm) ;
  plane.setB(B/norm) ;
  plane.setC(C/norm) ;
  plane.setD(D/norm) ;
}


/*!
  Build a line thanks to 4 points.
  
  The method is the following : Two plane are computed thanks to (P1,P2,P3) and (P1,P2,P4) (see the buildPlane method). Then the line equation is computed thanks to the intersection between the two planes.
  
  \param P1 : The first point to compute the line.
  \param P2 : The second point to compute the line.
  \param P3 : The third point to compute the line.
  \param P4 : The fourth point to compute the line.
  \param L : The instance of vpLine to store the computed line equation.
*/
void
buildLine(vpPoint &P1, vpPoint &P2, vpPoint &P3, vpPoint &P4, vpLine &L)
{
  vpPlane plane1;
  vpPlane plane2 ;
  buildPlane(P1,P2,P3,plane1) ;
  buildPlane(P1,P2,P4,plane2) ;

  L.setWorldCoordinates(plane1.getA(),plane1.getB(), plane1.getC(),plane1.getD(),
			plane2.getA(),plane2.getB(), plane2.getC(),plane2.getD()) ;
}


/*!
  Build a vpMbtDistanceLine thanks to two points corresponding to the extremities.
  
  \param _p1 : The first extremity.
  \param _p2 : The second extremity.
*/
void
vpMbtDistanceLine::buildFrom(vpPoint &_p1, vpPoint &_p2)
{
  line = new vpLine ;
  poly.setNbPoint(2);
  poly.addPoint(0, _p1);
  poly.addPoint(1, _p2);
  
  p1 = &poly.p[0];
  p2 = &poly.p[1];

  vpColVector V1(3);
  vpColVector V2(3);
  vpColVector V3(3);
  vpColVector V4(3);

  V1[0] = p1->get_oX();
  V1[1] = p1->get_oY();
  V1[2] = p1->get_oZ();
  V2[0] = p2->get_oX();
  V2[1] = p2->get_oY();
  V2[2] = p2->get_oZ();

  //if((V1-V2).sumSquare()!=0)
  if(std::fabs((V1-V2).sumSquare()) > std::numeric_limits<double>::epsilon())
  {
    {
      V3[0]=double(rand()%1000)/100;
      V3[1]=double(rand()%1000)/100;
      V3[2]=double(rand()%1000)/100;


      vpColVector v_tmp1,v_tmp2;
      v_tmp1 = V2-V1;
      v_tmp2 = V3-V1;
      V4=vpColVector::cross(v_tmp1,v_tmp2);
    }
    
    vpPoint P3(V3[0],V3[1],V3[2]);
    vpPoint P4(V4[0],V4[1],V4[2]);
    buildLine(*p1,*p2, P3,P4, *line) ;
  }
  else
  {
    vpPoint P3(V1[0],V1[1],V1[2]);
    vpPoint P4(V2[0],V2[1],V2[2]);
    buildLine(*p1,*p2,P3,P4,*line) ;
  }
}

/*!
  Add a polygon to the list of polygons the line belongs to.

  \param idx : Index of the polygon
*/
void
vpMbtDistanceLine::addPolygon(const int &idx)
{
  Lindex_polygon.push_back(idx);
  Lindex_polygon_tracked.push_back(true);
}

/*!
  Set if the line has to considered during tracking phase.
  Line won't be considered if all of its polygons are desactivated.

  \param polyname : name of the polygons that have to be modified.
  \param track : True if the polygon has to be tracked, False otherwise.
*/
void
vpMbtDistanceLine::setTracked(const std::string &polyname, const bool &track)
{
  unsigned int ind = 0;
  for(std::list<int>::const_iterator itpoly=Lindex_polygon.begin(); itpoly!=Lindex_polygon.end(); ++itpoly){
    if((*hiddenface)[(unsigned)(*itpoly)]->getName() == polyname){
      Lindex_polygon_tracked[ind] = track;
    }
    ind++;
  }

  isTrackedLine = false;
  for(unsigned int i = 0 ; i < Lindex_polygon_tracked.size() ; i++)
    if(Lindex_polygon_tracked[i])
    {
      isTrackedLine = true;
      break;
    }

  if(!isTrackedLine){
    isTrackedLineWithVisibility = false;
    return;
  }

  updateTracked();
}

/*!
  Update the boolean specifying if the line has to be tracked.
  It takes into account the desactivated polygons and the visibility of the others.
*/
void
vpMbtDistanceLine::updateTracked()
{
  if(!isTrackedLine){
    isTrackedLineWithVisibility = false;
    return;
  }

  unsigned int ind = 0;
  isTrackedLineWithVisibility = false;
  for(std::list<int>::const_iterator itpoly=Lindex_polygon.begin(); itpoly!=Lindex_polygon.end(); ++itpoly){
    if((*hiddenface)[(unsigned)(*itpoly)]->isVisible() && Lindex_polygon_tracked[ind]){
      isTrackedLineWithVisibility = true;
      break;
    }
    ind++;
  }
}

/*! 
  Set the moving edge parameters.
  
  \param _me : an instance of vpMe containing all the desired parameters
*/
void
vpMbtDistanceLine::setMovingEdge(vpMe *_me)
{
  me = _me ;

  for(unsigned int i = 0 ; i < meline.size() ; i++)
    if (meline[i] != NULL)
    {
//      nbFeature[i] = 0;
      meline[i]->reset();
      meline[i]->setMe(me) ;
    }

//  nbFeatureTotal = 0;
}


/*!
  Initialize the moving edge thanks to a given pose of the camera.                          
  The 3D model is projected into the image to create moving edges along the line.
  
  \param I : The image.
  \param cMo : The pose of the camera used to initialize the moving edges.
  \return false if an error occur, true otherwise.
*/
bool
vpMbtDistanceLine::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  for(unsigned int i = 0 ; i < meline.size() ; i++){
    if (meline[i] != NULL) delete meline[i] ;
  }

  meline.clear();
  nbFeature.clear();
  nbFeatureTotal = 0;

  if(isvisible)
  {
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);

    if(poly.getClipping() > 3) // Contains at least one FOV constraint
      cam.computeFov(I.getWidth(), I.getHeight());

    poly.computePolygonClipped(cam);

    if(poly.polyClipped.size() == 2){ //Les points sont visibles.

      std::vector<std::pair<vpPoint, vpPoint> > linesLst;

      if(useScanLine){
        hiddenface->computeScanLineQuery(poly.polyClipped[0].first,poly.polyClipped[1].first,linesLst);
      }
      else{
        linesLst.push_back(std::make_pair(poly.polyClipped[0].first,poly.polyClipped[1].first));
      }

      if(linesLst.size() == 0){
//        isvisible = false;
        return false;
      }

      // To have the exact same pose values as the old version (angle or ogre visibility test only), points should be reorganised when using scanline algorithm.
//      if(sqrt(vpMath::sqr(poly.polyClipped[0].first.get_X() - linesLst[0].first.get_X())) > sqrt(vpMath::sqr(poly.polyClipped[0].first.get_X() - linesLst[0].second.get_X())))
//      {
//          std::vector<std::pair<vpPoint, vpPoint> > linesLstTmp;
//          for(int i = linesLst.size()-1 ; i >= 0 ; i--)
//            linesLstTmp.push_back(std::make_pair(linesLst[i].second,linesLst[i].first));
//          linesLst = linesLstTmp;
//      }

      line->changeFrame(cMo);
      line->projection();
      double rho,theta;
      //rho theta uv
      vpMeterPixelConversion::convertLine(cam,line->getRho(),line->getTheta(),rho,theta);

      while (theta > M_PI) { theta -= M_PI ; }
      while (theta < -M_PI) { theta += M_PI ; }

      if (theta < -M_PI/2.0) theta = -theta - 3*M_PI/2.0;
      else theta = M_PI/2.0 - theta;


      for(unsigned int i = 0 ; i < linesLst.size() ; i++){
        vpImagePoint ip1, ip2;

        linesLst[i].first.project();
        linesLst[i].second.project();

        vpMeterPixelConversion::convertPoint(cam,linesLst[i].first.get_x(),linesLst[i].first.get_y(),ip1);
        vpMeterPixelConversion::convertPoint(cam,linesLst[i].second.get_x(),linesLst[i].second.get_y(),ip2);

        vpMbtMeLine *melinePt = new vpMbtMeLine ;
        melinePt->setMe(me) ;

        //    meline[i]->setDisplay(vpMeSite::RANGE_RESULT) ;
        melinePt->setInitRange(0);

        int marge = /*10*/5; //ou 5 normalement
        if (ip1.get_j()<ip2.get_j()) { melinePt->jmin = (int)ip1.get_j()-marge ; melinePt->jmax = (int)ip2.get_j()+marge ; } else{ melinePt->jmin = (int)ip2.get_j()-marge ; melinePt->jmax = (int)ip1.get_j()+marge ; }
        if (ip1.get_i()<ip2.get_i()) { melinePt->imin = (int)ip1.get_i()-marge ; melinePt->imax = (int)ip2.get_i()+marge ; } else{ melinePt->imin = (int)ip2.get_i()-marge ; melinePt->imax = (int)ip1.get_i()+marge ; }

        try
        {
          melinePt->initTracking(I,ip1,ip2,rho,theta);
          meline.push_back(melinePt);
  //        nbFeature.push_back((unsigned int) melinePt->getMeList().size());
  //        nbFeatureTotal += nbFeature.back();
        }
        catch(...)
        {
          //vpTRACE("the line can't be initialized");
  //        if (melinePt!=NULL) delete melinePt;
  //        melinePt=NULL;
  //        isvisible = false;
          return false;
        }
      }
    }
    else{
      isvisible = false;
//      return false;
    }
  }

//	trackMovingEdge(I,cMo)  ;
  return true;
}



/*!
  Track the moving edges in the image.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceLine::trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix & /*cMo*/)
{

  if (isvisible)
  {
//     p1->changeFrame(cMo) ;
//     p2->changeFrame(cMo) ;
// 
//     p1->projection() ;
//     p2->projection() ;
//     
//     vpImagePoint ip1, ip2;
// 
//     vpMeterPixelConversion::convertPoint(*cam,p1->get_x(),p1->get_y(),ip1) ;
//     vpMeterPixelConversion::convertPoint(*cam,p2->get_x(),p2->get_y(),ip2) ;
// 
//     int marge = /*10*/5; //ou 5 normalement
//     if (ip1.get_j()<ip2.get_j()) { meline->jmin = ip1.get_j()-marge ; meline->jmax = ip2.get_j()+marge ; }
//     else{ meline->jmin = ip2.get_j()-marge ; meline->jmax = ip1.get_j()+marge ; }
//     if (ip1.get_i()<ip2.get_i()) { meline->imin = ip1.get_i()-marge ; meline->imax = ip2.get_i()+marge ; }
//     else{ meline->imin = ip2.get_i()-marge ; meline->imax = ip1.get_i()+marge ; }

    try
    {
      nbFeatureTotal = 0;
      for(unsigned int i = 0 ; i < meline.size() ; i++){
        meline[i]->track(I) ;
        nbFeature.push_back((unsigned int) meline[i]->getMeList().size());
        nbFeatureTotal += (unsigned int) meline[i]->getMeList().size();
      }
    }
    catch(...)
    {
      for(unsigned int i = 0 ; i < meline.size() ; i++){
        if (meline[i] != NULL) delete meline[i] ;
      }

      nbFeature.clear();
      meline.clear();
      nbFeatureTotal = 0;
      Reinit = true;
      isvisible = false;
    }
  }
}


/*!
  Update the moving edges internal parameters.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceLine::updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  if(isvisible){
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);

    if(poly.getClipping() > 3) // Contains at least one FOV constraint
      cam.computeFov(I.getWidth(), I.getHeight());

    poly.computePolygonClipped(cam);

    if(poly.polyClipped.size() == 2){ //Les points sont visibles.

      std::vector<std::pair<vpPoint, vpPoint> > linesLst;

      if(useScanLine){
        hiddenface->computeScanLineQuery(poly.polyClipped[0].first,poly.polyClipped[1].first,linesLst);
      }
      else{
        linesLst.push_back(std::make_pair(poly.polyClipped[0].first,poly.polyClipped[1].first));
      }

      if(linesLst.size() != meline.size() || linesLst.size() == 0){
        for(unsigned int i = 0 ; i < meline.size() ; i++){
          if (meline[i] != NULL) delete meline[i] ;
        }

        meline.clear();
        nbFeature.clear();
        nbFeatureTotal = 0;
        isvisible = false;
        Reinit = true;
      }
      else{

        // To have the exact same pose values as the old version (angle or ogre visibility test only), points should be reorganised when using scanline algorithm.
//        if(sqrt(vpMath::sqr(poly.polyClipped[0].first.get_X() - linesLst[0].first.get_X())) > sqrt(vpMath::sqr(poly.polyClipped[0].first.get_X() - linesLst[0].second.get_X())))
//        {
//            std::vector<std::pair<vpPoint, vpPoint> > linesLstTmp;
//            for(int i = linesLst.size()-1 ; i >= 0 ; i--)
//              linesLstTmp.push_back(std::make_pair(linesLst[i].second,linesLst[i].first));
//            linesLst = linesLstTmp;
//        }

        line->changeFrame(cMo);
        line->projection();
        double rho,theta;
        //rho theta uv
        vpMeterPixelConversion::convertLine(cam,line->getRho(),line->getTheta(),rho,theta);

        while (theta > M_PI) { theta -= M_PI ; }
        while (theta < -M_PI) { theta += M_PI ; }

        if (theta < -M_PI/2.0) theta = -theta - 3*M_PI/2.0;
        else theta = M_PI/2.0 - theta;


        try
        {
          for(unsigned int i = 0 ; i < linesLst.size() ; i++){
            vpImagePoint ip1, ip2;

            linesLst[i].first.project();
            linesLst[i].second.project();

            vpMeterPixelConversion::convertPoint(cam,linesLst[i].first.get_x(),linesLst[i].first.get_y(),ip1);
            vpMeterPixelConversion::convertPoint(cam,linesLst[i].second.get_x(),linesLst[i].second.get_y(),ip2);

            int marge = /*10*/5; //ou 5 normalement
            if (ip1.get_j()<ip2.get_j()) { meline[i]->jmin = (int)ip1.get_j()-marge ; meline[i]->jmax = (int)ip2.get_j()+marge ; } else{ meline[i]->jmin = (int)ip2.get_j()-marge ; meline[i]->jmax = (int)ip1.get_j()+marge ; }
            if (ip1.get_i()<ip2.get_i()) { meline[i]->imin = (int)ip1.get_i()-marge ; meline[i]->imax = (int)ip2.get_i()+marge ; } else{ meline[i]->imin = (int)ip2.get_i()-marge ; meline[i]->imax = (int)ip1.get_i()+marge ; }

              meline[i]->updateParameters(I,ip1,ip2,rho,theta) ;
              nbFeature[i] = (unsigned int)meline[i]->getMeList().size();
              nbFeatureTotal += nbFeature[i];
          }
        }
        catch(...)
        {
          for(unsigned int j = 0 ; j < meline.size() ; j++){
            if (meline[j] != NULL) delete meline[j] ;
          }

          meline.clear();
          nbFeature.clear();
          nbFeatureTotal = 0;
          isvisible = false;
          Reinit = true;
        }
      }
    }
    else{
      for(unsigned int i = 0 ; i < meline.size() ; i++){
        if (meline[i] != NULL) delete meline[i] ;
      }
      nbFeature.clear();
      meline.clear();
      nbFeatureTotal = 0;
      isvisible = false;
    }
  }
}


/*!
  Reinitialize the line if it is required.
  
  A line is reinitialized if the 2D line do not match enough with the projected 3D line.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceLine::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  for(unsigned int i = 0 ; i < meline.size() ; i++){
    if (meline[i] != NULL) delete meline[i] ;
  }

  nbFeature.clear();
  meline.clear();
  nbFeatureTotal = 0;

  if (initMovingEdge(I,cMo) == false)
    Reinit = true;

  Reinit = false;
}


/*!
  Display the line. The 3D line is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the line.
  \param displayFullModel : If true, the line is displayed even if it is not visible.
*/
void
vpMbtDistanceLine::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                           const vpCameraParameters &camera, const vpColor col, const unsigned int thickness, const bool displayFullModel)
{
  if(isvisible || displayFullModel){
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);

    vpImagePoint ip1, ip2;
    vpCameraParameters c = camera;
    if(poly.getClipping() > 3) // Contains at least one FOV constraint
      c.computeFov(I.getWidth(), I.getHeight());
    
    poly.computePolygonClipped(c);
    
    if( poly.polyClipped.size() == 2 &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::NEAR_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::FAR_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::DOWN_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::UP_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::LEFT_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::RIGHT_CLIPPING) == 0)){

      std::vector<std::pair<vpPoint, vpPoint> > linesLst;
      if(useScanLine && !displayFullModel){
        hiddenface->computeScanLineQuery(poly.polyClipped[0].first,poly.polyClipped[1].first,linesLst,true);
      }
      else{
        linesLst.push_back(std::make_pair(poly.polyClipped[0].first,poly.polyClipped[1].first));
      }

      for(unsigned int i = 0 ; i < linesLst.size() ; i++){
        linesLst[i].first.project();
        linesLst[i].second.project();

        vpMeterPixelConversion::convertPoint(camera,linesLst[i].first.get_x(),linesLst[i].first.get_y(),ip1);
        vpMeterPixelConversion::convertPoint(camera,linesLst[i].second.get_x(),linesLst[i].second.get_y(),ip2);

        vpDisplay::displayLine(I,ip1,ip2,col, thickness);
      }
    }
  }
}


/*!
  Display the line. The 3D line is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the line.
  \param displayFullModel : If true, the line is displayed even if it is not visible.
*/
void
vpMbtDistanceLine::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                           const vpCameraParameters &camera, const vpColor col,
                           const unsigned int thickness, const bool displayFullModel)
{
  if(isvisible || displayFullModel){
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);

    vpImagePoint ip1, ip2;
    vpCameraParameters c = camera;
    if(poly.getClipping() > 3) // Contains at least one FOV constraint
      c.computeFov(I.getWidth(), I.getHeight());

    poly.computePolygonClipped(c);

    if( poly.polyClipped.size() == 2 &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::NEAR_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::FAR_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::DOWN_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::UP_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::LEFT_CLIPPING) == 0) &&
       ((poly.polyClipped[1].second & poly.polyClipped[0].second & vpPolygon3D::RIGHT_CLIPPING) == 0)){

      std::vector<std::pair<vpPoint, vpPoint> > linesLst;
      if(useScanLine && !displayFullModel){
        hiddenface->computeScanLineQuery(poly.polyClipped[0].first,poly.polyClipped[1].first,linesLst,true);
      }
      else{
        linesLst.push_back(std::make_pair(poly.polyClipped[0].first,poly.polyClipped[1].first));
      }

      for(unsigned int i = 0 ; i < linesLst.size() ; i++){
        linesLst[i].first.project();
        linesLst[i].second.project();

        vpMeterPixelConversion::convertPoint(camera,linesLst[i].first.get_x(),linesLst[i].first.get_y(),ip1);
        vpMeterPixelConversion::convertPoint(camera,linesLst[i].second.get_x(),linesLst[i].second.get_y(),ip2);

        vpDisplay::displayLine(I,ip1,ip2,col, thickness);
      }
    }
  }
}


/*!
    Enable to display the points along the line with a color corresponding to their state.
    
    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If blue : The point is removed because of the robust method in the virtual visual servoing.
    
    \param I : The image.
*/
void
vpMbtDistanceLine::displayMovingEdges(const vpImage<unsigned char> &I)
{
  for(unsigned int i = 0 ; i < meline.size() ; i++)
    if (meline[i] != NULL)
    {
      meline[i]->display(I);
    }
}

/*!
  Initialize the size of the interaction matrix and the error vector.
*/
void
vpMbtDistanceLine::initInteractionMatrixError()
{
  if (isvisible == true)
  {
    L.resize(nbFeatureTotal,6) ;
    error.resize(nbFeatureTotal) ;
  }
  else{
    for(unsigned int i = 0 ; i < meline.size() ; i++)
      nbFeature[i] = 0;
    nbFeatureTotal = 0 ;
  }
}

/*!
  Compute the interaction matrix and the error vector corresponding to the line.
*/
void
vpMbtDistanceLine::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo)
{
  if (isvisible)
  {
    // feature projection
    line->changeFrame(cMo) ;
    line->projection() ;

    vpFeatureBuilder::create(featureline,*line) ;

    double rho = featureline.getRho() ;
    double theta = featureline.getTheta() ;

    double co = cos(theta);
    double si = sin(theta);

    double mx = 1.0/cam.get_px() ;
    double my = 1.0/cam.get_py() ;
    double xc = cam.get_u0() ;
    double yc = cam.get_v0() ;

    double alpha_ ;
    vpMatrix H ;
    H = featureline.interaction() ;

    double x,y ;
    vpMeSite p ;
    unsigned int j =0 ;

    for(unsigned int i = 0 ; i < meline.size() ; i++){
      for(std::list<vpMeSite>::const_iterator it=meline[i]->getMeList().begin(); it!=meline[i]->getMeList().end(); ++it){
        x = (double)it->j ;
        y = (double)it->i ;

        x = (x-xc)*mx ;
        y = (y-yc)*my ;

        alpha_ = x*si - y*co;

        double *Lrho = H[0] ;
        double *Ltheta = H[1] ;
        // Calculate interaction matrix for a distance
        for (unsigned int k=0 ; k < 6 ; k++)
        {
          L[j][k] = (Lrho[k] + alpha_*Ltheta[k]);
        }
        error[j] = rho - ( x*co + y*si) ;
        j++;
      }
    }
  }
}

/*!
  Test wether the line is close to the border of the image (at a given threshold)
  
  \param I : the input image (to know its size)
  \param threshold : the threshold in pixel 
  \return true if the line is near the border of the image
*/
bool 
vpMbtDistanceLine::closeToImageBorder(const vpImage<unsigned char>& I, const unsigned int threshold)
{
  if(threshold > I.getWidth() || threshold > I.getHeight()){
    return true;
  }
  if (isvisible){

    for(unsigned int i = 0 ; i < meline.size() ; i++){
      for(std::list<vpMeSite>::const_iterator it=meline[i]->getMeList().begin(); it!=meline[i]->getMeList().end(); ++it){
        int i_ = it->i ;
        int j_ = it->j ;

        if(i_ < 0 || j_ < 0){ //out of image.
          return true;
        }

        if( ((unsigned int)i_ > (I.getHeight()- threshold) ) || (unsigned int)i_ < threshold ||
            ((unsigned int)j_ > (I.getWidth ()- threshold) ) || (unsigned int)j_ < threshold ) {
          return true;
        }
      }
    }
  }
  return false;
}
