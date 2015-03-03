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
#include <visp/vpConfig.h>

/*!
 \file vpMbtDistanceLine.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <visp/vpMbtDistanceLine.h>
#include <visp/vpPlane.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpFeatureBuilder.h>
#include <stdlib.h>

void buildPlane(vpPoint &P, vpPoint &Q, vpPoint &R, vpPlane &plane);
void buildLine(vpPoint &P1, vpPoint &P2, vpPoint &P3, vpPoint &P4, vpLine &L);

/*!
  Basic constructor
*/
vpMbtDistanceLine::vpMbtDistanceLine()
  : name(), index(0), cam(), me(NULL), alpha(0), wmean(1),
    featureline(), poly(), meline(NULL), line(NULL), p1(NULL), p2(NULL), L(),
    error(), nbFeature(0), Reinit(false), hiddenface(NULL), Lindex_polygon(),
    isvisible(false)
{
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtDistanceLine::~vpMbtDistanceLine()
{
//	cout << "Deleting line " << index << endl ;
  if (line != NULL) delete line ;
  if (meline != NULL) delete meline ;

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
    
    vpPoint P3;
    P3.setWorldCoordinates(V3[0],V3[1],V3[2]);
    vpPoint P4;
    P4.setWorldCoordinates(V4[0],V4[1],V4[2]);
    buildLine(*p1,*p2, P3,P4, *line) ;
  }
  else
  {
    vpPoint P3;
    P3.setWorldCoordinates(V1[0],V1[1],V1[2]);
    vpPoint P4;
    P4.setWorldCoordinates(V2[0],V2[1],V2[2]);
    buildLine(*p1,*p2,P3,P4,*line) ;
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
  if (meline != NULL)
  {
    meline->reset();
    meline->setMe(me) ;
  }
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
  if(isvisible){
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);
    
    if(poly.getClipping() > 3) // Contains at least one FOV constraint
      cam.computeFov(I.getWidth(), I.getHeight());
    
    poly.computeRoiClipped(cam);
    
    if(poly.roiPointsClip.size() == 2){ //Les points sont visibles.
      vpImagePoint ip1, ip2;
      double rho,theta;
      line->changeFrame(cMo);
      line->projection();
    
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[0].first.get_x(),poly.roiPointsClip[0].first.get_y(),ip1);
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[1].first.get_x(),poly.roiPointsClip[1].first.get_y(),ip2);
      
      //rho theta uv
      vpMeterPixelConversion::convertLine(cam,line->getRho(),line->getTheta(),rho,theta);
      
      while (theta > M_PI) { theta -= M_PI ; }
      while (theta < -M_PI) { theta += M_PI ; }
      
      if (theta < -M_PI/2.0) theta = -theta - 3*M_PI/2.0;
      else theta = M_PI/2.0 - theta;

      meline = new vpMbtMeLine ;
      meline->setMe(me) ;

    //    meline->setDisplay(vpMeSite::RANGE_RESULT) ;
      meline->setInitRange(0);
      
      int marge = /*10*/5; //ou 5 normalement
      if (ip1.get_j()<ip2.get_j()) { meline->jmin = (int)ip1.get_j()-marge ; meline->jmax = (int)ip2.get_j()+marge ; } else{ meline->jmin = (int)ip2.get_j()-marge ; meline->jmax = (int)ip1.get_j()+marge ; }
      if (ip1.get_i()<ip2.get_i()) { meline->imin = (int)ip1.get_i()-marge ; meline->imax = (int)ip2.get_i()+marge ; } else{ meline->imin = (int)ip2.get_i()-marge ; meline->imax = (int)ip1.get_i()+marge ; }
      
      try
      {
        meline->initTracking(I,ip1,ip2,rho,theta);
        nbFeature =(unsigned int) meline->getMeList().size();
      }
      catch(...)
      {
        //vpTRACE("the line can't be initialized");
        nbFeature = 0;
        return false;
      }
    }
    else{
      if (meline!=NULL) delete meline;
      meline=NULL;
      nbFeature = 0;
      isvisible = false;
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
      meline->track(I) ;
      nbFeature =(unsigned int) meline->getMeList().size();
    }
    catch(...)
    {
      meline->reset(); // Might not be necessary
      nbFeature = 0;
      Reinit = true;
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
    
    poly.computeRoiClipped(cam);
    
    if(poly.roiPointsClip.size() == 2){ //Les points sont visibles.
      vpImagePoint ip1, ip2;
      double rho,theta;
      line->changeFrame(cMo);
      line->projection();
    
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[0].first.get_x(),poly.roiPointsClip[0].first.get_y(),ip1);
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[1].first.get_x(),poly.roiPointsClip[1].first.get_y(),ip2);
      
      //rho theta uv
      vpMeterPixelConversion::convertLine(cam,line->getRho(),line->getTheta(),rho,theta);
      
      while (theta > M_PI) { theta -= M_PI ; }
      while (theta < -M_PI) { theta += M_PI ; }
      
      if (theta < -M_PI/2.0) theta = -theta - 3*M_PI/2.0;
      else theta = M_PI/2.0 - theta;

      int marge = /*10*/5; //ou 5 normalement
      if (ip1.get_j()<ip2.get_j()) { meline->jmin = (int)ip1.get_j()-marge ; meline->jmax = (int)ip2.get_j()+marge ; } else{ meline->jmin = (int)ip2.get_j()-marge ; meline->jmax = (int)ip1.get_j()+marge ; }
      if (ip1.get_i()<ip2.get_i()) { meline->imin = (int)ip1.get_i()-marge ; meline->imax = (int)ip2.get_i()+marge ; } else{ meline->imin = (int)ip2.get_i()-marge ; meline->imax = (int)ip1.get_i()+marge ; }

      try 
      {
        //meline->updateParameters(I,rho,theta) ;
        meline->updateParameters(I,ip1,ip2,rho,theta) ;
        nbFeature = (unsigned int)meline->getMeList().size();
      }
      catch(...)
      {
        Reinit = true;
        nbFeature = 0;
      }
    }
    else{
      if (meline!=NULL) delete meline;
      meline=NULL;
      isvisible = false;
      nbFeature = 0;
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
  if(meline!= NULL)
    delete meline;
  
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
    
    poly.computeRoiClipped(c);
    
    if( poly.roiPointsClip.size() == 2 && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::NEAR_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::FAR_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::DOWN_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::UP_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::LEFT_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::RIGHT_CLIPPING) == 0)){ 
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[0].first.get_x(),poly.roiPointsClip[0].first.get_y(),ip1);
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[1].first.get_x(),poly.roiPointsClip[1].first.get_y(),ip2);

      vpDisplay::displayLine(I,ip1,ip2,col, thickness);
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
    
    poly.computeRoiClipped(c);
    
    if( poly.roiPointsClip.size() == 2 && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::NEAR_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::FAR_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::DOWN_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::UP_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::LEFT_CLIPPING) == 0) && 
       ((poly.roiPointsClip[1].second & poly.roiPointsClip[0].second & vpMbtPolygon::RIGHT_CLIPPING) == 0)){ 
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[0].first.get_x(),poly.roiPointsClip[0].first.get_y(),ip1);
      vpMeterPixelConversion::convertPoint(cam,poly.roiPointsClip[1].first.get_x(),poly.roiPointsClip[1].first.get_y(),ip2);

      vpDisplay::displayLine(I,ip1,ip2,col, thickness);
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
  if (meline != NULL)
  {
    meline->display(I);
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
    L.resize((unsigned int)meline->getMeList().size(),6) ;
    error.resize((unsigned int)meline->getMeList().size()) ;
    //nbFeature = (unsigned int)meline->getMeList().size() ;
  }
//  else
//    nbFeature = 0 ;
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
    for(std::list<vpMeSite>::const_iterator it=meline->getMeList().begin(); it!=meline->getMeList().end(); ++it){
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

    for(std::list<vpMeSite>::const_iterator it=meline->getMeList().begin(); it!=meline->getMeList().end(); ++it){
      int i = it->i ;
      int j = it->j ;
      
      if(i < 0 || j < 0){ //out of image.
        return true;
      }
      
      if( ((unsigned int)i > (I.getHeight()- threshold) ) || (unsigned int)i < threshold ||
          ((unsigned int)j > (I.getWidth ()- threshold) ) || (unsigned int)j < threshold ) {
        return true;
      }
    }
  }
  return false;
}
