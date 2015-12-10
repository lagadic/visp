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
 * Klt cylinder, containing points of interest.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp3/mbt/vpMbtDistanceKltCylinder.h>
#include <visp3/mbt/vpMbtDistanceKltPoints.h>
#include <visp3/core/vpPolygon.h>


#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))

/*!
  Basic constructor.

*/
vpMbtDistanceKltCylinder::vpMbtDistanceKltCylinder()
  : c0Mo(), p1Ext(), p2Ext(), cylinder(), circle1(), circle2(),
    initPoints(), initPoints3D(), curPoints(), curPointsInd(),
    nbPointsCur(0), nbPointsInit(0), minNbPoint(4), enoughPoints(false),
    cam(), isTrackedKltCylinder(true), listIndicesCylinderBBox(), hiddenface(NULL), useScanLine(false)
{
}

/*!
  Basic destructor.

*/
vpMbtDistanceKltCylinder::~vpMbtDistanceKltCylinder()
{}



void
vpMbtDistanceKltCylinder::buildFrom(const vpPoint &p1, const vpPoint &p2, const double &r)
{
  p1Ext = p1;
  p2Ext = p2;

  vpColVector ABC(3);
  vpColVector V1(3);
  vpColVector V2(3);

  V1[0] = p1.get_oX();
  V1[1] = p1.get_oY();
  V1[2] = p1.get_oZ();
  V2[0] = p2.get_oX();
  V2[1] = p2.get_oY();
  V2[2] = p2.get_oZ();

  // Get the axis of the cylinder
  ABC = V1-V2;

  // Build our extremity circles
  circle1.setWorldCoordinates(ABC[0],ABC[1],ABC[2],p1.get_oX(),p1.get_oY(),p1.get_oZ(),r);
  circle2.setWorldCoordinates(ABC[0],ABC[1],ABC[2],p2.get_oX(),p2.get_oY(),p2.get_oZ(),r);

  // Build our cylinder
  cylinder.setWorldCoordinates(ABC[0],ABC[1],ABC[2],(p1.get_oX()+p2.get_oX())/2.0,(p1.get_oY()+p2.get_oY())/2.0,(p1.get_oZ()+p2.get_oZ())/2.0,r);
}

/*!
  Initialise the cylinder to track. All the points in the map, representing all the
  map detected in the image, are parsed in order to extract the id of the points
  that are indeed in the face.

  \param _tracker : ViSP OpenCV KLT Tracker.
  \param cMo : Pose of the object in the camera frame at initialization.
*/
void
vpMbtDistanceKltCylinder::init(const vpKltOpencv& _tracker, const vpHomogeneousMatrix &cMo)
{
  c0Mo = cMo;
  cylinder.changeFrame(cMo);

  // extract ids of the points in the face
  nbPointsInit = 0;
  nbPointsCur = 0;
  initPoints = std::map<int, vpImagePoint>();
  initPoints3D = std::map<int, vpPoint>();
  curPoints = std::map<int, vpImagePoint>();
  curPointsInd = std::map<int, int>();

  for (unsigned int i = 0; i < static_cast<unsigned int>(_tracker.getNbFeatures()); i ++){
    int id;
    float x_tmp, y_tmp;
    _tracker.getFeature((int)i, id, x_tmp, y_tmp);

    bool add = false;

    if(useScanLine)
    {
      if((unsigned int)y_tmp <  hiddenface->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
         (unsigned int)x_tmp <  hiddenface->getMbScanLineRenderer().getPrimitiveIDs().getWidth())
      {
        for(unsigned int kc = 0 ; kc < listIndicesCylinderBBox.size() ; kc++)
          if(hiddenface->getMbScanLineRenderer().getPrimitiveIDs()[(unsigned int)y_tmp][(unsigned int)x_tmp] == listIndicesCylinderBBox[kc])
          {
            add = true;
            break;
          }
      }
    }
    else
    {
      std::vector<vpImagePoint> roi;
      for(unsigned int kc = 0 ; kc < listIndicesCylinderBBox.size() ; kc++)
      {
        hiddenface->getPolygon()[listIndicesCylinderBBox[kc]]->getRoiClipped(cam, roi);
        if(vpPolygon::isInside(roi, y_tmp, x_tmp))
        {
          add = true;
          break;
        }
        roi.clear();
      }
    }

    if(add){

      double xm=0, ym=0;
      vpPixelMeterConversion::convertPoint(cam, x_tmp, y_tmp, xm, ym);
      double Z = computeZ(xm,ym);
      if(!vpMath::isNaN(Z)){
        initPoints[id] = vpImagePoint(y_tmp, x_tmp);
        curPoints[id] = vpImagePoint(y_tmp, x_tmp);
        curPointsInd[id] = (int)i;
        nbPointsInit++;
        nbPointsCur++;


        vpPoint p;
        p.setWorldCoordinates(xm * Z, ym * Z, Z);
        initPoints3D[id] = p;
        //std::cout << "Computed Z for : " << xm << "," << ym << " : " << computeZ(xm,ym) << std::endl;
      }
    }
  }

  if(nbPointsCur >= minNbPoint) enoughPoints = true;
  else enoughPoints = false;

  //std::cout << "Nb detected points in cylinder : " << nbPointsCur << std::endl;
}

/*!
  compute the number of point in this instanciation of the tracker that corresponds
  to the points of the cylinder

  \param _tracker : the KLT tracker
  \return the number of points that are tracked in this face and in this instanciation of the tracker
*/
unsigned int
vpMbtDistanceKltCylinder::computeNbDetectedCurrent(const vpKltOpencv& _tracker)
{
  int id;
  float x, y;
  nbPointsCur = 0;
  curPoints = std::map<int, vpImagePoint>();
  curPointsInd = std::map<int, int>();

  for (unsigned int i = 0; i < static_cast<unsigned int>(_tracker.getNbFeatures()); i++){
    _tracker.getFeature((int)i, id, x, y);
    if(isTrackedFeature(id)){
      curPoints[id] = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
      curPointsInd[id] = (int)i;
      nbPointsCur++;
    }
  }

  if(nbPointsCur >= minNbPoint) enoughPoints = true;
  else enoughPoints = false;

  return nbPointsCur;
}

/*!
  This method removes the outliers. A point is considered as outlier when its
  associated weight is below a given threshold (threshold_outlier).

  \param _w : Vector containing the weight of all the tracked points.
  \param threshold_outlier : Threshold to specify wether or not a point has to be deleted.
*/
void
vpMbtDistanceKltCylinder::removeOutliers(const vpColVector& _w, const double &threshold_outlier)
{
  std::map<int, vpImagePoint> tmp;
  std::map<int, int> tmp2;
  unsigned int nbSupp = 0;
  unsigned int k = 0;

  nbPointsCur = 0;
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for( ; iter != curPoints.end(); iter++){
    if(_w[k] > threshold_outlier && _w[k+1] > threshold_outlier){
//     if(_w[k] > threshold_outlier || _w[k+1] > threshold_outlier){
      tmp[iter->first] = vpImagePoint(iter->second.get_i(), iter->second.get_j());
      tmp2[iter->first] = curPointsInd[iter->first];
      nbPointsCur++;
    }
    else{
      nbSupp++;
      initPoints.erase(iter->first);
    }

    k+=2;
  }

  if(nbSupp != 0){
    curPoints = std::map<int, vpImagePoint>();
    curPointsInd = std::map<int, int>();

    curPoints = tmp;
    curPointsInd = tmp2;
    if(nbPointsCur >= minNbPoint) enoughPoints = true;
    else enoughPoints = false;
  }
}

/*!
  Compute the interaction matrix and the residu vector for the face.
  The method assumes that these two objects are properly sized in order to be
  able to improve the speed with the use of SubCoVector and subMatrix.

  \warning The function preCompute must be called before the this method.

  \param _cMc0 : camera displacement since initialisation
  \param _R : the residu vector
  \param _J : the interaction matrix
*/
void
vpMbtDistanceKltCylinder::computeInteractionMatrixAndResidu(const vpHomogeneousMatrix &_cMc0, vpColVector& _R, vpMatrix& _J)
{
  unsigned int index_ = 0;

  cylinder.changeFrame(_cMc0 * c0Mo);

  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for( ; iter != curPoints.end(); iter++){
    int id(iter->first);
    double i_cur(iter->second.get_i()), j_cur(iter->second.get_j());

    double x_cur(0), y_cur(0);
    vpPixelMeterConversion::convertPoint(cam, j_cur, i_cur, x_cur, y_cur);

    vpPoint p0 =  initPoints3D[id];
    p0.changeFrame(_cMc0);
    p0.project();

    double x0_transform(p0.get_x()), y0_transform(p0.get_y()) ;

    double Z = computeZ(x_cur, y_cur);

    if(vpMath::isNaN(Z) || Z < std::numeric_limits<double>::epsilon()){
//      std::cout << "Z is Nan : " << A << " , " << B << " , " << C << " | " << Z << " | " << x_cur << " , " << y_cur << std::endl;
//      std::cout << std::sqrt(B*B - A*C) << " , " << B*B - A*C << std::endl;

      _J[2*index_][0] = 0;
      _J[2*index_][1] = 0;
      _J[2*index_][2] = 0;
      _J[2*index_][3] = 0;
      _J[2*index_][4] = 0;
      _J[2*index_][5] = 0;

      _J[2*index_+1][0] = 0;
      _J[2*index_+1][1] = 0;
      _J[2*index_+1][2] = 0;
      _J[2*index_+1][3] = 0;
      _J[2*index_+1][4] = 0;
      _J[2*index_+1][5] = 0;

      _R[2*index_] =  (x0_transform - x_cur);
      _R[2*index_+1] = (y0_transform - y_cur);
      index_++;
    }
    else
    {
      double invZ = 1.0/Z;

      _J[2*index_][0] = - invZ;
      _J[2*index_][1] = 0;
      _J[2*index_][2] = x_cur * invZ;
      _J[2*index_][3] = x_cur * y_cur;
      _J[2*index_][4] = -(1+x_cur*x_cur);
      _J[2*index_][5] = y_cur;

      _J[2*index_+1][0] = 0;
      _J[2*index_+1][1] = - invZ;
      _J[2*index_+1][2] = y_cur * invZ;
      _J[2*index_+1][3] = (1+y_cur*y_cur);
      _J[2*index_+1][4] = - y_cur * x_cur;
      _J[2*index_+1][5] = - x_cur;

      _R[2*index_] =  (x0_transform - x_cur);
      _R[2*index_+1] = (y0_transform - y_cur);
      index_++;
    }
  }
}

/*!
  Test whether the feature with identifier id in paramters is in the list of tracked
  features.

  \param _id : the id of the current feature to test
  \return true if the id is in the list of tracked feature
*/
bool
vpMbtDistanceKltCylinder::isTrackedFeature(const int _id)
{
  std::map<int, vpImagePoint>::iterator iter = initPoints.find(_id);
  if(iter != initPoints.end())
    return true;

  return false;
}

/*!
  Modification of all the pixels that are in the roi to the value of _nb (
  default is 255).

  \param mask : the mask to update (0, not in the object, _nb otherwise).
  \param nb : Optionnal value to set to the pixels included in the face.
  \param shiftBorder : Optionnal shift for the border in pixel (sort of built-in erosion) to avoid to consider pixels near the limits of the face.
*/
void
vpMbtDistanceKltCylinder::updateMask(
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cv::Mat &mask,
#else
    IplImage* mask,
#endif
    unsigned char nb, unsigned int shiftBorder)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  int width  = mask.cols;
  int height = mask.rows;
#else
  int width  = mask->width;
  int height = mask->height;
#endif

  for(unsigned int kc = 0 ; kc < listIndicesCylinderBBox.size() ; kc++)
  {
      if((*hiddenface)[listIndicesCylinderBBox[kc]]->isVisible() && (*hiddenface)[listIndicesCylinderBBox[kc]]->getNbPoint() > 2)
      {
          int i_min, i_max, j_min, j_max;
          std::vector<vpImagePoint> roi;
          (*hiddenface)[listIndicesCylinderBBox[kc]]->getRoiClipped(cam, roi);
          vpPolygon3D::getMinMaxRoi(roi, i_min, i_max, j_min,j_max);

          /* check image boundaries */
          if(i_min > height){ //underflow
            i_min = 0;
          }
          if(i_max > height){
            i_max = height;
          }
          if(j_min > width){ //underflow
            j_min = 0;
          }
          if(j_max > width){
            j_max = width;
          }

          double shiftBorder_d = (double) shiftBorder;
        #if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
          for(int i=i_min; i< i_max; i++){
            double i_d = (double) i;
            for(int j=j_min; j< j_max; j++){
              double j_d = (double) j;
              if(shiftBorder != 0){
                if( vpPolygon::isInside(roi, i_d, j_d)
                    && vpPolygon::isInside(roi, i_d+shiftBorder_d, j_d+shiftBorder_d)
                    && vpPolygon::isInside(roi, i_d-shiftBorder_d, j_d+shiftBorder_d)
                    && vpPolygon::isInside(roi, i_d+shiftBorder_d, j_d-shiftBorder_d)
                    && vpPolygon::isInside(roi, i_d-shiftBorder_d, j_d-shiftBorder_d) ){
                  mask.at<unsigned char>(i,j) = nb;
                }
              }
              else{
                if(vpPolygon::isInside(roi, i, j)){
                  mask.at<unsigned char>(i,j) = nb;
                }
              }
            }
          }
        #else
          unsigned char* ptrData = (unsigned char*)mask->imageData + i_min*mask->widthStep+j_min;
          for(int i=i_min; i< i_max; i++){
            double i_d = (double) i;
            for(int j=j_min; j< j_max; j++){
              double j_d = (double) j;
              if(shiftBorder != 0){
                if( vpPolygon::isInside(roi, i_d, j_d)
                    && vpPolygon::isInside(roi, i_d+shiftBorder_d, j_d+shiftBorder_d)
                    && vpPolygon::isInside(roi, i_d-shiftBorder_d, j_d+shiftBorder_d)
                    && vpPolygon::isInside(roi, i_d+shiftBorder_d, j_d-shiftBorder_d)
                    && vpPolygon::isInside(roi, i_d-shiftBorder_d, j_d-shiftBorder_d) ){
                  *(ptrData++) = nb;
                }
                else{
                  ptrData++;
                }
              }
              else{
                if(vpPolygon::isInside(roi, i, j)){
                  *(ptrData++) = nb;
                }
                else{
                  ptrData++;
                }
              }
            }
            ptrData += mask->widthStep - j_max + j_min;
          }
        #endif
    }
  }
}

/*!
  Display the primitives tracked for the cylinder.

  \param _I : The image where to display.
*/
void
vpMbtDistanceKltCylinder::displayPrimitive(const vpImage<unsigned char>& _I)
{
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for( ; iter != curPoints.end(); iter++){
    int id(iter->first);
    vpImagePoint iP;
    iP.set_i(static_cast<double>(iter->second.get_i()));
    iP.set_j(static_cast<double>(iter->second.get_j()));

    vpDisplay::displayCross(_I, iP, 10, vpColor::red);

    iP.set_i( vpMath::round( iP.get_i() + 7 ) );
    iP.set_j( vpMath::round( iP.get_j() + 7 ) );
    char ide[10];
    sprintf(ide, "%ld", static_cast<long int>(id));
    vpDisplay::displayText(_I, iP, ide, vpColor::red);
  }
}

/*!
  Display the primitives tracked for the cylinder.

  \param _I : The image where to display.
*/
void
vpMbtDistanceKltCylinder::displayPrimitive(const vpImage<vpRGBa>& _I)
{
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for( ; iter != curPoints.end(); iter++){
    int id(iter->first);
    vpImagePoint iP;
    iP.set_i(static_cast<double>(iter->second.get_i()));
    iP.set_j(static_cast<double>(iter->second.get_j()));

    vpDisplay::displayCross(_I, iP, 10, vpColor::red);

    iP.set_i( vpMath::round( iP.get_i() + 7 ) );
    iP.set_j( vpMath::round( iP.get_j() + 7 ) );
    char ide[10];
    sprintf(ide, "%ld", static_cast<long int>(id));
    vpDisplay::displayText(_I, iP, ide, vpColor::red);
  }
}

void
vpMbtDistanceKltCylinder::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &camera,
                                  const vpColor col, const unsigned int thickness, const bool /*displayFullModel*/)
{
  //if(isvisible || displayFullModel)
  {
    // Perspective projection
    circle1.changeFrame(cMo);
    circle2.changeFrame(cMo);
    cylinder.changeFrame(cMo);

    try{
      circle1.projection();
    }
    catch(...){std::cout<<"Problem projection circle 1";}
    try{
      circle2.projection();
    }
    catch(...){std::cout<<"Problem projection circle 2";}

    cylinder.projection();

    double rho1,theta1;
    double rho2,theta2;

    // Meters to pixels conversion
    vpMeterPixelConversion::convertLine(camera,cylinder.getRho1(),cylinder.getTheta1(),rho1,theta1);
    vpMeterPixelConversion::convertLine(camera,cylinder.getRho2(),cylinder.getTheta2(),rho2,theta2);

    // Determine intersections between circles and limbos
    double i11,i12,i21,i22,j11,j12,j21,j22;

    vpCircle::computeIntersectionPoint(circle1, cam, rho1, theta1, i11, j11);
    vpCircle::computeIntersectionPoint(circle2, cam, rho1, theta1, i12, j12);

    vpCircle::computeIntersectionPoint(circle1, cam, rho2, theta2, i21, j21);
    vpCircle::computeIntersectionPoint(circle2, cam, rho2, theta2, i22, j22);

    // Create the image points
    vpImagePoint ip11,ip12,ip21,ip22;
    ip11.set_ij(i11,j11);
    ip12.set_ij(i12,j12);
    ip21.set_ij(i21,j21);
    ip22.set_ij(i22,j22);

    // Display
    vpDisplay::displayLine(I,ip11,ip12,col, thickness);
    vpDisplay::displayLine(I,ip21,ip22,col, thickness);
  }
}

void
vpMbtDistanceKltCylinder::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &camera,
                                  const vpColor col, const unsigned int thickness, const bool /*displayFullModel*/)
{
  //if(isvisible || displayFullModel)
  {
    // Perspective projection
    circle1.changeFrame(cMo);
    circle2.changeFrame(cMo);
    cylinder.changeFrame(cMo);

    try{
      circle1.projection();
    }
    catch(...){std::cout<<"Problem projection circle 1";}
    try{
      circle2.projection();
    }
    catch(...){std::cout<<"Problem projection circle 2";}

    cylinder.projection();

    double rho1,theta1;
    double rho2,theta2;

    // Meters to pixels conversion
    vpMeterPixelConversion::convertLine(camera,cylinder.getRho1(),cylinder.getTheta1(),rho1,theta1);
    vpMeterPixelConversion::convertLine(camera,cylinder.getRho2(),cylinder.getTheta2(),rho2,theta2);

    // Determine intersections between circles and limbos
    double i11,i12,i21,i22,j11,j12,j21,j22;

    vpCircle::computeIntersectionPoint(circle1, camera, rho1, theta1, i11, j11);
    vpCircle::computeIntersectionPoint(circle2, camera, rho1, theta1, i12, j12);

    vpCircle::computeIntersectionPoint(circle1, camera, rho2, theta2, i21, j21);
    vpCircle::computeIntersectionPoint(circle2, camera, rho2, theta2, i22, j22);

    // Create the image points
    vpImagePoint ip11,ip12,ip21,ip22;
    ip11.set_ij(i11,j11);
    ip12.set_ij(i12,j12);
    ip21.set_ij(i21,j21);
    ip22.set_ij(i22,j22);

    // Display
    vpDisplay::displayLine(I,ip11,ip12,col, thickness);
    vpDisplay::displayLine(I,ip21,ip22,col, thickness);
  }
}


// ######################
//   Private Functions
// ######################

double
vpMbtDistanceKltCylinder::computeZ(const double &x, const double &y)
{
  double A = x*x + y*y + 1 - ((cylinder.getA()*x+cylinder.getB()*y+cylinder.getC()) * (cylinder.getA()*x+cylinder.getB()*y+cylinder.getC()));
  double B = (x * cylinder.getX() + y * cylinder.getY() + cylinder.getZ());
  double C = cylinder.getX() * cylinder.getX() + cylinder.getY() * cylinder.getY() + cylinder.getZ() * cylinder.getZ() - cylinder.getR() * cylinder.getR();

  return (B - std::sqrt(B*B - A*C))/A;
}
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbtDistanceKltCylinder.cpp.o) has no symbols
void dummy_vpMbtDistanceKltCylinder() {};
#endif
