/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 2005 - 2013 Inria. All rights reserved.
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
 * Generic model polygon, containing points of interest.
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/


#include <visp/vpMbtKltPolygon.h>

#ifdef VISP_HAVE_OPENCV

/*!
  Basic constructor.

*/
vpMbtKltPolygon::vpMbtKltPolygon()
{
  minNbPoint = 4;
  enoughPoints = false;
  
  nbPointsInit = 0;
  nbPointsCur = 0;
  initPoints = std::map<int, vpImagePoint>();
  curPoints = std::map<int, vpImagePoint>();
  
  isvisible = false;
}

/*!
  Basic destructor.

*/
vpMbtKltPolygon::~vpMbtKltPolygon()
{}

/*!
  Initialise the face to track. All the points in the map, representing all the
  map detected in the image, are parsed in order to extract the id of the points
  that are indeed in the face.

  \param _tracker : ViSP OpenCV KLT Tracker.
*/
void
vpMbtKltPolygon::init(const vpKltOpencv& _tracker)
{
  // extract ids of the points in the face
  nbPointsInit = 0;
  nbPointsCur = 0;
  initPoints = std::map<int, vpImagePoint>();
  curPoints = std::map<int, vpImagePoint>();
  
  for (unsigned int i = 0; i < static_cast<unsigned int>(_tracker.getNbFeatures()); i ++){
    int id;
    float x_tmp, y_tmp;
    _tracker.getFeature((int)i, id, x_tmp, y_tmp);
    
    if(isInside(getRoi(cam), y_tmp, x_tmp)){
      initPoints[id] = vpImagePoint(y_tmp, x_tmp);
      curPoints[id] = vpImagePoint(y_tmp, x_tmp);
      nbPointsInit++;
      nbPointsCur++;
    }
  }

  // initialisation of the value for the computation in SE3
  vpPlane plan(p[0], p[1], p[2]);

  d0 = plan.getD();
  N = plan.getNormal(); 
  
  N.normalize();
  N_cur = N;
  invd0 = 1.0 / d0;
}

/*!
  compute the number of point in this instanciation of the tracker that corresponds
  to the points of the face

  \param _tracker : the KLT tracker
  \return the number of points that are tracked in this face and in this instanciation of the tracker
*/
unsigned int
vpMbtKltPolygon::computeNbDetectedCurrent(const vpKltOpencv& _tracker)
{
  int id;
  float x, y;
  nbPointsCur = 0;
  curPoints = std::map<int, vpImagePoint>();
  
  for (unsigned int i = 0; i < static_cast<unsigned int>(_tracker.getNbFeatures()); i++){
    _tracker.getFeature((int)i, id, x, y);
    if(isTrackedFeature(id)){
      curPoints[id] = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
      nbPointsCur++;
    }
  }
  
  if(nbPointsCur >= minNbPoint) enoughPoints = true;
  else enoughPoints = false;
  
  return nbPointsCur;
}

/*!
  Compute the interaction matrix and the residu vector for the face.
  The method assumes that these two objects are properly sized in order to be
  able to improve the speed with the use of SubCoVector and subMatrix.

  \warning The function preCompute must be called before the this method.

  \param _R : the residu vector
  \param _J : the interaction matrix
*/
void
vpMbtKltPolygon::computeInteractionMatrixAndResidu(vpColVector& _R, vpMatrix& _J)
{
  unsigned int index = 0;
  
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for( ; iter != curPoints.end(); iter++){
    int id(iter->first);
    double i_cur(iter->second.get_i()), j_cur(iter->second.get_j());
    
    double x_cur(0), y_cur(0);
    vpPixelMeterConversion::convertPoint(cam, j_cur, i_cur, x_cur, y_cur);
    
    vpImagePoint iP0 = initPoints[id];
    double x0(0), y0(0);
    vpPixelMeterConversion::convertPoint(cam, iP0, x0, y0);
    
    double x0_transform, y0_transform ;// equivalent x and y in the first image (reference)
    computeP_mu_t(x0, y0, x0_transform, y0_transform, H );

    double invZ = compute_1_over_Z(x_cur, y_cur);
      
    _J[2*index][0] = - invZ;
    _J[2*index][1] = 0;
    _J[2*index][2] = x_cur * invZ;
    _J[2*index][3] = x_cur * y_cur;
    _J[2*index][4] = -(1+x_cur*x_cur);
    _J[2*index][5] = y_cur;
    
    _J[2*index+1][0] = 0;
    _J[2*index+1][1] = - invZ;
    _J[2*index+1][2] = y_cur * invZ;
    _J[2*index+1][3] = (1+y_cur*y_cur);
    _J[2*index+1][4] = - y_cur * x_cur;
    _J[2*index+1][5] = - x_cur;

    _R[2*index] =  (x0_transform - x_cur);
    _R[2*index+1] = (y0_transform - y_cur);
    index++;
  }
}

double
vpMbtKltPolygon::compute_1_over_Z(const double x, const double y)
{
  double num = cRc0_0n[0] * x + cRc0_0n[1] * y + cRc0_0n[2];
  double den = -(d0 - dt);
  return num/den;
}

/*!
  Compute the new coordinates of a point given by its \f$(x,y)\f$ coordinates
  after the homography.

  \f$ P_t = {}^{c_t}H_{c_0} . P_0 \f$

  \param x_in : the x coordinates of the input point
  \param y_in : the y coordinates of the input point
  \param x_out : the x coordinates of the output point
  \param y_out : the y coordinates of the output point
  \param _cHc0 : the homography used to transfer the point
*/
inline void
vpMbtKltPolygon::computeP_mu_t(const double x_in, const double y_in, double& x_out, double& y_out, const vpMatrix& _cHc0)
{
  double p_mu_t_2 = x_in * _cHc0[2][0] + y_in * _cHc0[2][1] + _cHc0[2][2];

  if( fabs(p_mu_t_2) < std::numeric_limits<double>::epsilon()){
    x_out = 0.0;
    y_out = 0.0;
    throw vpException(vpException::divideByZeroError, "the depth of the point is calculated to zero");
  }

  x_out = (x_in * _cHc0[0][0] + y_in * _cHc0[0][1] + _cHc0[0][2]) / p_mu_t_2;
  y_out = (x_in * _cHc0[1][0] + y_in * _cHc0[1][1] + _cHc0[1][2]) / p_mu_t_2;
}  
  
/*!
  compute the homography using a displacement matrix.

  the homography is given by:

  \f$ {}^cH_{c_0} = {}^cR_{c_0} + \frac{{}^cT_{c_0} . {}^tN}{d_0} \f$

  Several internal variables are computed (dt, cRc0_0n)

  \param _cTc0 : the displacement matrix of the camera between the initial position of the camera and the current camera position
  \param _cHc0 : the homography of the plane
*/
void
vpMbtKltPolygon::computeHomography(const vpHomogeneousMatrix& _cTc0, vpHomography& _cHc0)
{
  vpRotationMatrix cRc0;
  vpTranslationVector ctransc0;

  _cTc0.extract(cRc0);
  _cTc0.extract(ctransc0);
  
//   vpGEMM(cRc0, 1.0, invd0, cRc0, -1.0, _cHc0, VP_GEMM_A_T);
  vpGEMM(ctransc0, N, -invd0, cRc0, 1.0, _cHc0, VP_GEMM_B_T);
  _cHc0 /= _cHc0[2][2];
  
  H = _cHc0;
  
//   vpQuaternionVector NQuat(N[0], N[1], N[2], 0.0);
//   vpQuaternionVector RotQuat(cRc0);
//   vpQuaternionVector RotQuatConj(-RotQuat.x(), -RotQuat.y(), -RotQuat.z(), RotQuat.w());
//   vpQuaternionVector partial = RotQuat * NQuat;
//   vpQuaternionVector resQuat = (partial * RotQuatConj);
//   
//   cRc0_0n = vpColVector(3);
//   cRc0_0n[0] = resQuat.x();
//   cRc0_0n[1] = resQuat.y();
//   cRc0_0n[2] = resQuat.z();
  
  cRc0_0n = cRc0*N;
  
//   vpPlane p(corners[0], corners[1], corners[2]);
//   vpColVector Ncur = p.getNormal();
//   Ncur.normalize();
  N_cur = cRc0_0n;
  dt = 0.0;
  for (unsigned int i = 0; i < 3; i += 1){
    dt += ctransc0[i] * (N_cur[i]);
  }
}

/*!
  Test whether the feature with identifier id in paramters is in the list of tracked
  features.

  \param _id : the id of the current feature to test
  \return true if the id is in the list of tracked feature
*/
bool
vpMbtKltPolygon::isTrackedFeature(const int _id)
{
//   std::map<int, vpImagePoint>::const_iterator iter = initPoints.begin();
//   while(iter != initPoints.end()){
//     if(iter->first == _id){
//       return true;
//     }
//     iter++;
//   }
  
  std::map<int, vpImagePoint>::iterator iter = initPoints.find(_id);
  if(iter != initPoints.end())
    return true;
  return false;
}

/*!
  Modification of all the pixels that are in the roi to the value of _nb (
  default is 255).

  \param _mask : the mask to update (0, not in the object, _nb otherwise).
  \param _nb : Optionnal value to set to the pixels included in the face.
  \param _shiftBorder : Optionnal shift for the border in pixel (sort of built-in erosion) to avoid to consider pixels near the limits of the face.
*/
void
vpMbtKltPolygon::updateMask(IplImage* _mask, unsigned int _nb, unsigned int _shiftBorder)
{
  int width = _mask->width;
  int i_min, i_max, j_min, j_max;
  std::vector<vpImagePoint> roi = getRoi(cam);
  vpMbtPolygon::getMinMaxRoi(roi, i_min, i_max, j_min,j_max);

  /* check image boundaries */
  if(i_min > _mask->height){ //underflow
    i_min = 0;
  }
  if(i_max > _mask->height){
    i_max = _mask->height;
  }
  if(j_min > _mask->width){ //underflow
    j_min = 0;
  }
  if(j_max > _mask->width){
    j_max = _mask->width;
  }
  double shiftBorder_d = (double) _shiftBorder;
  char* ptrData = _mask->imageData + i_min*width+j_min;
  for(int i=i_min; i< i_max; i++){
    double i_d = (double) i;
    for(int j=j_min; j< j_max; j++){
      double j_d = (double) j;
      if(_shiftBorder != 0){
        if( vpMbtKltPolygon::isInside(roi, i_d, j_d)
            && vpMbtKltPolygon::isInside(roi, i_d+shiftBorder_d, j_d+shiftBorder_d)
            && vpMbtKltPolygon::isInside(roi, i_d-shiftBorder_d, j_d+shiftBorder_d)
            && vpMbtKltPolygon::isInside(roi, i_d+shiftBorder_d, j_d-shiftBorder_d)
            && vpMbtKltPolygon::isInside(roi, i_d-shiftBorder_d, j_d-shiftBorder_d) ){
          *(ptrData++) = _nb;
        }
        else{
          ptrData++;
        }
      }
      else{
        if(vpMbtKltPolygon::isInside(roi, i, j)){
          *(ptrData++) = _nb;
        }
        else{
          ptrData++;
        }
      }
    }
    ptrData += width - j_max + j_min;
  }
}

/*!
  This method removes the outliers. A point is considered as outlier when its
  associated weight is below a given threshold (threshold_outlier).

  \param _w : Vector containing the weight of all the tracked points.
  \param threshold_outlier : Threshold to specify wether or not a point has to be deleted.
*/
void
vpMbtKltPolygon::removeOutliers(const vpColVector& _w, const double &threshold_outlier)
{
  std::map<int, vpImagePoint> tmp;
  unsigned int nbSupp = 0;
  unsigned int k = 0;
  
  nbPointsCur = 0;
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for( ; iter != curPoints.end(); iter++){
    if(_w[k] > threshold_outlier && _w[k+1] > threshold_outlier){
//     if(_w[k] > threshold_outlier || _w[k+1] > threshold_outlier){
      tmp[iter->first] = vpImagePoint(iter->second.get_i(), iter->second.get_j());
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
    curPoints = tmp;
    if(nbPointsCur >= minNbPoint) enoughPoints = true;
    else enoughPoints = false; 
  }
}

/*!
  Display the primitives tracked for the face.

  \param _I : The image where to display.
*/
void
vpMbtKltPolygon::displayPrimitive(const vpImage<unsigned char>& _I)
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
    vpDisplay::displayCharString(_I, iP, ide, vpColor::red);
  }
}

/*!
  Display the primitives tracked for the face.

  \param _I : The image where to display.
*/
void
vpMbtKltPolygon::displayPrimitive(const vpImage<vpRGBa>& _I)
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
    vpDisplay::displayCharString(_I, iP, ide, vpColor::red);
  }
}

//###################################
//      Static functions
//###################################

bool vpMbtKltPolygon::intersect(const vpImagePoint& p1, const vpImagePoint& p2, const double i_test, const double j_test, const double i, const double j)
{
  /* denominator */
  double dx = p2.get_j() - p1.get_j();
  double dy = p2.get_i() - p1.get_i();
  double ex = j - j_test;
  double ey = i - i_test;


  int den = (int)(dx * ey - dy * ex) ;
  double t = 0, u = 0;
  if(den != 0){
    t = -( p1.get_j() * ey - j_test*ey - ex * p1.get_i() + ex * i_test ) / den;
    u = -( -dx*p1.get_i() + dx * i_test + dy * p1.get_j() - dy * j_test ) / den;
  }
  else{
    throw vpException(vpException::divideByZeroError, "denominator null");
  }
  return ( t >=0 && t < 1 && u >= 0 && u < 1);
}

bool vpMbtKltPolygon::isInside(const std::vector<vpImagePoint>& roi, const double i, const double j)
{
  double i_test = 1000000.;
  double j_test = 1000000.;
  unsigned int nbInter = 0;
  bool computeAgain = true;

  if(computeAgain){
    computeAgain = false;
    for(unsigned int k=0; k< roi.size(); k++){
      try{
        if(vpMbtKltPolygon::intersect(roi[k], roi[(k+1)%roi.size()], i, j, i_test, j_test)){
          nbInter++;
        }
      }
      catch(...){
        computeAgain = true;
        break;
      }
    }

    if(computeAgain){
      i_test += 100;
      j_test -= 100;
      nbInter = 0;
    }
  }
  return ((nbInter%2) == 1);
}

#endif // VISP_HAVE_OPENCV
