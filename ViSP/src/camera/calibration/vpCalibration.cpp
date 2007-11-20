/****************************************************************************
 *
 * $Id: vpCalibration.cpp,v 1.4 2007-11-20 12:38:00 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Camera calibration.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 * Anthony Saunier
 *
 *****************************************************************************/


/*!
  \file vpCalibration.cpp
  \brief Tools for camera calibration.
*/

#include <visp/vpCalibration.h>
#include <visp/vpDebug.h>


const double vpCalibration::threshold = 1e-10f;
const unsigned int vpCalibration::nbIterMax = 3000;
/*!
  Basic initialisation (called by the constructors)
*/
int vpCalibration::init()
{
  npt = 0 ;

  LoX.kill() ;
  LoY.kill() ;
  LoZ.kill() ;
  Lu.kill() ;
  Lv.kill() ;

  return 0 ;
}

vpCalibration::vpCalibration()
{
  init() ;
}


/*!
  Destructor : delete the array of point (freed the memory)
*/
vpCalibration::~vpCalibration()
{
  clearPoint() ;
}

/*!
  = operator.

  \param twinCalibration : object to be copied
*/
void vpCalibration::operator=(vpCalibration& twinCalibration )
{
  npt = twinCalibration.npt ;
  LoX = twinCalibration.LoX ;
  LoY = twinCalibration.LoY ;
  LoZ = twinCalibration.LoZ ;
  Lu = twinCalibration.Lu ;
  Lv = twinCalibration.Lv ;

  residual = twinCalibration.residual;
  cMo = twinCalibration.cMo;
  residual_pm = twinCalibration.residual_pm;
  cMo_pm = twinCalibration.cMo_pm ;
  residual_mp = twinCalibration.residual_mp;
  cMo_mp = twinCalibration.cMo_mp ;

  cam = twinCalibration.cam ;

  wMe = twinCalibration.wMe;

  eMc = twinCalibration.eMc;
  eMc_mp = twinCalibration.eMc_mp;
  eMc_pm = twinCalibration.eMc_pm;
}


/*!
  Delete the array of points.
*/
int vpCalibration::clearPoint()
{
  LoX.kill() ;
  LoY.kill() ;
  LoZ.kill() ;
  Lu.kill() ;
  Lv.kill() ;
  npt = 0 ;

  return 0 ;
}

/*!
  Add a new point in the array of points.
  \param  X,Y,Z : 3D information of a point in the object frame
  \param u,v : 2D information (in pixel) of a point in the camera frame
*/
int vpCalibration::addPoint(double X, double Y, double Z, double u, double v)
{
  LoX += X ;
  LoY += Y ;
  LoZ += Z ;

  Lu += u ;
  Lv += v ;

  npt++ ;

  return 0 ;
}

/*!
  Compute and return the standard deviation expressed in pixel
  for pose matrix and camera intrinsic parameters for model without distortion.
  \param cMo : the matrix that defines the pose to be tested.
  \param cam : camera intrinsic parameters to be tested.
  \return the standard deviation by point of the error in pixel .
*/
double
vpCalibration::computeStdDeviation(vpHomogeneousMatrix& cMo,
				   vpCameraParameters& cam)
{
  double residual = 0 ;

  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  double u0 = cam.get_u0() ;
  double v0 = cam.get_v0() ;
  double px = cam.get_px() ;
  double py = cam.get_py() ;

  for (unsigned int i =0 ; i < npt ; i++)
  {

    double oX = LoX.value() ;
    double oY = LoY.value() ;
    double oZ = LoZ.value() ;

    double cX = oX*cMo[0][0]+oY*cMo[0][1]+oZ*cMo[0][2] + cMo[0][3];
    double cY = oX*cMo[1][0]+oY*cMo[1][1]+oZ*cMo[1][2] + cMo[1][3];
    double cZ = oX*cMo[2][0]+oY*cMo[2][1]+oZ*cMo[2][2] + cMo[2][3];

    double x = cX/cZ ;
    double y = cY/cZ ;

    double u = Lu.value() ;
    double v = Lv.value() ;

    double xp = u0 + x*px;
    double yp = v0 + y*py;

    residual += (vpMath::sqr(xp-u) + vpMath::sqr(yp-v))  ;

    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }
  this->residual = residual ;
  return sqrt(residual/npt) ;
}
/*!
  Compute and return the standard deviation expressed in pixel
  for pose matrix and camera intrinsic parameters with pixel to meter model.
  \param cMo : the matrix that defines the pose to be tested.
  \param cam : camera intrinsic parameters to be tested.
  \return the standard deviation by point of the error in pixel .
*/
double
vpCalibration::computeStdDeviation_pm(vpHomogeneousMatrix& cMo,
				      vpCameraParameters& cam)
{
  double residual = 0 ;

  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  double u0_pm = cam.get_u0_pm() ;
  double v0_pm = cam.get_v0_pm() ;
  double px_pm = cam.get_px_pm() ;
  double py_pm = cam.get_py_pm() ;
  double kd_pm = cam.get_kd_pm() ;

  for (unsigned int i =0 ; i < npt ; i++)
  {

    double oX = LoX.value() ;
    double oY = LoY.value() ;
    double oZ = LoZ.value() ;

    double cX = oX*cMo[0][0]+oY*cMo[0][1]+oZ*cMo[0][2] + cMo[0][3];
    double cY = oX*cMo[1][0]+oY*cMo[1][1]+oZ*cMo[1][2] + cMo[1][3];
    double cZ = oX*cMo[2][0]+oY*cMo[2][1]+oZ*cMo[2][2] + cMo[2][3];

    double x = cX/cZ ;
    double y = cY/cZ ;

    double u = Lu.value() ;
    double v = Lv.value() ;

    double r2 = (vpMath::sqr((u-u0_pm)/px_pm)+vpMath::sqr((v-v0_pm)/py_pm)) ;

    double xp = u0_pm + x*px_pm - kd_pm *(u-u0_pm)*r2;
    double yp = v0_pm + y*py_pm - kd_pm *(v-v0_pm)*r2;

    residual += (vpMath::sqr(xp-u) + vpMath::sqr(yp-v))  ;

    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }
  this->residual_pm = residual ;
  return sqrt(residual/npt) ;
}
/*!
  Compute and return the standard deviation expressed in pixel
  for pose matrix and camera intrinsic parameters with meter to pixel model.
  \param cMo : the matrix that defines the pose to be tested.
  \param cam : camera intrinsic parameters to be tested.
  \return the standard deviation by point of the error in pixel .
*/
double
vpCalibration::computeStdDeviation_mp(vpHomogeneousMatrix& cMo,
				      vpCameraParameters& cam)
{
  double residual = 0 ;

  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  double u0_mp = cam.get_u0_mp() ;
  double v0_mp = cam.get_v0_mp() ;
  double px_mp = cam.get_px_mp() ;
  double py_mp = cam.get_py_mp() ;
  double kd_mp = cam.get_kd_mp() ;

  for (unsigned int i =0 ; i < npt ; i++)
  {

    double oX = LoX.value() ;
    double oY = LoY.value() ;
    double oZ = LoZ.value() ;

    double cX = oX*cMo[0][0]+oY*cMo[0][1]+oZ*cMo[0][2] + cMo[0][3];
    double cY = oX*cMo[1][0]+oY*cMo[1][1]+oZ*cMo[1][2] + cMo[1][3];
    double cZ = oX*cMo[2][0]+oY*cMo[2][1]+oZ*cMo[2][2] + cMo[2][3];

    double x = cX/cZ ;
    double y = cY/cZ ;

    double u = Lu.value() ;
    double v = Lv.value() ;

    double r2 = (vpMath::sqr(x)+vpMath::sqr(y)) ;

    double xp = u0_mp + x*px_mp*(1 + kd_mp*r2);
    double yp = v0_mp + y*py_mp*(1 + kd_mp*r2);

    residual += (vpMath::sqr(xp-u) + vpMath::sqr(yp-v))  ;

    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }
  this->residual_mp = residual ;
  return sqrt(residual/npt) ;
}
/*!
  Compute and return the standard deviation expressed in pixel
  for pose matrix and camera intrinsic parameters with meter to pixel model
  and pixel to meter model.
  \param deviation   : the standard deviation computed for the model without distortion.
  \param deviation_pm : the standard deviation computed for the pixel to meter model.
  \param deviation_mp : the standard deviation computed for the meter to pixel model
*/
void
vpCalibration::computeStdDeviation(double &deviation,double &deviation_pm, double &deviation_mp)
{
  deviation   = computeStdDeviation(cMo,cam);
  deviation_pm = computeStdDeviation_pm(cMo_pm,cam);
  deviation_mp = computeStdDeviation_mp(cMo_mp,cam);
}
/*!
  Compute the calibration according to the desired method.

  \param method :
  the different method are

  CALIB_LAGRANGE         Lagrange approach.

  CALIB_VIRTUAL_VS       Virtual visual servoing approach.
  (results are similar to Lowe appraoch)
  CALIB_VIRTUAL_VS_DIST  Virtual visual servoing approach with distortion.

  CALIB_LAGRANGE_VIRTUAL_VS Lagrange approach first,
  than virtual visual servoing approach.

  CALIB_LAGRANGE_VIRTUAL_VS_DIST  Lagrange approach first,
  than virtual visual servoing approach,
  with distortion.
  \param cMo : the homogeneous matrix that defines the initial pose.
  \param cam : intrinsic camera parameters.
  \param verbose : set at true if information about the residual at each loop
  of the algorithm is hoped.

  \return 0 if the calibration computation managed.
*/
int
vpCalibration::computeCalibration(vpCalibrationMethod method,
				  vpHomogeneousMatrix &cMo,
				  vpCameraParameters &cam,
				  bool verbose)
{
  try{
    vpHomogeneousMatrix cMo_mp,cMo_pm ;
    switch (method)
    {
    case CALIB_LAGRANGE :
    case CALIB_LAGRANGE_VIRTUAL_VS :
      {
	calibLagrange(cam, cMo);
      }
      break;
    default:
      break;
    }

    switch (method)
    {
    case CALIB_VIRTUAL_VS:
    case CALIB_VIRTUAL_VS_DIST:
    case CALIB_LAGRANGE_VIRTUAL_VS:
    case CALIB_LAGRANGE_VIRTUAL_VS_DIST:
      {
	if (verbose){std::cout << "start calibration without distortion"<< std::endl;}
	calibVVS(cam, cMo, verbose);
	cMo_pm = cMo ;
	cMo_mp = cMo ;
      }
      break ;
    default:
      break;
    }

    switch (method)
    {
    case CALIB_VIRTUAL_VS_DIST:
    case CALIB_LAGRANGE_VIRTUAL_VS_DIST:
      {
	if (verbose){std::cout << "start calibration with distortion"<< std::endl;}
	if (verbose){std::cout << "For pixel to meter camera parameters :"<< std::endl;}
	calibVVSWithDistortion_pm(cam, cMo_pm, verbose);
	if (verbose){std::cout << "For meter to pixel camera parameters : "<< std::endl;}
	calibVVSWithDistortion_mp(cam, cMo_mp, verbose);
      }
      break ;
    default:
      break;
    }

    this->cam = cam ;

    this->cMo = cMo ;
    this->cMo_mp = cMo_mp ;
    this->cMo_pm = cMo_pm ;
    return 0 ;
  }
  catch(...){
    throw;
  }
}

/*!
  Compute the multi-images calibration according to the desired method.

  \param method : method to use to compute calibration
  the different method available here are
  CALIB_VIRTUAL_VS       Virtual visual servoing approach.
  (results are similar to Lowe appraoch)
  CALIB_VIRTUAL_VS_DIST  Virtual visual servoing approach with distortion.
  \param nbPose : number of images used to compute multi-images calibration
  \param table_cal : array of vpCalibration. All the vpHomogeneousMatrix cMo of
  each vpCalibration have to be initialized.
  \param cam : intrinsic camera parameters.
  \param verbose : set at true if information about the residual at each loop
  of the algorithm is hoped.

  \return 0 if the computation managed.
*/
int
vpCalibration::computeCalibrationMulti(vpCalibrationMethod method,unsigned int nbPose,
				       vpCalibration table_cal[],
				       vpCameraParameters& cam,
				       bool verbose)
{
  try{
    switch (method) {
    case CALIB_LAGRANGE :
      if(nbPose > 1){
	std::cout << "this calibration method is not available in" << std::endl
		  << "vpCalibration::computeCalibrationMulti()" << std::endl;
	return -1 ;
      }
      else {
	table_cal[0].calibLagrange(cam,table_cal[0].cMo);
	table_cal[0].cam = cam ;
	table_cal[0].cMo_pm = table_cal[0].cMo ;
	table_cal[0].cMo_mp = table_cal[0].cMo ;
      }
      break;
    case CALIB_LAGRANGE_VIRTUAL_VS :
    case CALIB_LAGRANGE_VIRTUAL_VS_DIST :
      if(nbPose > 1){
	std::cout << "this calibration method is not available in" << std::endl
		  << "vpCalibration::computeCalibrationMulti()" << std::endl
		  << "with several images." << std::endl;
	return -1 ;
      }
      else {
	table_cal[0].calibLagrange(cam,table_cal[0].cMo);
	table_cal[0].cam = cam ;
	table_cal[0].cMo_pm = table_cal[0].cMo ;
	table_cal[0].cMo_mp = table_cal[0].cMo ;
      }
    case CALIB_VIRTUAL_VS:
    case CALIB_VIRTUAL_VS_DIST:
      {
        calibVVSMulti(nbPose, table_cal, cam, verbose);
      }
      break ;
    }
    //Print camera parameters
    if(verbose){
      std::cout << "Camera parameters without distortion :" << std::endl;
      cam.printParameters(1);
    }

    switch (method)
    {
    case CALIB_LAGRANGE :
    case CALIB_LAGRANGE_VIRTUAL_VS :
    case CALIB_VIRTUAL_VS:
      verbose = false ;
      break;
    case CALIB_LAGRANGE_VIRTUAL_VS_DIST :
    case CALIB_VIRTUAL_VS_DIST:
      {
	if(verbose)
	  std::cout << "Compute camera parameters with distortion"<<std::endl
		    << "for pixel to meter model:" << std::endl;

	calibVVSWithDistortionMulti_pm(nbPose, table_cal, cam, verbose);

	if(verbose)
	  std::cout << "Compute camera parameters with distortion"<<std::endl
		    << "for meter to pixel model:" << std::endl;

	calibVVSWithDistortionMulti_mp(nbPose, table_cal, cam, verbose);
      }
      break ;
    }
    //Print camera parameters
    if(verbose){
      std::cout << "Camera parameters :" << std::endl;
      cam.printParameters(1);
      std::cout<<std::endl;
    }

    return 0 ;
  }
  catch(...){ throw; }
}

/*!
  Compute the multi-image Tsai calibration.

  \param nbPose : number of images used to compute multi-images calibration
  \param table_cal : array of vpCalibration. All the vpHomogeneousMatrix cMo
  and wMe of each vpCalibration have to be initialized.
  \param eMc : output: estimated pose of the camera in relation to the effector
  with the camera model without distortion.
  \param eMc_mp : output: estimated pose of the camera in relation to the effector
  with the meter based distortion model.
  \param eMc_pm : output: estimated pose of the camera in relation to the effector
  with the pixel based distortion model.
  \return 0 if the computation managed.
*/
int
vpCalibration::computeCalibrationTsai(unsigned int nbPose,
                                      vpCalibration table_cal[],
                                      vpHomogeneousMatrix& eMc,
                                      vpHomogeneousMatrix& eMc_mp,
                                      vpHomogeneousMatrix& eMc_pm)
{
  vpHomogeneousMatrix* table_cMo = new vpHomogeneousMatrix[nbPose];
  vpHomogeneousMatrix* table_cMo_mp = new vpHomogeneousMatrix[nbPose];
  vpHomogeneousMatrix* table_cMo_pm = new vpHomogeneousMatrix[nbPose];
  vpHomogeneousMatrix* table_wMe = new vpHomogeneousMatrix[nbPose];
  try{
    if (nbPose > 2){
      for(unsigned int i=0;i<nbPose;i++){
        table_cMo[i] = table_cal[i].cMo;
        table_cMo_mp[i] = table_cal[i].cMo_mp;
        table_cMo_pm[i] = table_cal[i].cMo_pm;
        table_wMe[i] = table_cal[i].wMe;
      }
      calibrationTsai(nbPose,table_cMo,table_wMe,eMc);
      calibrationTsai(nbPose,table_cMo_mp,table_wMe,eMc_mp);
      calibrationTsai(nbPose,table_cMo_pm,table_wMe,eMc_pm);
      delete [] table_cMo;
      delete [] table_cMo_mp;
      delete [] table_cMo_pm;
      delete [] table_wMe;
      return 0;
    }
    else{
      vpERROR_TRACE("Three images are needed to compute Tsai calibration !\n");
      delete [] table_cMo;
      delete [] table_cMo_mp;
      delete [] table_cMo_pm;
      delete [] table_wMe;
      return -1;
    }
  }
  catch(...){
    delete [] table_cMo;
    delete [] table_cMo_mp;
    delete [] table_cMo_pm;
    delete [] table_wMe;
    throw;
  }
}


/*!
  Write data into a file.

  data are organized as follow oX oY oZ u v

  \param filename : name of the file
*/
int
vpCalibration::writeData(const char *filename)
{
  std::ofstream f(filename) ;

  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  f.precision(10);
  f.setf(std::ios::fixed,std::ios::floatfield);
  f << LoX.nbElement() << std::endl ;

  for (int i =0 ; i < LoX.nbElement() ; i++)
  {

    double oX = LoX.value() ;
    double oY = LoY.value() ;
    double oZ = LoZ.value() ;

    double u = Lu.value() ;
    double v = Lv.value() ;


    f << oX <<" " << oY << " " << oZ << " " << u << "  " << v <<std::endl ;

    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;

  }

  f.close() ;
  return 0 ;
}

/*!
  Read data from disk :
  data are organized as follow oX oY oZ u v

  \param filename : name of the file
*/
int
vpCalibration::readData(const char* filename)
{
  std::ifstream f;
  f.open(filename);
  if (f != NULL){
    int n ;
    f >> n ;
    std::cout << "There are "<< n <<" point on the calibration grid " << std::endl ;

    clearPoint() ;
    for (int i=0 ; i < n ; i++)
    {
      double x, y, z, u, v ;
      f >> x >> y >> z >> u >> v ;
      std::cout << x <<" "<<y <<" "<<z<<" "<<u<<" "<<v <<std::endl ;
      addPoint(x,y,z,u,v) ;
    }

    f.close() ;
    return 0 ;
  }
  else{
    return -1;
  }
}
/*!
  Read calibration grid coordinates from disk :
  data are organized as follow oX oY oZ

  \param filename : name of the file
  \param n : number of points in the calibration grid
  \param oX : vpList of oX coordinates
  \param oY : vpList of oY coordinates
  \param oZ : vpList of oZ coordinates

  \return 0 : if success
*/
int
vpCalibration::readGrid(const char* filename,unsigned int &n,
			vpList<double> &oX,vpList<double> &oY,vpList<double> &oZ)
{
  try{
    std::ifstream f;
    f.open(filename);
    if (f != NULL){

      f >> n ;
      std::cout << "There are "<< n <<" points on the calibration grid " << std::endl ;
      int no_pt;
      double x,y,z;

      oX.front();
      oY.front();
      oZ.front();
      for (unsigned int i=0 ; i < n ; i++)
      {
        f >> no_pt >> x >> y >> z ;
        std::cout << no_pt <<std::endl ;
        std::cout << x <<"  "<< y <<"  "<< z <<std::endl ;
        oX.addRight(x) ;
        oY.addRight(y) ;
        oZ.addRight(z) ;
      }

      f.close() ;
    }
    else{
      return -1;
    }
  }
  catch(...){return -1;}
  return 0 ;
}

/*!
  Display the data of the calibration (center of the tracked dots)
  \param I : Image where to display data.
  \param col : color of the data.
*/
int
vpCalibration::displayData(vpImage<unsigned char> &I, vpColor::vpColorType col)
{

  Lu.front() ;
  Lv.front() ;


  for (unsigned int i =0 ; i < npt ; i++)
  {
    double u = Lu.value() ;
    double v = Lv.value() ;

    vpDisplay::displayCross(I,(int)vpMath::round(v),(int)vpMath::round(u),10,col) ;


    Lu.next() ;
    Lv.next() ;
  }
  return 0 ;
}

/*!
  Display estimated centers of dots using intrinsic camera parameters
  with meter to pixel model and the computed pose.
  \param I : Image where to display grid data.
  \param col : color of the data.
*/
int
vpCalibration::displayGrid(vpImage<unsigned char> &I, vpColor::vpColorType col)
{
  double u0 = cam.get_u0_mp() ;
  double v0 = cam.get_v0_mp() ;
  double px = cam.get_px_mp() ;
  double py = cam.get_py_mp() ;
  double kd = cam.get_kd_mp() ;

  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;

  for (unsigned int i =0 ; i < npt ; i++)
  {

    double oX = LoX.value() ;
    double oY = LoY.value() ;
    double oZ = LoZ.value() ;

    double cX = oX*cMo_mp[0][0]+oY*cMo_mp[0][1]+oZ*cMo_mp[0][2] + cMo_mp[0][3];
    double cY = oX*cMo_mp[1][0]+oY*cMo_mp[1][1]+oZ*cMo_mp[1][2] + cMo_mp[1][3];
    double cZ = oX*cMo_mp[2][0]+oY*cMo_mp[2][1]+oZ*cMo_mp[2][2] + cMo_mp[2][3];

    double x = cX/cZ ;
    double y = cY/cZ ;

    double r2 = vpMath::sqr(x)+vpMath::sqr(y) ;

    double xp = u0 + x*px*(1+kd*r2) ;
    double yp = v0 + y*py*(1+kd*r2) ;

    vpDisplay::displayCross(I,(int)vpMath::round(yp), (int)vpMath::round(xp), 5,col) ;

    //    std::cout << oX << "  " << oY <<  "  " <<oZ << std::endl ;
    //    I.getClick() ;
    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
  }
  return 0;
}
