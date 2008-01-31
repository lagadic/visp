/****************************************************************************
 *
 * $Id: vpCalibration.h,v 1.5 2008-01-31 14:43:50 asaunier Exp $
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
  \file vpCalibration.h
  \brief Tools for camera calibration.
  
  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)

  \sa the example in calibrate.cpp
*/
#ifndef vpCalibration_h
#define vpCalibration_h

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMath.h>
#include <visp/vpList.h>
#include <visp/vpDisplay.h>
#include <visp/vpImage.h>
#include <visp/vpCalibrationException.h>
/*!
  \class vpCalibration
  \brief Tools for perspective camera calibration.

*/
class VISP_EXPORT vpCalibration
{
public: 
  typedef enum{
    CALIB_LAGRANGE,
    CALIB_VIRTUAL_VS,
    CALIB_VIRTUAL_VS_DIST,
    CALIB_LAGRANGE_VIRTUAL_VS,
    CALIB_LAGRANGE_VIRTUAL_VS_DIST,
  } vpCalibrationMethodType ;
private:
  unsigned int npt ;       //!< number of points used in calibration computation
  vpList<double> LoX, LoY, LoZ  ;  //!< list of points coordinates (3D in meters)
  vpList<double> Lu, Lv ; //!< list of points coordinates (2D in pixels)

  double residual ; //!< residual in pixel for camera model without distortion
  double residual_dist ;     //!< residual in pixel for perspective projection
                             //!< with distortion model 
public:
  vpHomogeneousMatrix cMo ;    //!< the pose computed for the model without distortion
                               //!< (as a 3x4 matrix [R T])
  vpHomogeneousMatrix cMo_dist ;  //!< the pose computed for perspective projection
                                  //!< with distortion model 
                                  //!< (as a 3x4 matrix [R T])
  vpCameraParameters cam;   //!< camera intrinsic parameters for perspective
                            //!< projection model without distortion 
  vpCameraParameters cam_dist; //!< camera intrinsic parameters for perspective
                            //!< projection model with distortion

  vpHomogeneousMatrix rMe; //!< position of the effector in relation to the
                           //!< reference coordinates (manipulator base coordinates)
  vpHomogeneousMatrix eMc; //!< position of the camera in relation to the effector
  vpHomogeneousMatrix eMc_dist;
public:
  int init() ;

  //! Suppress all the point in the array of point
  int clearPoint() ;
  //! Add a new point in this array
  int addPoint(double X, double Y, double Z, double u, double v) ;

  //! Constructor
  vpCalibration() ;
   
  //! Destructor
  virtual ~vpCalibration() ;
  //! = operator
  void operator=(vpCalibration& twinCalibration );
private:
  void calibLagrange( vpCameraParameters &cam , vpHomogeneousMatrix &cMo) ;
  
  //! Compute the calibration using virtual visual servoing approach
  void calibVVS( vpCameraParameters &cam , vpHomogeneousMatrix &cMo,
                      bool verbose = false) ;
  static void calibVVSMulti(unsigned int nbPose, vpCalibration table_cal[] ,
                     vpCameraParameters &cam, bool verbose = false) ;
  void calibVVSWithDistortion( vpCameraParameters &cam,
                               vpHomogeneousMatrix &cMo,
                               bool verbose = false) ;
  static void calibVVSWithDistortionMulti( unsigned int nbPose,
                                    vpCalibration table_cal[],
                                    vpCameraParameters &cam,
                                    bool verbose = false );
  static const double threshold;
  static const unsigned int nbIterMax;


public:
  //!get the residual in pixels
  double getResidual(void) const {return residual;}
  //!get the residual for perspective projection with distortion (in pixels)
  double getResidual_dist(void) const {return residual_dist;}
  //!get the number of points
  unsigned int get_npt() const {return npt;}
  
  int readData(const char *filename) ;
  static int readGrid(const char *filename,unsigned int &n,
                  vpList<double> &oX,vpList<double> &oY,vpList<double> &oZ);
  int writeData(const char *filename) ;
  int displayData(vpImage<unsigned char> &I,
                      vpColor::vpColorType col=vpColor::red) ;
  int displayGrid(vpImage<unsigned char> &I,
                      vpColor::vpColorType col=vpColor::blue) ;
  
  double computeStdDeviation(vpHomogeneousMatrix &cMo,
                          vpCameraParameters &cam);
  double computeStdDeviation_dist(vpHomogeneousMatrix &cMo,
                          vpCameraParameters &cam);
  void computeStdDeviation(double &deviation, double &deviation_dist);
  //! Compute the calibration for a given method
  int computeCalibration(vpCalibrationMethodType method,
			  vpHomogeneousMatrix &cMo,
        vpCameraParameters &cam,
        bool verbose = false) ;
  //! Compute the calibration for a given method using many poses
  static int computeCalibrationMulti(vpCalibrationMethodType method,unsigned int nbPose,
        vpCalibration table_cal[],
        vpCameraParameters &cam,
        bool verbose = false) ;

  static void calibrationTsai(unsigned int nbPose, vpHomogeneousMatrix cMo[],
                                    vpHomogeneousMatrix rMe[],
                                    vpHomogeneousMatrix &eMc);
  static int computeCalibrationTsai(unsigned int nbPose,
                                    vpCalibration table_cal[],
                                    vpHomogeneousMatrix &eMc,
                                    vpHomogeneousMatrix &eMc_dist);
  int writeCalibrationParameters(char *filename) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
