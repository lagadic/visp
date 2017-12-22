/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <list>
#include <vector>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/vision/vpCalibrationException.h>
/*!
  \class vpCalibration

  \ingroup group_vision_calib

  \brief Tools for perspective camera calibration.

*/
class VISP_EXPORT vpCalibration
{
public:
  /*!
    Minimization algorithm use to estimate the camera parameters.
  */
  typedef enum {
    CALIB_LAGRANGE,                 /*!< Lagrange approach without estimation of the
                                       distortion. */
    CALIB_VIRTUAL_VS,               /*!< Virtual visual servoing approach without estimation
                                       of the distortion (results are similar to Lowe
                                       approach). */
    CALIB_VIRTUAL_VS_DIST,          /*!< Virtual visual servoing approach with
                                       estimation of the distortion. */
    CALIB_LAGRANGE_VIRTUAL_VS,      /*!< Lagrange approach first, than virtual
                                       visual servoing approach,  without
                                       estimation of the distortion. */
    CALIB_LAGRANGE_VIRTUAL_VS_DIST, /*!< Lagrange approach first, than virtual
                                       visual servoing approach, with
                                       estimation of the distortion. */
  } vpCalibrationMethodType;

  vpHomogeneousMatrix cMo; //!< the pose computed for the model without distortion
  //!< (as a 3x4 matrix [R T])
  vpHomogeneousMatrix cMo_dist; //!< the pose computed for perspective projection
  //!< with distortion model
  //!< (as a 3x4 matrix [R T])
  vpCameraParameters cam; //!< camera intrinsic parameters for perspective
  //!< projection model without distortion
  vpCameraParameters cam_dist; //!< camera intrinsic parameters for perspective
  //!< projection model with distortion

  vpHomogeneousMatrix rMe; //!< position of the effector in relation to the
  //!< reference coordinates (manipulator base coordinates)
  vpHomogeneousMatrix eMc; //!< position of the camera in relation to the effector
  vpHomogeneousMatrix eMc_dist;

public:
  // Constructor
  vpCalibration();
  vpCalibration(const vpCalibration &c);

  // Destructor
  virtual ~vpCalibration();

  // Add a new point in this array
  int addPoint(double X, double Y, double Z, vpImagePoint &ip);

  // = operator
  vpCalibration &operator=(const vpCalibration &twinCalibration);

  static void calibrationTsai(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                              vpHomogeneousMatrix &eMc);

  //! Suppress all the point in the array of point
  int clearPoint();

  void computeStdDeviation(double &deviation, double &deviation_dist);
  int computeCalibration(vpCalibrationMethodType method, vpHomogeneousMatrix &cMo_est, vpCameraParameters &cam_est,
                         bool verbose = false);
  static int computeCalibrationMulti(vpCalibrationMethodType method, std::vector<vpCalibration> &table_cal,
                                     vpCameraParameters &cam, double &globalReprojectionError, bool verbose = false);

  static int computeCalibrationTsai(const std::vector<vpCalibration> &table_cal, vpHomogeneousMatrix &eMc,
                                    vpHomogeneousMatrix &eMc_dist);
  double computeStdDeviation(const vpHomogeneousMatrix &cMo_est, const vpCameraParameters &camera);
  double computeStdDeviation_dist(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam);
  int displayData(vpImage<unsigned char> &I, vpColor color = vpColor::red, unsigned int thickness = 1,
                  int subsampling_factor = 1);
  int displayGrid(vpImage<unsigned char> &I, vpColor color = vpColor::yellow, unsigned int thickness = 1,
                  int subsampling_factor = 1);

  //! Set the gain for the virtual visual servoing algorithm.
  static double getLambda() { return gain; }

  //! get the residual in pixels
  double getResidual(void) const { return residual; }
  //! get the residual for perspective projection with distortion (in pixels)
  double getResidual_dist(void) const { return residual_dist; }
  //! get the number of points
  unsigned int get_npt() const { return npt; }

  int init();

  int readData(const char *filename);
  static int readGrid(const char *filename, unsigned int &n, std::list<double> &oX, std::list<double> &oY,
                      std::list<double> &oZ, bool verbose = false);

  //! set the gain for the virtual visual servoing algorithm
  static void setLambda(const double &lambda) { gain = lambda; }
  int writeData(const char *filename);

private:
  void computePose(const vpCameraParameters &cam, vpHomogeneousMatrix &cMo);
  void calibLagrange(vpCameraParameters &cam, vpHomogeneousMatrix &cMo);

  //! Compute the calibration using virtual visual servoing approach
  void calibVVS(vpCameraParameters &cam, vpHomogeneousMatrix &cMo, bool verbose = false);

  static void calibVVSMulti(unsigned int nbPose, vpCalibration table_cal[], vpCameraParameters &cam,
                            bool verbose = false);
  static void calibVVSMulti(std::vector<vpCalibration> &table_cal, vpCameraParameters &cam,
                            double &globalReprojectionError, bool verbose = false);
  void calibVVSWithDistortion(vpCameraParameters &cam, vpHomogeneousMatrix &cMo, bool verbose = false);
  static void calibVVSWithDistortionMulti(unsigned int nbPose, vpCalibration table_cal[], vpCameraParameters &cam,
                                          bool verbose = false);
  static void calibVVSWithDistortionMulti(std::vector<vpCalibration> &table_cal, vpCameraParameters &cam,
                                          double &globalReprojectionError, bool verbose = false);

private:
  unsigned int npt; //!< number of points used in calibration computation
  std::list<double> LoX, LoY,
      LoZ;                     //!< list of points coordinates (3D in meters)
  std::list<vpImagePoint> Lip; //!< list of points coordinates (2D in pixels)

  double residual;      //!< residual in pixel for camera model without distortion
  double residual_dist; //!< residual in pixel for perspective projection with
                        //!< distortion model

  static double threshold;
  static unsigned int nbIterMax;
  static double gain;
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
