/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpCalibration.h
 * \brief Tools for camera calibration.
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

BEGIN_VISP_NAMESPACE
/*!
 *  \class vpCalibration
 *
 * \ingroup group_vision_calib
 *
 * \brief Tools for perspective camera calibration.
*/
class VISP_EXPORT vpCalibration
{
public:
  /*!
   * Minimization algorithm use to estimate the camera parameters.
   */
  typedef enum
  {
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

  /*!
   * Pose computed using camera parameters without distortion (as a 3x4 matrix [R T])
   */
  vpHomogeneousMatrix cMo;
  /*!
   * Pose computed using camera parameters with distortion with distortion model
   * (as a 3x4 matrix [R T])
   */
  vpHomogeneousMatrix cMo_dist;
  /*!
   * Camera intrinsic parameters for perspective projection model without distortion
   */
  vpCameraParameters cam;
  /*!
   * Camera intrinsic parameters for perspective projection model with distortion
   */
  vpCameraParameters cam_dist;

  /*!
   * Position of the effector in relation to the reference coordinates (manipulator base coordinates)
   */
  vpHomogeneousMatrix rMe;
  /*!
   * Position of the camera in end-effector frame using camera parameters without distortion
   */
  vpHomogeneousMatrix eMc;
  /*!
   * Position of the camera in end-effector frame using camera parameters with distortion
   */
  vpHomogeneousMatrix eMc_dist;

  /*!
   * Fix aspect ratio (px/py)
   */
  double m_aspect_ratio;

  /*!
   * Default constructor.
   */
  vpCalibration();

  /*!
   * Copy constructor.
   */
  vpCalibration(const vpCalibration &c);

  /*!
   * Destructor : delete the array of point (freed the memory)
   */
  virtual ~vpCalibration();

  /*!
   * Add a new point in the array of points.
   * \param  X,Y,Z : 3D coordinates of a point in the object frame
   * \param ip : 2D Coordinates of the point in the camera frame.
   */
  int addPoint(double X, double Y, double Z, vpImagePoint &ip);

  /*!
   * Copy operator.
   *
   * \param twinCalibration : object to be copied
   */
  vpCalibration &operator=(const vpCalibration &twinCalibration);

  /*!
   * Suppress all the point in the array of point.
   */
  int clearPoint();

  /*!
   * Compute the calibration according to the desired method using one pose.
   *
   * \param method : Method that will be used to estimate the parameters.
   * \param cMo_est : estimated homogeneous matrix that defines the pose.
   * \param cam_est : estimated intrinsic camera parameters.
   * \param verbose : set at true if information about the residual at each loop
   * of the algorithm is hoped.
   *
   * \return EXIT_SUCCESS if the calibration succeed, EXIT_FAILURE otherwise.
   */
  int computeCalibration(vpCalibrationMethodType method, vpHomogeneousMatrix &cMo_est, vpCameraParameters &cam_est,
                         bool verbose = false);

  /*!
   * Compute the multi-images calibration according to the desired method using
   * many poses.
   *
   * \param method : Method used to estimate the camera parameters.
   * \param table_cal : Vector of vpCalibration.
   * \param cam_est : Estimated intrinsic camera parameters.
   * \param globalReprojectionError : Global reprojection error or global
   * residual.
   * \param verbose : Set at true if information about the residual at
   * each loop of the algorithm is hoped.
   *
   * \return EXIT_SUCCESS if the calibration succeed, EXIT_FAILURE otherwise.
   */
  static int computeCalibrationMulti(vpCalibrationMethodType method, std::vector<vpCalibration> &table_cal,
                                     vpCameraParameters &cam_est, double &globalReprojectionError, bool verbose = false);

  /*!
   * Compute and return the standard deviation expressed in pixel
   * for pose matrix and camera intrinsic parameters.
   * \param deviation   : the standard deviation computed for the model without
   * distortion.
   * \param deviation_dist : the standard deviation computed for the
   * model with distortion.
   */
  void computeStdDeviation(double &deviation, double &deviation_dist);

  /*!
   * Compute and return the standard deviation expressed in pixel
   * for pose matrix and camera intrinsic parameters for model without
   * distortion.
   *
   * \param cMo_est : the matrix that defines the pose to be tested.
   * \param camera : camera intrinsic parameters to be tested.
   * \return the standard deviation by point of the error in pixel .
   */
  double computeStdDeviation(const vpHomogeneousMatrix &cMo_est, const vpCameraParameters &camera);

  /*!
   * Compute and return the standard deviation expressed in pixel
   * for pose matrix and camera intrinsic parameters with pixel to meter model.
   *
   * \param cMo_est : the matrix that defines the pose to be tested.
   * \param camera : camera intrinsic parameters to be tested.
   * \return the standard deviation by point of the error in pixel .
   */
  double computeStdDeviation_dist(const vpHomogeneousMatrix &cMo_est, const vpCameraParameters &camera);

  /*!
   * Display the data of the calibration (center of the tracked dots).
   *
   * \param I : Image where to display data.
   * \param color : Color of the data.
   * \param thickness : Thickness of the displayed data.
   * \param subsampling_factor : Subsampling factor. Default value is 1.
   * Admissible values are multiple of 2. Divide by this parameter the
   * coordinates of the data points resulting from image processing.
   */
  int displayData(vpImage<unsigned char> &I, vpColor color = vpColor::red, unsigned int thickness = 1,
                  int subsampling_factor = 1);

  /*!
   * Display estimated centers of dots using intrinsic camera parameters
   * with model with distortion and the computed pose.
   * \param I : Image where to display grid data.
   * \param color : Color of the data.
   * \param thickness : Thickness of the displayed data.
   * \param subsampling_factor : Subsampling factor. Default value is 1.
   * Admissible values are multiple of 2. Divide by this parameter the
   * values of the camera parameters.
   */
  int displayGrid(vpImage<unsigned char> &I, vpColor color = vpColor::yellow, unsigned int thickness = 1,
                  int subsampling_factor = 1);

  //! Get the gain of the virtual visual servoing algorithm.
  static double getLambda() { return m_gain; }

  /*!
   * Get the residual in pixels.
   */
  double getResidual(void) const { return m_residual; }

  /*!
   * Get the residual for perspective projection with distortion (in pixels).
   */
  double getResidual_dist(void) const { return m_residual_dist; }

  /*!
   * Get the number of points.
   */
  unsigned int get_npt() const { return m_npt; }

  /*!
   * Basic initialisation (called by the constructors).
   */
  int init();

  /*!
   * Read data from disk :
   * data are organized as follow oX oY oZ u v
   *
   * \param filename : Name of the file.
   */
  int readData(const std::string &filename);

  /*!
   * Read calibration grid coordinates from disk.
   * Data are organized as follow oX oY oZ
   *
   * \param filename : Name of the file.
   * \param n : Number of points in the calibration grid.
   * \param oX : List of oX coordinates.
   * \param oY : List of oY coordinates.
   * \param oZ : List of oZ coordinates.
   * \param verbose : Additional printings if true (number of points on
   * the calibration grid and their respective coordinates in the object
   * frame).
   *
   * \return 0 if success, -1 if an error occurs.
  */
  static int readGrid(const std::string &filename, unsigned int &n, std::list<double> &oX, std::list<double> &oY,
                      std::list<double> &oZ, bool verbose = false);

  /*!
   * Set the gain for the virtual visual servoing algorithm.
   */
  static void setLambda(const double &lambda) { m_gain = lambda; }

  /*!
   * Set pixel aspect ratio px/py.
   *
   * \param[in] aspect_ratio : px/py aspect ratio. Value need to be positive.
   * To estimate a model where px=py set 1 as aspect ratio.
   */
  void setAspectRatio(double aspect_ratio);

  /*!
   * Write data into a file.
   *
   * Data are organized as follow oX oY oZ u v
   *
   * \param filename : Name of the file.
   */
  int writeData(const std::string &filename);

private:
  void computePose(const vpCameraParameters &cam, vpHomogeneousMatrix &cMo);
  void calibLagrange(vpCameraParameters &cam, vpHomogeneousMatrix &cMo);

  //! Compute the calibration using virtual visual servoing approach
  void calibVVS(vpCameraParameters &cam, vpHomogeneousMatrix &cMo, bool verbose = false);

  static void calibVVSMulti(unsigned int nbPose, vpCalibration table_cal[], vpCameraParameters &cam,
                            bool verbose = false, double aspect_ratio = -1);
  static void calibVVSMulti(std::vector<vpCalibration> &table_cal, vpCameraParameters &cam,
                            double &globalReprojectionError, bool verbose = false, double aspect_ratio = -1);
  void calibVVSWithDistortion(vpCameraParameters &cam, vpHomogeneousMatrix &cMo, bool verbose = false);
  static void calibVVSWithDistortionMulti(unsigned int nbPose, vpCalibration table_cal[], vpCameraParameters &cam,
                                          bool verbose = false, double aspect_ratio = -1);
  static void calibVVSWithDistortionMulti(std::vector<vpCalibration> &table_cal, vpCameraParameters &cam,
                                          double &globalReprojectionError, bool verbose = false,
                                          double aspect_ratio = -1);

  unsigned int m_npt; //!< number of points used in calibration computation
  std::list<double> m_LoX, m_LoY, m_LoZ; //!< list of points coordinates (3D in meters)
  std::list<vpImagePoint> m_Lip; //!< list of points coordinates (2D in pixels)

  double m_residual;      //!< residual in pixel for camera model without distortion
  double m_residual_dist; //!< residual in pixel for perspective projection with
                        //!< distortion model

  static double m_threshold;
  static unsigned int m_nbIterMax;
  static double m_gain;
};
END_VISP_NAMESPACE
#endif
