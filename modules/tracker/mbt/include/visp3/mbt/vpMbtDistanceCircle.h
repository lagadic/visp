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
 * Manage a circle used in the model-based tracker.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
 \file vpMbtDistanceCircle.h
 \brief Manage a circle used in the model-based tracker.
*/

#ifndef vpMbtDistanceCircle_HH
#define vpMbtDistanceCircle_HH

#include <visp3/core/vpCircle.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/mbt/vpMbHiddenFaces.h>
#include <visp3/mbt/vpMbtMeEllipse.h>
#include <visp3/visual_features/vpFeatureEllipse.h>

/*!
  \class vpMbtDistanceCircle

  \brief Manage a circle used in the model-based tracker.

  \ingroup group_mbt_features
*/
class VISP_EXPORT vpMbtDistanceCircle
{
private:
  std::string name;
  unsigned int index;
  vpCameraParameters cam;
  vpMe *me;
  // double alpha;
  double wmean;
  vpFeatureEllipse featureEllipse;
  //! Polygon describing the circle bbox
  //    vpMbtPolygon poly;
  bool isTrackedCircle;

public:
  //! The moving edge containers
  vpMbtMeEllipse *meEllipse;

  //! The circle to track
  vpCircle *circle;

  //! The radius of the circle
  double radius;

  //! The center of the circle
  vpPoint *p1;
  //! A point on the plane containing the circle
  vpPoint *p2;
  //! An other point on the plane containing the circle
  vpPoint *p3;

  //! The interaction matrix
  vpMatrix L;
  //! The error vector
  vpColVector error;
  //! The number of moving edges
  unsigned int nbFeature;
  //! Indicates if the circle has to be reinitialized
  bool Reinit;
  //! Pointer to the list of faces
  vpMbHiddenFaces<vpMbtPolygon> *hiddenface;
  //! Index of the faces which contain the line
  int index_polygon;
  //! Indicates if the circle is visible or not
  bool isvisible;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //    vpMbtDistanceCircle(const vpMbtDistanceCircle &)
  //      : name(), index(0), cam(), me(NULL), wmean(1),
  //        featureEllipse(), isTrackedCircle(true), meEllipse(NULL),
  //        circle(NULL), radius(0.), p1(NULL), p2(NULL), p3(NULL),
  //        L(), error(), nbFeature(0), Reinit(false),
  //        hiddenface(NULL), index_polygon(-1), isvisible(false)
  //    {
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!");
  //    }
  //    vpMbtDistanceCircle &operator=(const vpMbtDistanceCircle &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  vpMbtDistanceCircle();
  ~vpMbtDistanceCircle();

  void buildFrom(const vpPoint &_p1, const vpPoint &_p2, const vpPoint &_p3, const double r);

  void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
  void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
  void displayMovingEdges(const vpImage<unsigned char> &I);

  /*!
   Get the camera paramters.

   \param camera : The vpCameraParameters used to store the camera parameters.
  */
  inline void getCameraParameters(vpCameraParameters &camera) { camera = this->cam; }

  /*!
    Get the index of the circle.

    \return Return the index of the line.
  */
  inline unsigned int getIndex() { return index; }

  /*!
   Get the mean weight of the circle. The mean weight is computed thanks to
   the weight of each moving edge. Those weights are computed by the robust
   estimation method used during the virtual visual servoing.

   \return The mean weight of the circle.
  */
  inline double getMeanWeight() const { return wmean; }

  /*!
    Get the name of the circle.

    \return Return the name of the circle.
  */
  inline std::string getName() const { return name; }

  /*!
   Get the polygon associated to the circle.

   \return poly.
  */
  //    inline vpMbtPolygon& getPolygon() {return poly;}

  void initInteractionMatrixError();

  bool initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
  /*!
   Return if the circle is used for tracking.

   \return True if it is used, False otherwise.
  */
  inline bool isTracked() const { return isTrackedCircle; }

  /*!
    Check if the circle is visible in the image or not.

    \return Return true if the circle is visible
  */
  inline bool isVisible() const { return isvisible; }

  void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  /*!
   Set the camera paramters.
   \param camera : The camera parameters.
  */
  inline void setCameraParameters(const vpCameraParameters &camera) { this->cam = camera; }

  /*!
    Set if the circle has to considered during tracking phase.

    \param track : True if the circle has to be tracked, False otherwise.
  */
  inline void setTracked(const bool &track) { this->isTrackedCircle = track; }

  /*!
    Set the index of the circle.

    \param i : The index number
  */
  inline void setIndex(const unsigned int i) { index = i; }

  /*!
   Set the mean weight of the circle.

   \param _wmean : The mean weight of the circle.
  */
  inline void setMeanWeight(const double _wmean) { this->wmean = _wmean; }

  void setMovingEdge(vpMe *Me);

  /*!
    Set the name of the circle.

    \param circle_name : The name of the circle.
  */
  inline void setName(const std::string &circle_name) { this->name = circle_name; }

  /*!
    Set the name of the circle.

    \param circle_name : The name of the circle.
  */
  inline void setName(const char *circle_name) { this->name = std::string(circle_name); }

  /*!
    Set a boolean parameter to indicates if the circle is visible in the image
    or not.

    \param _isvisible : Set to true if the circle is visible
  */
  inline void setVisible(bool _isvisible) { isvisible = _isvisible; }

  void trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  void updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

private:
  void project(const vpHomogeneousMatrix &cMo);
};

#endif
