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
 * Manage a cylinder used in the model-based tracker.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Bertrand Delabarre
 *
 *****************************************************************************/

/*!
 \file vpMbtDistanceCylinder.h
 \brief Manage a cylinder used in the model-based tracker.
*/

#ifndef vpMbtDistanceCylinder_HH
#define vpMbtDistanceCylinder_HH

#include <visp3/core/vpCircle.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpPoint.h>
#include <visp3/mbt/vpMbHiddenFaces.h>
#include <visp3/mbt/vpMbtMeLine.h>
#include <visp3/visual_features/vpFeatureLine.h>

/*!
  \class vpMbtDistanceCylinder

  \brief Manage a cylinder used in the model-based tracker.

  \ingroup group_mbt_features
*/
class VISP_EXPORT vpMbtDistanceCylinder
{
private:
  std::string name;
  unsigned int index;
  vpCameraParameters cam;
  vpMe *me;
  double wmean1;
  double wmean2;
  vpFeatureLine featureline1;
  vpFeatureLine featureline2;
  bool isTrackedCylinder;

public:
  //! The moving edge containers (first line of the cylinder)
  vpMbtMeLine *meline1;
  //! The moving edge containers (second line of the cylinder)
  vpMbtMeLine *meline2;

  //! The upper circle limiting the cylinder
  vpCircle *cercle1;
  //! The lower circle limiting the cylinder
  vpCircle *cercle2;

  //! The radius of the cylinder
  double radius;

  //! The first extremity on the axe
  vpPoint *p1;
  //! The second extremity on the axe
  vpPoint *p2;

  //! The interaction matrix
  vpMatrix L;
  //! The error vector
  vpColVector error;
  //! The number of moving edges
  unsigned int nbFeature;
  //! The number of moving edges on line 1
  unsigned int nbFeaturel1;
  //! The number of moving edges on line 2
  unsigned int nbFeaturel2;
  //! Indicates if the line has to be reinitialized
  bool Reinit;
  //! The cylinder
  vpCylinder *c;

  //! Pointer to the list of faces
  vpMbHiddenFaces<vpMbtPolygon> *hiddenface;
  //! Index of the face which contains the cylinder
  int index_polygon;
  //! Indicates if the cylinder is visible or not
  bool isvisible;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //    vpMbtDistanceCylinder(const vpMbtDistanceCylinder &)
  //      : name(), index(0), cam(), me(NULL), wmean1(1), wmean2(1),
  //        featureline1(), featureline2(), isTrackedCylinder(true),
  //        meline1(NULL), meline2(NULL), cercle1(NULL), cercle2(NULL),
  //        radius(0), p1(NULL), p2(NULL), L(), error(), nbFeature(0),
  //        nbFeaturel1(0), nbFeaturel2(0), Reinit(false), c(NULL),
  //        hiddenface(NULL), index_polygon(-1), isvisible(false)
  //    {
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!");
  //    }
  //    vpMbtDistanceCylinder &operator=(const vpMbtDistanceCylinder &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  vpMbtDistanceCylinder();
  ~vpMbtDistanceCylinder();

  void buildFrom(const vpPoint &_p1, const vpPoint &_p2, const double r);

  void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I);

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
    Get the index of the cylinder.

    \return Return the index of the line.
  */
  inline unsigned int getIndex() { return index; }

  /*!
   Get the mean weight of the first line. The mean weight is computed thanks
   to the weight of each moving edge. Those weights are computed by the robust
   estimation method used during the virtual visual servoing.

   \return The mean weight of the first line.
  */
  inline double getMeanWeight1() const { return wmean1; }

  /*!
   Get the mean weight of the second line. The mean weight is computed thanks
   to the weight of each moving edge. Those weights are computed by the robust
   estimation method used during the virtual visual servoing.

   \return The mean weight of the second line.
  */
  inline double getMeanWeight2() const { return wmean2; }

  /*!
    Get the name of the cylinder.

    \return Return the name of the cylinder
  */
  inline std::string getName() const { return name; }

  void initInteractionMatrixError();

  bool initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  /*!
   Return if the cylinder is used for tracking.

   \return True if it is used, False otherwise.
  */
  inline bool isTracked() const { return isTrackedCylinder; }

  /*!
    Check if the cylinder is visible in the image or not.

    \return Return true if the cylinder is visible
  */
  inline bool isVisible() const { return isvisible; }

  void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  /*!
   Set the camera paramters.
   \param camera : The camera parameters.
  */
  inline void setCameraParameters(const vpCameraParameters &camera) { this->cam = camera; }

  /*!
    Set if the cylinder has to considered during tracking phase.

    \param track : True if the cylinder has to be tracked, False otherwise.
  */
  inline void setTracked(const bool &track) { this->isTrackedCylinder = track; }

  /*!
    Set the index of the cylinder.

    \param i : The index number
  */
  inline void setIndex(const unsigned int i) { index = i; }

  /*!
   Set the mean weight of the first line.

   \param wmean : The mean weight of the first line.
  */
  inline void setMeanWeight1(const double wmean) { this->wmean1 = wmean; }

  /*!
   Set the mean weight of the second line.

   \param wmean : The mean weight of the second line.
  */
  inline void setMeanWeight2(const double wmean) { this->wmean2 = wmean; }

  void setMovingEdge(vpMe *Me);

  /*!
    Set the name of the cylinder.

    \param cyl_name : The name of the cylinder.
  */
  inline void setName(const std::string &cyl_name) { this->name = cyl_name; }

  /*!
    Set the name of the cylinder.

    \param cyl_name : The name of the cylinder
  */
  inline void setName(const char *cyl_name) { this->name = std::string(cyl_name); }

  /*!
    Set a boolean parameter to indicates if the cylinder is visible in the
    image or not.

    \param _isvisible : Set to true if the cylinder is visible
  */
  inline void setVisible(bool _isvisible) { isvisible = _isvisible; }

  void trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  void updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

private:
  void project(const vpHomogeneousMatrix &cMo);
};

#endif
