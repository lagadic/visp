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
 * Manage the line of a polygon used in the model-based tracker.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
 \file vpMbtDistanceLine.h
 \brief Manage the line of a polygon used in the model-based tracker.
*/

#ifndef vpMbtDistanceLine_HH
#define vpMbtDistanceLine_HH

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpPoint.h>
#include <visp3/mbt/vpMbHiddenFaces.h>
#include <visp3/mbt/vpMbtMeLine.h>
#include <visp3/visual_features/vpFeatureLine.h>

#include <list>

/*!
  \class vpMbtDistanceLine

  \brief Manage the line of a polygon used in the model-based tracker.

  \ingroup group_mbt_features

 */
class VISP_EXPORT vpMbtDistanceLine
{
private:
  std::string name;
  unsigned int index;
  vpCameraParameters cam;
  vpMe *me;
  bool isTrackedLine;
  bool isTrackedLineWithVisibility;
  double wmean;
  vpFeatureLine featureline;
  //! Polygon describing the line
  vpMbtPolygon poly;

public:
  //! Use scanline rendering
  bool useScanLine;
  //! The moving edge container
  // vpMbtMeLine *meline;
  std::vector<vpMbtMeLine *> meline;
  //! The 3D line
  vpLine *line;
  //! The first extremity
  vpPoint *p1;
  //! The second extremity
  vpPoint *p2;
  //! The interaction matrix
  vpMatrix L;
  //! The error vector
  vpColVector error;
  //! The number of moving edges
  // unsigned int nbFeature;
  std::vector<unsigned int> nbFeature;
  //! The number of moving edges
  unsigned int nbFeatureTotal;
  //! Indicates if the line has to be reinitialized
  bool Reinit;
  //! Pointer to the list of faces
  vpMbHiddenFaces<vpMbtPolygon> *hiddenface;
  //! Index of the faces which contain the line
  std::list<int> Lindex_polygon;
  //! Vector of bool associated with Lindex_polygon to know if
  //! Lindex_polygon[i] is tracked
  std::vector<bool> Lindex_polygon_tracked;
  //! Indicates if the line is visible or not
  bool isvisible;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //    vpMbtDistanceLine(const vpMbtDistanceLine &)
  //      : name(), index(0), cam(), me(NULL), isTrackedLine(true),
  //      isTrackedLineWithVisibility(true),
  //        wmean(1), featureline(), poly(), useScanLine(false), meline(),
  //        line(NULL), p1(NULL), p2(NULL), L(), error(), nbFeature(),
  //        nbFeatureTotal(0), Reinit(false), hiddenface(NULL),
  //        Lindex_polygon(), Lindex_polygon_tracked(), isvisible(false)
  //    {
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!");
  //    }
  //    vpMbtDistanceLine &operator=(const vpMbtDistanceLine &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  vpMbtDistanceLine();
  ~vpMbtDistanceLine();

  void addPolygon(const int &index);

  void buildFrom(vpPoint &_p1, vpPoint &_p2);

  bool closeToImageBorder(const vpImage<unsigned char> &I, const unsigned int threshold);
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
  inline void getCameraParameters(vpCameraParameters &camera) const { camera = this->cam; }

  /*!
    Get the index of the line.

    \return Return the index of the line.
  */
  inline unsigned int getIndex() const { return index; }

  /*!
   Get the mean weight of the line. The mean weight is computed thanks to the
   weight of each moving edge. Those weights are computed by the robust
   estimation method used during the virtual visual servoing.

   \return The mean weight of the line.
  */
  inline double getMeanWeight() const { return wmean; }

  /*!
    Get the name of the line.

    \return Return the name of the line
  */
  inline std::string getName() const { return name; }

  /*!
   Get the polygon associated to the line.

   \return poly.
  */
  inline vpMbtPolygon &getPolygon() { return poly; }

  void initInteractionMatrixError();

  bool initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  /*!
   Return if the line is used for tracking.

   \return True if it is used, False otherwise.
  */
  inline bool isTracked() const { return isTrackedLineWithVisibility; }

  /*!
    Check if the line is visible in the image or not.

    \return Return true if the line is visible
  */
  inline bool isVisible() const { return isvisible; }

  void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  /*!
   Set the camera paramters.
   \param camera : The camera parameters.
  */
  inline void setCameraParameters(const vpCameraParameters &camera) { this->cam = camera; }

  /*!
    Set the index of the line.

    \param i : The index number
  */
  inline void setIndex(const unsigned int i) { index = i; }

  /*!
   Set the mean weight of the line.

   \param w_mean : The mean weight of the line.
  */
  inline void setMeanWeight(const double w_mean) { this->wmean = w_mean; }

  void setMovingEdge(vpMe *Me);

  /*!
    Set the name of the line.

    \param line_name : The name of the line.
  */
  inline void setName(const std::string &line_name) { this->name = line_name; }

  /*!
    Set the name of the line.

    \param line_name : The name of the line.
  */
  inline void setName(const char *line_name) { this->name = std::string(line_name); }

  void setTracked(const std::string &name, const bool &track);

  /*!
    Set a boolean parameter to indicates if the line is visible in the image
    or not.

    \param _isvisible : Set to true if the line is visible
  */
  void setVisible(bool _isvisible) { isvisible = _isvisible; }

  void trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  void updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  void updateTracked();

private:
  void project(const vpHomogeneousMatrix &cMo);
};

#endif
