/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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

#ifndef vpMbtKltHiddenFace_h
#define vpMbtKltHiddenFace_h

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_OPENCV

#include <visp/vpMbtPolygon.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpPlane.h>
#include <visp/vpDisplay.h>
#include <visp/vpGEMM.h>
#include <visp/vpHomography.h>
#include <visp/vpPlot.h>

/*!
  \class vpMbtKltPolygon
  
  \brief Implementation of a polygon of the model containing points of interest. It is used by the model-based tracker KLT, and hybrid.
  
  \warning This class is only available if OpenCV is installed, and used.
  
  \ingroup ModelBasedTracking
*/
class VISP_EXPORT vpMbtKltPolygon: public vpMbtPolygon
{
private:
  //! the homography in meter
  vpMatrix H; 
  //! normal to the initial plane
  vpColVector N;
  //! current normal
  vpColVector N_cur;
  //! inverse of the distance between the plane and the camera at the initial position (speed up computation)
  double invd0;
  //! cRc0_0n (temporary variable to speed up the computation)
  vpColVector cRc0_0n;
  //! Initial points and their ID
  std::map<int, vpImagePoint> initPoints;
  //! Current points and their ID
  std::map<int, vpImagePoint> curPoints;
  //! number of points detected
  unsigned int nbPointsCur;
  //! initial number of points
  unsigned int nbPointsInit;
  //! Minimal number of points to be tracked
  unsigned int minNbPoint;
  //! Boolean to know if there is enough point to be tracked
  bool enoughPoints;
  //! current camera to plane distance to speed up the computation
  double dt;
  //! distance between the plane and the camera at the initial position
  double d0;
  //! Camera parameters
  vpCameraParameters cam;
  
private:
  
  double              compute_1_over_Z(const double x, const double y);
  void                computeP_mu_t(const double x_in, const double y_in, double& x_out, double& y_out, const vpMatrix& cHc0);
  bool                isTrackedFeature(const int id);
  
public:
                      vpMbtKltPolygon();
  virtual             ~vpMbtKltPolygon();
  
  unsigned int        computeNbDetectedCurrent(const vpKltOpencv& _tracker);
  void                computeHomography(const vpHomogeneousMatrix& _cTc0, vpHomography& cHc0);
  void                computeInteractionMatrixAndResidu(vpColVector& _R, vpMatrix& _J);
  
  void                displayPrimitive(const vpImage<unsigned char>& _I);
  void                displayPrimitive(const vpImage<vpRGBa>& _I);
  
  /*!
    Get the camera parameters of the face.

    \return cam : the camera parameters of the face.
  */
  inline vpCameraParameters& getCameraParameters(){ return cam; }
  
  inline vpColVector  getCurrentNormal() const {return N_cur; }
  
  vpImagePoint&       getImagePoint(const unsigned int _index);
  /*!
    Get the number of point that was belonging to the face at the initialisation

    \return the number of initial point
  */
  inline unsigned int getInitialNumberPoint() const { return nbPointsInit;}
  
  /*!
    get the number of points detected in the last image.

    \warning to have the real number of points, the function computeNbDetectedCurrent
    must be called first.

    \return the number of points detected in the current image
  */
  inline unsigned int getNbPointsCur() const {return nbPointsCur;}
  
  inline  bool        hasEnoughPoints() const {return enoughPoints;}
  
          void        init(const vpKltOpencv& _tracker);
          
          void        removeOutliers(const vpColVector& weight, const double &threshold_outlier);
  
  /*!
    Set the camera parameters

    \param _cam : the new camera parameters
  */
  virtual inline void setCameraParameters(const vpCameraParameters& _cam){ cam = _cam; } 
  
  void                updateMask(IplImage* _mask, unsigned int _nb = 255, unsigned int _shiftBorder = 0);
  
//###################
// Static Functions
//###################
private:
  static bool         isInside(const std::vector<vpImagePoint>& roi, const double i, const double  j);
  static bool         intersect(const vpImagePoint& p1, const vpImagePoint& p2, const double  i, const double  j, const double  i_test, const double  j_test);
};

#endif

#endif // VISP_HAVE_OPENCV