/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Track a white dot.
 */

/*!
  \file vpDot.h
  \brief Track a white dot
*/

#ifndef VP_DOT_H
#define VP_DOT_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpRect.h>
#include <visp3/core/vpTracker.h>

#include <fstream>
#include <list>
#include <math.h>
#include <vector>

#ifdef VISP_USE_MSVC
#pragma comment(linker, "/STACK:256000000") // Increase max recursion depth
#endif

BEGIN_VISP_NAMESPACE

/*!
 * \class vpDot
 *
 * \ingroup module_blob
 *
 * \brief This tracker is meant to track a dot (connected pixels with same
 * gray level) on a vpImage.
 *
 * The underground algorithm is based on a binarization of the image
 * and a connex component segmentation to determine the dot
 * characteristics (location, moments, size...).
 *
 * The following sample code shows how to grab images from a firewire camera,
 * track a blob and display the tracking results.
 *
 * \code
 * #include <visp3/blob/vpDot.h>
 * #include <visp3/gui/vpDisplayX.h>
 * #include <visp3/sensor/vp1394TwoGrabber.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_DC1394)
 *   vpImage<unsigned char> I; // Create a gray level image container
 *   vp1394TwoGrabber g(false); // Create a grabber based on libdc1394-2.x third party lib
 *   g.acquire(I); // Acquire an image
 *
 * #if defined(VISP_HAVE_X11)
 *   vpDisplayX d(I, 0, 0, "Camera view");
 * #endif
 *   vpDisplay::display(I);
 *   vpDisplay::flush(I);
 *
 *   vpDot blob;
 *   blob.initTracking(I);
 *   blob.setGraphics(true);
 *
 *   while(1) {
 *     g.acquire(I); // Acquire an image
 *     vpDisplay::display(I);
 *     blob.track(I);
 *
 *     vpDisplay::flush(I);
 *   }
 * #endif
 * }
 * \endcode
 *
 * \sa vpDot2
*/
class VISP_EXPORT vpDot : public vpTracker
{
public:
  /*! \enum vpConnexityType
   * Type of connexity 4, or 8.
   */
  typedef enum
  {
    CONNEXITY_4, /*!< For a given pixel 4 neighbors are considered (left,
       right, up, down) */
    CONNEXITY_8  /*!< For a given pixel 8 neighbors are considered (left,
       right, up, down, and the 4 pixels located on the diagonal) */
  } vpConnexityType;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
  static const unsigned int SPIRAL_SEARCH_SIZE; /*!< Spiral size for the dot search. */

  double m00;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
                    with \f$i = j = 0 \f$.

                    \sa setComputeMoments()
                */
  double m01;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
                    obtained with \f$i = 0 \f$ and \f$ j = 1 \f$.

                    \sa setComputeMoments()
                */
  double m10;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
                    obtained with \f$i = 1 \f$ and \f$ j = 0 \f$.

                    \sa setComputeMoments()
                */
  double m11;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{11} \f$ is a first order moment
                    obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

                    \sa setComputeMoments()
                */
  double m20;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{20} \f$ is a second order moment
                    obtained with \f$i = 2 \f$ and \f$ j = 0 \f$.

                    \sa setComputeMoments()
                */
  double m02;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
                    obtained with \f$i = 0 \f$ and \f$ j = 2 \f$.

                     \sa setComputeMoments()
                */
  double mu11; /*!< \f$ \mu_{11} \f$ is a second order centered moment defined
                    by: \f$ \mu_{11} = m_{11} - \frac{m_{10}}{m_{00}}m_{01} \f$

                    \sa setComputeMoments()
                */
  double mu20; /*!< \f$ \mu_{20} \f$ is a second order centered moment defined
                    by: \f$ \mu_{20} = m_{20} - \frac{m_{10}}{m_{00}}m_{10} \f$

                    \sa setComputeMoments()
                */
  double mu02; /*!< \f$ \mu_{02} \f$ is a second order centered moment defined
                     by: \f$ \mu_{02} = m_{02} - \frac{m_{01}}{m_{00}}m_{01} \f$

                     \sa setComputeMoments()
                */
#endif

public:
  vpDot();
  VP_EXPLICIT vpDot(const vpImagePoint &ip);
  vpDot(const vpDot &d);
  virtual ~vpDot() VP_OVERRIDE;

  void display(const vpImage<unsigned char> &I, vpColor color = vpColor::red, unsigned int thickness = 1) const;

  /*!
   * Gets the second order normalized centered moment \f$ n_{ij} \f$
   * as a 3-dim vector containing \f$ n_{20}, n_{11}, n_{02} \f$
   * such as \f$ n_{ij} = \mu_{ij}/m_{00} \f$
   *
   * \return The 3-dim vector containing \f$ n_{20}, n_{11}, n_{02} \f$.
   *
   * \sa getCog(), getArea()
   */
  inline vpColVector get_nij() const
  {
    vpColVector nij(3);
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    nij[index_0] = mu20 / m00;
    nij[index_1] = mu11 / m00;
    nij[index_2] = mu02 / m00;

    return nij;
  }

  /*!
   * Gets the area of the blob corresponding also to the zero order moment.
   *
   * \return The blob area.
   */
  inline double getArea() const { return m00; }

  /*!
   * Return the dot bounding box.
   *
   * \sa getWidth(), getHeight()
   */
  inline vpRect getBBox() const
  {
    vpRect bbox;

    bbox.setRect(m_u_min, m_v_min, (m_u_max - m_u_min) + 1, (m_v_max - m_v_min) + 1);

    return bbox;
  };
  /*!
   * Return the location of the dot center of gravity.
   *
   * \return The coordinates of the center of gravity.
   */
  inline vpImagePoint getCog() const { return m_cog; }

  /*!
   * Return the list of all the image points on the border of the dot.
   *
   * \warning Doesn't return the image points inside the dot anymore. To get
   * those points see getConnexities().
   */
  inline std::list<vpImagePoint> getEdges() const { return m_ip_edges_list; };

  /*!
   * Return the list of all the image points inside the dot.
   *
   * \return The list of all the images points in the dot.
   * This list is updated after a call to track().
   */
  inline std::list<vpImagePoint> getConnexities() const { return m_ip_connexities_list; };

  inline double getGamma() const { return m_gamma; };

  /*!
   * Return the precision of the gray level of the dot. It is a double
   * precision float witch value is in ]0,1]. 1 means full precision, whereas
   * values close to 0 show a very bad precision.
   */
  double getGrayLevelPrecision() const { return m_grayLevelPrecision; }
  double getMaxDotSize() const { return m_maxDotSizePercentage; }

  /*!
   * Return the mean gray level value of the dot.
   */
  double getMeanGrayLevel() const { return m_mean_gray_level; };

  /*!
   * \return a vpPolygon made from the edges of the dot.
   */
  vpPolygon getPolygon() const { return (vpPolygon(m_ip_edges_list)); };

  /*!
   * Return the width of the dot.
   *
   * \sa getHeight()
   */
  inline unsigned int getWidth() const { return ((m_u_max - m_u_min) + 1); };

  /*!
   * Return the width of the dot.
   *
   * \sa getHeight()
   */
  inline unsigned int getHeight() const { return ((m_v_max - m_v_min) + 1); };

  void initTracking(const vpImage<unsigned char> &I);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int gray_level_min,
                    unsigned int gray_level_max);

  vpDot &operator=(const vpDot &d);
  bool operator==(const vpDot &d) const;
  bool operator!=(const vpDot &d) const;
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDot &d);

  void print(std::ostream &os) { os << *this << std::endl; }

  /*!
   * Initialize the dot center of gravity coordinates with \e cog.
   */
  inline void setCog(const vpImagePoint &cog) { m_cog = cog; }

  /*!
   * Activates the dot's moments computation.
   *
   * \param activate true, if you want to compute the moments. If false,
   * moments are not computed.
   *
   * Computed moment are vpDot::m00, vpDot::m10, vpDot::m01, vpDot::m11,
   * vpDot::m20, vpDot::m02 and second order centered moments vpDot::mu11,
   * vpDot::mu20, vpDot::mu02 computed with respect to the blob centroid.
   *
   * The coordinates of the region's centroid (u, v) can be computed from the
   * moments by \f$u=\frac{m10}{m00}\f$ and  \f$v=\frac{m01}{m00}\f$.
   */
  void setComputeMoments(bool activate) { m_compute_moment = activate; }

  /*!
   * Set the type of connexity: 4 or 8.
   */
  void setConnexity(const vpConnexityType &connexityType) { m_connexityType = connexityType; };
  void setMaxDotSize(double percentage);
  void setGrayLevelMin(const unsigned int &level_min) { m_gray_level_min = level_min; };
  void setGrayLevelMax(const unsigned int &level_max) { m_gray_level_max = level_max; };
  void setGrayLevelPrecision(const double &grayLevelPrecision);

  /*!
   * Activates the display of all the pixels of the dot during the tracking.
   * The default thickness of the overlayed drawings can be modified using
   * setGraphicsThickness().
   *
   * \warning To effectively display the dot graphics a call to
   * vpDisplay::flush() is needed.
   *
   * \param activate true to activate the display of dot pixels, false to turn
   * off the display.
   *
   * \sa setGraphicsThickness()
   */

  void setGraphics(bool activate) { m_graphics = activate; }
  /*!
   * Modify the default thickness that is set to 1 of the drawings in overlay
   * when setGraphics() is enabled.
   *
   * \sa setGraphics()
   */
  void setGraphicsThickness(unsigned int thickness) { m_thickness = thickness; };

  void track(const vpImage<unsigned char> &I);
  void track(const vpImage<unsigned char> &I, vpImagePoint &ip);

  // Static Functions
public:
  static void display(const vpImage<unsigned char> &I, const vpImagePoint &cog,
                      const std::list<vpImagePoint> &edges_list, vpColor color = vpColor::red,
                      unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const vpImagePoint &cog, const std::list<vpImagePoint> &edges_list,
                      vpColor color = vpColor::red, unsigned int thickness = 1);

#ifndef VISP_BUILD_DEPRECATED_FUNCTIONS
private:
  static const unsigned int SPIRAL_SEARCH_SIZE; /*!< Spiral size for the dot search. */

  double m00;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
                    with \f$i = j = 0 \f$.

                    \sa setComputeMoments()
                */
  double m01;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
                    obtained with \f$i = 0 \f$ and \f$ j = 1 \f$.

                    \sa setComputeMoments()
                */
  double m10;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
                    obtained with \f$i = 1 \f$ and \f$ j = 0 \f$.

                    \sa setComputeMoments()
                */
  double m11;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{11} \f$ is a first order moment
                    obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

                    \sa setComputeMoments()
                */
  double m20;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{20} \f$ is a second order moment
                    obtained with \f$i = 2 \f$ and \f$ j = 0 \f$.

                    \sa setComputeMoments()
                */
  double m02;  /*!< Considering the general distribution moments for \f$ N \f$
                    points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
                    u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
                    obtained with \f$i = 0 \f$ and \f$ j = 2 \f$.

                     \sa setComputeMoments()
                */
  double mu11; /*!< \f$ \mu_{11} \f$ is a second order centered moment defined
                    by: \f$ \mu_{11} = m_{11} - \frac{m_{10}}{m_{00}}m_{01} \f$

                    \sa setComputeMoments()
                */
  double mu20; /*!< \f$ \mu_{20} \f$ is a second order centered moment defined
                    by: \f$ \mu_{20} = m_{20} - \frac{m_{10}}{m_{00}}m_{10} \f$

                    \sa setComputeMoments()
                */
  double mu02; /*!< \f$ \mu_{02} \f$ is a second order centered moment defined
                     by: \f$ \mu_{02} = m_{02} - \frac{m_{01}}{m_{00}}m_{01} \f$

                     \sa setComputeMoments()
                */
#endif

private:
  //! internal use only
  std::list<vpImagePoint> m_ip_connexities_list;

  //! List of border points
  std::list<vpImagePoint> m_ip_edges_list;

  /*! Type of connexity

   \warning In previous version this variable was called connexity
  */
  vpConnexityType m_connexityType;

  //! Coordinates of the point center of gravity
  vpImagePoint m_cog;

  // Bounding box
  unsigned int m_u_min, m_u_max, m_v_min, m_v_max;

  // Flag used to allow display
  bool m_graphics;

  unsigned int m_thickness; // Graphics thickness

  double m_maxDotSizePercentage;
  unsigned char m_gray_level_out;

  double m_mean_gray_level;      // Mean gray level of the dot
  unsigned int m_gray_level_min; // left threshold for binarization
  unsigned int m_gray_level_max; // right threshold for binarization
  double m_grayLevelPrecision;   // precision of the gray level of the dot.
  // It is a double precision float witch value is in ]0,1].
  // 1 means full precision, whereas values close to 0 show a very bad
  // precision
  double m_gamma;
  //! flag : true moment are computed
  bool m_compute_moment;
  double m_nbMaxPoint;

  void init();
  void setGrayLevelOut();
  bool connexe(const vpImage<unsigned char> &I, unsigned int u, unsigned int v, double &mean_value,
               vpImagePoint &uv_cog, unsigned int &npoints);
  bool connexe(const vpImage<unsigned char> &I, unsigned int u, unsigned int v, double &mean_value,
               vpImagePoint &uv_cog, unsigned int &npoints, std::vector<bool> &checkTab);
  void COG(const vpImage<unsigned char> &I, double &u, double &v);
};

END_VISP_NAMESPACE
#endif
