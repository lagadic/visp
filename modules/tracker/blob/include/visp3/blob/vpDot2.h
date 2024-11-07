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
  \file vpDot2.h
  \brief This tracker is meant to track some zones on a vpImage.
*/

#ifndef VP_DOT2_H
#define VP_DOT2_H

#include <visp3/core/vpColor.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpRect.h>
#include <visp3/core/vpTracker.h>

#include <list>
#include <vector>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpDot2
 *
 * \ingroup module_blob
 *
 * \brief This tracker is meant to track a blob (connex pixels with same
 * gray level) on a vpImage.
 *
 * The underground algorithm is based on a binarization of the image
 * and then on a contour detection using the Freeman chain coding to
 * determine the blob characteristics (location, moments, size...).
 *
 * The binarization is done using gray level minimum and maximum values
 * that define the admissible gray levels of the blob. You can specify these
 * levels by setGrayLevelMin() and setGrayLevelMax(). These levels are also
 * set automatically by setGrayLevelPrecision(). The algorithm allows
 * to track white objects on a black background and vice versa.
 *
 * When a blob is found, some tests are done to see if it is valid:
 * - A blob is considered by default as ellipsoid. The found blob could
 *   be rejected if the shape is not ellipsoid. To determine if the shape
 *   is ellipsoid the algorithm consider an inner and outside ellipse.
 *   Sampled points on these two ellipses should have the right gray levels.
 *   Along the inner ellipse the sampled points should have gray levels
 *   that are in the gray level minimum and maximum bounds, while
 *   on the outside ellipse, the gray levels should be out of the gray level
 *   bounds. To set the percentage of the sample points which should have the
 *   right levels use setEllipsoidBadPointsPercentage(). The distance between the
 *   inner ellipsoid and the blob contour, as well the distance between the
 *   blob contour and the outside ellipse is fixed by
 *   setEllipsoidShapePrecision(). If you want to track a non ellipsoid shape,
 *   and turn off this validation test, you have to call
 *   setEllipsoidShapePrecision(0).
 * - The width, height and surface of the blob are compared to the
 *   corresponding values of the previous blob. If they differ to much
 *   the blob could be rejected. To set the admissible distance you can
 *   use setSizePrecision().
 *
 * Note that track() and searchDotsInArea() are the most important features
 * of this class.
 *
 * - track() estimate the current position of the dot using its previous
 *   position, then try to compute the new parameters of the dot. If everything
 *   went ok, tracking succeeds, otherwise we search this dot in a window
 *   around the last position of the dot.
 *
 * - searchDotsInArea() enable to find dots similar to this dot in a window. It
 *   is used when there was a problem performing basic tracking of the dot, but
 *   can also be used to find a certain type of dots in the full image.
 *
 * The following sample code available in
 * tutorial-blob-tracker-live-firewire.cpp shows how to grab images from a
 * firewire camera, track a blob and display the tracking results.
 *
 * \include tutorial-blob-tracker-live-firewire.cpp
 * A line by line explanation of the previous example is provided in
 * \ref tutorial-tracking-blob.
 *
 * This other example available in tutorial-blob-auto-tracker.cpp shows firstly
 * how to detect in the first image all the blobs that match some
 * characteristics in terms of size, area, gray level. Secondly, it shows how
 * to track all the dots that are detected.
 *
 * \include tutorial-blob-auto-tracker.cpp
 * A line by line explanation of this last example is also provided in
 * \ref tutorial-tracking-blob, section \ref tracking_blob_tracking.
 *
 * \sa vpDot
*/
class VISP_EXPORT vpDot2 : public vpTracker
{
public:
  vpDot2();
  VP_EXPLICIT vpDot2(const vpImagePoint &ip);
  vpDot2(const vpDot2 &twinDot);

  static vpMatrix defineDots(vpDot2 dot[], const unsigned int &n, const std::string &dotFile, vpImage<unsigned char> &I,
                             vpColor col = vpColor::blue, bool trackDot = true);

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

  double getArea() const;

  /*!
   * Return the dot bounding box.
   *
   * \sa getWidth(), getHeight()
   */
  inline vpRect getBBox() const
  {
    vpRect bbox;

    bbox.setRect(m_bbox_u_min, m_bbox_v_min, (m_bbox_u_max - m_bbox_u_min) + 1,
                 (m_bbox_v_max - m_bbox_v_min) + 1);

    return bbox;
  };

  /*!
   * Return the location of the dot center of gravity.
   *
   * \return The coordinates of the center of gravity.
   */
  inline vpImagePoint getCog() const { return m_cog; }

  double getDistance(const vpDot2 &distantDot) const;
  /*!
   * Return the list of all the image points on the dot
   * border.
   *
   * \param edges_list : The list of all the images points on the dot
   * border. This list is update after a call to track().
   */
  void getEdges(std::list<vpImagePoint> &edges_list) const { edges_list = m_ip_edges_list; };

  /*!
   * Return the list of all the image points on the dot
   * border.
   *
   * \return The list of all the images points on the dot
   * border. This list is update after a call to track().
   */
  std::list<vpImagePoint> getEdges() const { return m_ip_edges_list; };

  /*!
   * Get the percentage of sampled points that are considered non conform
   * in terms of the gray level on the inner and the outside ellipses.
   *
   * \sa setEllipsoidBadPointsPercentage()
   */
  double getEllipsoidBadPointsPercentage() const { return m_allowedBadPointsPercentage; }

  double getEllipsoidShapePrecision() const;
  void getFreemanChain(std::list<unsigned int> &freeman_chain) const;

  inline double getGamma() const { return m_gamma; };
  /*!
   * Return the color level of pixels inside the dot.
   *
   * \sa getGrayLevelMax()
   */
  inline unsigned int getGrayLevelMin() const { return m_gray_level_min; };
  /*!
   * Return the color level of pixels inside the dot.
   *
   * \sa getGrayLevelMin()
   */
  inline unsigned int getGrayLevelMax() const { return m_gray_level_max; };
  double getGrayLevelPrecision() const;

  double getHeight() const;
  double getMaxSizeSearchDistPrecision() const;

  /*!
   * \return The mean gray level value of the dot.
   */
  double getMeanGrayLevel() const { return m_mean_gray_level; };

  /*!
   * \return a vpPolygon made from the edges of the dot.
   */
  vpPolygon getPolygon() const { return (vpPolygon(m_ip_edges_list)); };
  double getSizePrecision() const;
  double getWidth() const;

  void initTracking(const vpImage<unsigned char> &I, unsigned int size = 0);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int size = 0);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int gray_lvl_min,
                    unsigned int gray_lvl_max, unsigned int size = 0);

  vpDot2 &operator=(const vpDot2 &twinDot);
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDot2 &d);

  void print(std::ostream &os) { os << *this << std::endl; }
  void searchDotsInArea(const vpImage<unsigned char> &I, int area_u, int area_v, unsigned int area_w,
                        unsigned int area_h, std::list<vpDot2> &niceDots);

  void searchDotsInArea(const vpImage<unsigned char> &I, std::list<vpDot2> &niceDots);

  void setArea(const double &area);
  /*!
   * Initialize the dot coordinates with \e ip.
   */
  inline void setCog(const vpImagePoint &ip) { m_cog = ip; }

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
   * moments by \f$u=\frac{m10}{m00}\f$ and \f$v=\frac{m01}{m00}\f$.
   */
  void setComputeMoments(bool activate) { m_compute_moment = activate; }

  /*!
   * Set the percentage of sampled points that are considered non conform
   * in terms of the gray level on the inner and the outside ellipses.
   * Points located on the inner ellipse should have the same gray level
   * than the blob, while points located on the outside ellipse should
   * have a different gray level.
   *
   * \param percentage : Percentage of points sampled with bad gray level
   * on the inner and outside ellipses that are admissible. 0 means
   * that all the points should have a right level, while a value of 1
   * means that all the points can have a bad gray level.
   */
  void setEllipsoidBadPointsPercentage(const double &percentage = 0.0)
  {
    if (percentage < 0.) {
      m_allowedBadPointsPercentage = 0.;
    }
    else if (percentage > 1.) {
      m_allowedBadPointsPercentage = 1.;
    }
    else {
      m_allowedBadPointsPercentage = percentage;
    }
  }

  void setEllipsoidShapePrecision(const double &ellipsoidShapePrecision);

  /*!
   * Activates the display of the border of the dot during the tracking.
   * The default thickness of the overlayed drawings can be modified using
   * setGraphicsThickness().
   *
   * \warning To effectively display the dot graphics a call to
   * vpDisplay::flush() is needed.
   *
   * \param activate If true, the border of the dot will be painted. false to
   * turn off border painting.
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

  /*!
   * Set the color level of the dot to search a dot in a region of interest. This
   * level will be used to know if a pixel in the image belongs to the dot or
   * not. Only pixels with higher level can belong to the dot.  If the level is
   * lower than the minimum level for a dot, set the level to MIN_IN_LEVEL.
   *
   * \param min : Color level of a dot to search in a region of interest.
   *
   * \sa setGrayLevelMax(), setGrayLevelPrecision()
   */
  inline void setGrayLevelMin(const unsigned int &min)
  {
    const unsigned int val_max = 255;
    if (min > val_max) {
      m_gray_level_min = val_max;
    }
    else {
      m_gray_level_min = min;
    }
  };

  /*!
   * Set the color level of pixels surrounding the dot. This is meant to be used
   * to search a dot in a region of interest.
   *
   * \param max : Intensity level of a dot to search in a region of interest.
   *
   * \sa  setGrayLevelMin(), setGrayLevelPrecision()
   */
  inline void setGrayLevelMax(const unsigned int &max)
  {
    const unsigned int val_max = 255;
    if (max > val_max) {
      m_gray_level_max = val_max;
    }
    else {
      m_gray_level_max = max;
    }
  };

  void setGrayLevelPrecision(const double &grayLevelPrecision);
  void setHeight(const double &height);
  void setMaxSizeSearchDistPrecision(const double &maxSizeSearchDistancePrecision);
  void setSizePrecision(const double &sizePrecision);
  void setWidth(const double &width);

  void track(const vpImage<unsigned char> &I, bool canMakeTheWindowGrow = true);
  void track(const vpImage<unsigned char> &I, vpImagePoint &cog, bool canMakeTheWindowGrow = true);

  static void trackAndDisplay(vpDot2 dot[], const unsigned int &n, vpImage<unsigned char> &I,
                              std::vector<vpImagePoint> &cogs, vpImagePoint *cogStar = nullptr);

  // Static funtions
  static void display(const vpImage<unsigned char> &I, const vpImagePoint &cog,
                      const std::list<vpImagePoint> &edges_list, vpColor color = vpColor::red,
                      unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const vpImagePoint &cog, const std::list<vpImagePoint> &edges_list,
                      vpColor color = vpColor::red, unsigned int thickness = 1);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
private:
#endif
  double m00;  /*!< Considering the general distribution moments for \f$ N \f$
     points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
     u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
     with \f$i = j = 0 \f$. This moment corresponds to the dot
     surface.

     \sa setComputeMoments()
         */
  double m10;  /*!< Considering the general distribution moments for \f$ N \f$
     points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
     u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
     obtained with \f$i = 1 \f$ and \f$ j = 0 \f$. \f$ m_{10} \f$
     corresponds to the inertia first order moment along the v axis.

     \sa setComputeMoments()
         */
  double m01;  /*!< Considering the general distribution moments for \f$ N \f$
     points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
     u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
     obtained with \f$i = 0 \f$ and \f$ j = 1 \f$. \f$ m_{01} \f$
     corresponds to the inertia first order moment along the u axis.

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
     obtained with \f$i = 2 \f$ and \f$ j = 0 \f$. \f$ m_{20} \f$
     corresponds to the inertia second order moment along the v
     axis.

     \sa setComputeMoments()
         */
  double m02;  /*!< Considering the general distribution moments for \f$ N \f$
     points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
     u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
     obtained with \f$i = 0 \f$ and \f$ j = 2 \f$. \f$ m_{02} \f$
     corresponds to the inertia second order moment along the u
     axis.

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
  double mu02; /*!< \f$ \mu_{02} \f$ is a second order centered moments defined
     by: \f$ \mu_{02} = m_{02} - \frac{m_{01}}{m_{00}}m_{01} \f$

     \sa setComputeMoments()
         */

private:
  virtual bool isValid(const vpImage<unsigned char> &I, const vpDot2 &wantedDot);

  virtual bool hasGoodLevel(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v) const;
  virtual bool hasReverseLevel(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v) const;

  virtual vpDot2 *getInstance();

  void init();

  bool computeParameters(const vpImage<unsigned char> &I, const double &u = -1.0, const double &v = -1.0);

  bool findFirstBorder(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                       unsigned int &border_u, unsigned int &border_v);
  void computeMeanGrayLevel(const vpImage<unsigned char> &I);

  /*!
    Get the starting point on a dot border. The dot border is
    computed from this point.
   *
    \sa getFirstBorder_v()
   */
  unsigned int getFirstBorder_u() const { return m_firstBorder_u; }

  /*!
    Get the starting point on a dot border. The dot border is
    computed from this point.
   *
    \sa getFirstBorder_u()
   */
  unsigned int getFirstBorder_v() const { return m_firstBorder_v; }

  bool computeFreemanChainElement(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                                  unsigned int &element);
  void computeFreemanParameters(const int &u_p, const int &v_p, unsigned int &element, int &du, int &dv, float &dS,
                                float &dMu, float &dMv, float &dMuv, float &dMu2, float &dMv2);
  void updateFreemanPosition(unsigned int &u, unsigned int &v, const unsigned int &dir);

  bool isInImage(const vpImage<unsigned char> &I) const;
  bool isInImage(const vpImage<unsigned char> &I, const vpImagePoint &ip) const;

  bool isInArea(const unsigned int &u, const unsigned int &v) const;

  void getGridSize(unsigned int &gridWidth, unsigned int &gridHeight);
  void setArea(const vpImage<unsigned char> &I, int u, int v, unsigned int w, unsigned int h);
  void setArea(const vpImage<unsigned char> &I);
  void setArea(const vpRect &a);

  unsigned char getMeanGrayLevel(vpImage<unsigned char> &I) const;

  typedef struct vpSearchDotsInAreaGoodGermData
  {
    const vpImage<unsigned char> &m_I;
    const vpRect &m_area;
    unsigned int &m_u;
    unsigned int &m_v;
    std::list<vpDot2> &m_niceDots;
    std::list<vpDot2> &m_badDotsVector;

    vpSearchDotsInAreaGoodGermData(const vpImage<unsigned char> &I, const vpRect &area,
                                   unsigned int &u, unsigned int &v,
                                   std::list<vpDot2> &niceDots, std::list<vpDot2> &badDotsVector)
      : m_I(I)
      , m_area(area)
      , m_u(u)
      , m_v(v)
      , m_niceDots(niceDots)
      , m_badDotsVector(badDotsVector)
    {

    }
  } vpSearchDotsInAreaGoodGermData;

  void searchDotsAreaGoodGerm(vpSearchDotsInAreaGoodGermData &data);
  //! Coordinates (float) of the point center of gravity
  vpImagePoint m_cog;

  double m_width;
  double m_height;
  double m_surface;
  unsigned int m_gray_level_min; // minumum gray level for the dot. Pixel with lower level don't belong to this dot.

  unsigned int m_gray_level_max; // maximum gray level for the dot. Pixel with higher level don't belong to this dot.
  double m_mean_gray_level;      // Mean gray level of the dot
  double m_grayLevelPrecision;
  double m_gamma;
  double m_sizePrecision;
  double m_ellipsoidShapePrecision;
  double m_maxSizeSearchDistPrecision;
  double m_allowedBadPointsPercentage;
  // Area where the dot is to search
  vpRect m_area;

  // other
  std::list<unsigned int> m_direction_list;
  std::list<vpImagePoint> m_ip_edges_list;

  // flag
  bool m_compute_moment; // true moment are computed
  bool m_graphics;       // true for graphic overlay display

  unsigned int m_thickness; // Graphics thickness

  // Bounding box
  int m_bbox_u_min, m_bbox_u_max, m_bbox_v_min, m_bbox_v_max;

  // The first point coordinate on the dot border
  unsigned int m_firstBorder_u;
  unsigned int m_firstBorder_v;

};

END_VISP_NAMESPACE
#endif
