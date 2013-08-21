/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Track a white dot.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpDot.h
  \brief Track a white dot
*/

#ifndef vpDot_hh
#define vpDot_hh

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpTracker.h>
#include <visp/vpRect.h>
#include <visp/vpImagePoint.h>

#include <math.h>
#include <fstream>
#include <list>
#include <vector>

/*!
  \class vpDot

  \ingroup TrackingImageBasic

  \brief This tracker is meant to track a dot (connected pixels with same
  gray level) on a vpImage.

  The underground algorithm is based on a binarization of the image
  and a connex component segmentation to determine the dot
  characteristics (location, moments, size...).

  The following sample code shows how to grab images from a firewire camera,
  track a blob and display the tracking results.

  \code
#include <visp/vpConfig.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot.h>
#include <visp/vpImage.h>

int main()
{
#if defined(VISP_HAVE_DC1394_2)
  vpImage<unsigned char> I; // Create a gray level image container
  vp1394TwoGrabber g(false); // Create a grabber based on libdc1394-2.x third party lib
  g.acquire(I); // Acquire an image

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, 0, 0, "Camera view");
#endif
  vpDisplay::display(I);
  vpDisplay::flush(I);

  vpDot blob;
  blob.initTracking(I);
  blob.setGraphics(true);

  while(1) {
    g.acquire(I); // Acquire an image
    vpDisplay::display(I);
    blob.track(I);

    vpDisplay::flush(I);
  }
#endif
}
  \endcode

  \sa vpDot2
*/
class VISP_EXPORT vpDot : public vpTracker
{
public :
  /*! \enum vpConnexityType
  Type of connexity 4, or 8.
  */
  typedef enum {
    CONNEXITY_4, /*!< For a given pixel 4 neighbors are considered (left,
		   right, up, down) */
    CONNEXITY_8 /*!< For a given pixel 8 neighbors are considered (left,
		  right, up, down, and the 4 pixels located on the diagonal) */
  } vpConnexityType;

  static const unsigned int SPIRAL_SEARCH_SIZE; /*!< Spiral size for the dot
						  search. */
  double m00; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
		with \f$i = j = 0 \f$.

		\sa setComputeMoments()
	      */
  double m01; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
		obtained with \f$i = 0 \f$ and \f$ j = 1 \f$.

		\sa setComputeMoments()
	      */
  double m10; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 0 \f$.

		\sa setComputeMoments()
	      */
  double m11; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{11} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

		\warning This moment is not computed for the moment.

		\sa setComputeMoments()
	      */
  double m20; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{20} \f$ is a second order moment
		obtained with \f$i = 2 \f$ and \f$ j = 0 \f$.

		\warning This moment is not computed for the moment.

		\sa setComputeMoments()
	      */
  double m02; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
		obtained with \f$i = 0 \f$ and \f$ j = 2 \f$.

		\warning This moment is not computed for the moment.

		\sa setComputeMoments()
	      */
  double mu11;/*!< \f$ \mu_{11} \f$ is a second order central moments defined by:
		\f$ \mu_{11} = m_{11} - \frac{m_{10}}{m_{00}}m_{01} \f$
		
		\sa setComputeMoments()
	      */
  double mu20;/*!< \f$ \mu_{20} \f$ is a second order central moments defined by:
		\f$ \mu_{20} = m_{20} - \frac{m_{10}}{m_{00}}m_{10} \f$
		
		\sa setComputeMoments()
	      */
  double mu02;/*!< \f$ \mu_{02} \f$ is a second order central moments defined by:
		\f$ \mu_{02} = m_{02} - \frac{m_{01}}{m_{00}}m_{01} \f$
		
		\sa setComputeMoments()
	      */

  vpDot() ;
  vpDot(const vpImagePoint &ip) ;
  vpDot(const vpDot& d) ;
  virtual ~vpDot() ;

  void display(const vpImage<unsigned char>& I, vpColor color = vpColor::red,
               unsigned int thickness=1) const;

  /*!

    Return the dot bounding box.

    \sa getWidth(), getHeight()

  */
  inline vpRect getBBox() const {
    vpRect bbox;

    bbox.setRect(this->u_min,
     this->v_min,
     this->u_max - this->u_min + 1,
     this->v_max - this->v_min + 1);

    return (bbox);
  };
  /*!
    Return the location of the dot center of gravity.

    \return The coordinates of the center of gravity.
  */
  inline vpImagePoint getCog() const {
    return cog;
  }

  /*!
      Return the list of all the image points on the border of the dot.

      \warning Doesn't return the image points inside the dot anymore. To get those points see getConnexities().
  */
  inline std::list<vpImagePoint> getEdges() const {
    return this->ip_edges_list;
  };

  /*!

    Return the list of all the image points inside the dot.

    \return The list of all the images points in the dot.
    This list is updated after a call to track().

  */
  inline std::list<vpImagePoint> getConnexities() const {
    return this->ip_connexities_list;
  };

  inline double getGamma() const {return this->gamma;};
  /*!

    Return the precision of the gray level of the dot. It is a double
    precision float witch value is in ]0,1]. 1 means full precision, whereas
    values close to 0 show a very bad precision.

  */
  double getGrayLevelPrecision() const {return grayLevelPrecision;}
  double getMaxDotSize() const {
    return this->maxDotSizePercentage;
  }
  /*!
    Return the mean gray level value of the dot.
  */
  double getMeanGrayLevel() const {
    return (this->mean_gray_level);
  };

  /*!

    Return the width of the dot.

    \sa getHeight()

  */
  inline unsigned int getWidth() const {
    return (this->u_max - this->u_min + 1);
  };

  /*!

    Return the width of the dot.

    \sa getHeight()

  */
  inline unsigned int getHeight() const {
    return (this->v_max - this->v_min + 1);
  };

  void initTracking(const vpImage<unsigned char> &I) ;
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip);
  void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip,
		    unsigned int gray_level_min, unsigned int gray_level_max);

  vpDot& operator =(const vpDot& d) ;
  bool operator ==(const vpDot& d);
  bool operator !=(const vpDot& d);
  /*!
    Writes the dot center of gravity coordinates in the frame (i,j) (For more details
    about the orientation of the frame see the vpImagePoint documentation) to the stream \e os,
    and returns a reference to the stream.
  */
  friend VISP_EXPORT std::ostream& operator<< (std::ostream& os, vpDot& d) {
    return (os << "(" << d.getCog() << ")" ) ;
  } ;

  void print(std::ostream& os) { os << *this << std::endl ; }

  /*!
    Initialize the dot coordinates with \e cog.
  */
  inline void setCog(const vpImagePoint &cog) {
    this->cog = cog;
  }

  /*!

    Activates the dot's moments computation.

    \param activate true, if you want to compute the moments. If false, moments
    are not computed.

    Computed moment are vpDot::m00, vpDot::m10, vpDot::m01, vpDot::m11,
    vpDot::m20, vpDot::m02.

    The coordinates of the region's centroid (u, v) can be computed from the
    moments by \f$u=\frac{m10}{m00}\f$ and  \f$v=\frac{m01}{m00}\f$.

  */
  void setComputeMoments(const bool activate) { compute_moment = activate; }
  
  /*!
    Set the type of connexity: 4 or 8.
  */
  void setConnexity(vpConnexityType connexityType) {this->connexityType = connexityType; };
  void setMaxDotSize(double percentage) ;
  void setGrayLevelMin( const unsigned int &gray_level_min ) {
    this->gray_level_min = gray_level_min;
  };
  void setGrayLevelMax( const unsigned int &gray_level_max ) {
    this->gray_level_max = gray_level_max;
  };
  void setGrayLevelPrecision( const double & grayLevelPrecision );

  /*!
    Activates the display of all the pixels of the dot during the tracking.
    The default thickness of the overlayed drawings can be modified using
    setGraphicsThickness().

    \warning To effectively display the dot graphics a call to
    vpDisplay::flush() is needed.

    \param activate true to activate the display of dot pixels, false to turn
    off the display.

    \sa setGraphicsThickness()
  */
  void setGraphics(const bool activate) { graphics = activate ; }
  /*!
    Modify the default thickness that is set to 1 of the drawings in overlay when setGraphics() is enabled.

    \sa setGraphics()
    */
  void setGraphicsThickness(unsigned int thickness) {this->thickness = thickness;};

  void track(const vpImage<unsigned char> & I) ;
  void track(const vpImage<unsigned char> & I, vpImagePoint &ip) ;

private:
  //! internal use only
  std::list<vpImagePoint> ip_connexities_list;
  
  //! List of border points
  std::list<vpImagePoint> ip_edges_list;

  /*! Type of connexity
   
   \warning In previous version this variable was called connexity
  */
  vpConnexityType connexityType;

  //! coordinates of the point center of gravity
  vpImagePoint cog;

  // Bounding box
  unsigned int u_min, u_max, v_min, v_max;

  // Flag used to allow display
  bool graphics ;

  unsigned int thickness; // Graphics thickness

  double maxDotSizePercentage;
  unsigned char gray_level_out;
  
  double mean_gray_level; // Mean gray level of the dot
  unsigned int gray_level_min; // left threshold for binarisation
  unsigned int gray_level_max; // right threshold for binarisation
  double grayLevelPrecision;  //precision of the gray level of the dot.
  //It is a double precision float witch value is in ]0,1].
  //1 means full precision, whereas values close to 0 show a very bad precision
  double gamma ;
  //! flag : true moment are computed
  bool compute_moment ;
  double nbMaxPoint;
  
  void init() ;
  void setGrayLevelOut();
  bool connexe(const vpImage<unsigned char>& I,unsigned int u,unsigned int v,
	      double &mean_value, double &u_cog, double &v_cog, double &n);
  bool connexe(const vpImage<unsigned char>& I,unsigned int u,unsigned int v,
	      double &mean_value, double &u_cog, double &v_cog, double &n,std::vector<bool> &checkTab);
  void COG(const vpImage<unsigned char> &I,double& u, double& v) ;
  
//Static Functions
public:
  static void display(const vpImage<unsigned char>& I,const vpImagePoint &cog,
                      const std::list<vpImagePoint> &edges_list, vpColor color = vpColor::red,
                      unsigned int thickness=1);
  static void display(const vpImage<vpRGBa>& I,const vpImagePoint &cog,
                      const std::list<vpImagePoint> &edges_list, vpColor color = vpColor::red,
                      unsigned int thickness=1);
} ;

#endif



