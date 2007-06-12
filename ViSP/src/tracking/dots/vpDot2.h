/****************************************************************************
 *
 * $Id: vpDot2.h,v 1.25 2007-06-12 13:46:55 asaunier Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Track a white dot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


/*
  \file vpDot2.h
  \brief This tracker is meant to track some white zones on a vpImage.
*/

#ifndef vpDot2_hh
#define vpDot2_hh

#include <visp/vpConfig.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>
#include <visp/vpRect.h>
#include <visp/vpTracker.h>


class VISP_EXPORT vpDot2 : public vpTracker
{
public:
  vpDot2();
  vpDot2(const unsigned int u, const unsigned int v) ;
  vpDot2(const double u, const double v) ;
  vpDot2( const vpDot2& twinDot );
  virtual ~vpDot2();
  void operator=( const vpDot2& twinDot );

  void initTracking(vpImage<unsigned char>& I);
  void initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v);
  void initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
		    unsigned int gray_level_min,
		    unsigned int gray_level_max);

  void track(vpImage<unsigned char> &I);
  void track(vpImage<unsigned char> &I, double &u, double &v);

  double get_u() const;
  double get_v() const;
  double getWidth() const;
  double getHeight() const;
  double getSurface() const;
  /*!

    Return the dot bounding box.

    \sa getWidth(), getHeight()

  */
  inline vpRect getBBox() {
    vpRect bbox;

    bbox.setRect(this->bbox_u_min,
		 this->bbox_v_min,
		 this->bbox_u_max - this->bbox_u_min + 1,
		 this->bbox_v_max - this->bbox_v_min + 1);

    return (bbox);
  };
  /*!
    Return the color level of pixels inside the dot.

    \sa getGrayLevelMax()
  */
  inline unsigned int getGrayLevelMin() const {
    return gray_level_min;
  };
  /*!
    Return the color level of pixels inside the dot.

    \sa getGrayLevelMin()
  */
  inline unsigned int getGrayLevelMax() const {
    return gray_level_max;
  };
  double getGrayLevelPrecision() const;
  double getSizePrecision() const;

  /*!
    Activates the display of the border of the dot during the tracking.

    \warning To effectively display the dot graphics a call to
    vpDisplay::flush() is needed.

    \param activate If true, the border of the dot will be painted. false to
    turn off border painting.

  */
  void setGraphics(const bool activate) { graphics = activate ; }
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
  void set_u( const double & u );
  void set_v( const double & v );
  void setWidth( const double & width );
  void setHeight( const double & height );
  void setSurface( const double & surface );
  /*!

  Set the color level of the dot to search a dot in an area. This level will be
  used to know if a pixel in the image belongs to the dot or not. Only pixels
  with higher level can belong to the dot.  If the level is lower than the
  minimum level for a dot, set the level to MIN_IN_LEVEL.

  \param min : Color level of a dot to search in an area.

  \sa setWidth(), setHeight(), setSurface(), setOutLevel(), setGrayLevelPrecision()

  */
  inline void setGrayLevelMin( const unsigned int & min ) {
    if (min > 255)
      this->gray_level_min = 255;
    else
      this->gray_level_min = min;
  };

  /*!

  Set the color level of pixels surrounding the dot. This is meant to be used
  to search a dot in an area.

  \param max : Intensity level of a dot to search in an area.

  \sa setWidth(), setHeight(), setSurface(), setInLevel(), setGrayLevelPrecision()
  */
  inline void setGrayLevelMax( const unsigned int & max ) {
    if (max > 255)
      this->gray_level_max = 255;
    else
      this->gray_level_max = max;
  };
  void setGrayLevelPrecision( const double & grayLevelPrecision );
  void setSizePrecision( const double & sizePrecision );

  double getDistance( const vpDot2& distantDot ) const;


  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I,
				    int area_u, int area_v,
				    int area_w, int area_h );

  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I );

private :

  virtual bool isValid(vpImage<unsigned char>& I, const vpDot2& wantedDot);

  virtual bool hasGoodLevel(const vpImage<unsigned char>& I,
			    const unsigned int &u,
			    const unsigned int &v) const;
  virtual bool hasReverseLevel(vpImage<unsigned char>& I,
			       const unsigned int &u,
			       const unsigned int &v) const;

  virtual vpDot2* getInstance();

public:
  /*!
    Print the coordinates of the point center of gravity
    in the stream.
  */
  friend std::ostream& operator<< (std::ostream& os, vpDot2& p) {
    return (os <<"("<<p.cog_ufloat<<","<<p.cog_vfloat<<")" ) ;
  } ;

  void print(std::ostream& os) { os << *this << std::endl ; }

public :
  double m00; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
		with \f$i = j = 0 \f$. This moment corresponds to the dot
		surface.

		\sa setComputeMoments()
	      */
  double m10; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 0 \f$. \f$ m_{10} \f$
		corresponds to the inertia first order moment along the v axis.

		\sa setComputeMoments()
	      */
  double m01; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
		obtained with \f$i = 0 \f$ and \f$ j = 1 \f$. \f$ m_{01} \f$
		corresponds to the inertia first order moment along the u axis.

		\sa setComputeMoments()
	      */
  double m11; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{11} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

		\sa setComputeMoments()
	      */
  double m20; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{20} \f$ is a second order moment
		obtained with \f$i = 2 \f$ and \f$ j = 0 \f$. \f$ m_{20} \f$
		corresponds to the inertia second order moment along the v
		axis.

		\sa setComputeMoments()
	      */
  double m02; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
		obtained with \f$i = 0 \f$ and \f$ j = 2 \f$. \f$ m_{02} \f$
		corresponds to the inertia second order moment along the u
		axis.

		\sa setComputeMoments()
	      */

protected:
  vpList<int> getListFreemanElement() ;
  vpList<unsigned int> getList_u();
  vpList<unsigned int> getList_v() ;

private:
  bool computeParameters(vpImage<unsigned char> &I,
                              const double &u = -1.0,
                              const double &v = -1.0);
        
  

  bool findFirstBorder(const vpImage<unsigned char> &I, const unsigned int &u,
                        const unsigned int &v, unsigned int &border_u,
                        unsigned int &border_v);
  
  
  /*!

  Get the starting point on a dot border. The dot border is
  computed from this point.

  \sa getFirstBorder_v()

  */
  int getFirstBorder_u() const {
    return this->firstBorder_u;
  }
  /*!

  Get the starting point on a dot border. The dot border is
  computed from this point.

  \sa getFirstBorder_u()

  */
  int getFirstBorder_v() const {
    return this->firstBorder_v;
  }

  bool computeFreemanChainElement(const vpImage<unsigned char> &I,
				  const unsigned int &u,
				  const unsigned int &v,
				  unsigned int &element);
  void computeFreemanParameters(const int &u_p,
				const int &v_p, unsigned int &element,
				int &du, int &dv, float &dS,
				float &dMu, float &dMv,
				float &dMuv,
				float &dMu2, float &dMv2);
  void updateFreemanPosition( unsigned int& u, unsigned int& v,
			      const unsigned int &dir );


  bool isInImage( vpImage<unsigned char> &I ) const;
  bool isInImage( vpImage<unsigned char> &I, const int &u, const int &v) const;

  bool isInArea(const unsigned int &u, const unsigned int &v) const;

  void getGridSize( unsigned int &gridWidth, unsigned int &gridHeight );
  void setArea(vpImage<unsigned char> &I,
	       int u, int v, int w, int h);
  void setArea(vpImage<unsigned char> &I);
  void setArea(const vpRect & a);

  //! coordinates (float) of the point center of gravity
  double cog_ufloat, cog_vfloat ;

  double width;
  double height;
  double surface;
  unsigned int gray_level_min;  // minumum gray level for the dot.
				// pixel with lower level don't belong
				// to this dot.

  unsigned int gray_level_max;  // maximum gray level for the dot.
				// pixel with higher level don't belong
				// to this dot.
  double grayLevelPrecision ;
  double gamma ;
  double sizePrecision ;

  // Area where the dot is to search
  vpRect area;

  // other
  vpList<int> direction_list;
  vpList<unsigned int> u_list;
  vpList<unsigned int> v_list;

  // flag
  bool compute_moment ; // true moment are computed
  bool graphics ; // true for graphic overlay display

  // Bounding box
  int bbox_u_min, bbox_u_max, bbox_v_min, bbox_v_max;

  // The first point coodinate on the dot border
  unsigned int firstBorder_u;
  unsigned int firstBorder_v;

};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

