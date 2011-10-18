/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Object input structure used by moments.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentObject.h
  \brief Object input structure used by moments.
*/
#ifndef __MOMENTOBJECT_H__
#define __MOMENTOBJECT_H__
#include <utility>
#include <vector>
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMoment.h>
#include <visp/vpPoint.h>

class vpCameraParameters;

/*!
  \class vpMomentObject

  \ingroup TrackingMoments

  \brief Class for generic objects. 

  It contains all basic moments often described by \f$m_{ij}\f$ of order \f$i+j\f$ going from \f$m_{00}\f$ to the order used as parameter in vpMomentObject() constructor.
  All other moments implemented in ViSP (gravity center, alpha orientation, centered moments...) use this moment object as a combination of its different values.

  When constructing a vpMomentObject() you need first to specify the maximum used moment order as parameter.

  Then there are three ways to initialize a vpMomentObject. Firstly using fromImage() you can considerer
  a dense object \e O defined by a binary image. Secondly, as described in fromVector() you can also define a dense object \e O by a closed contour.
  In these two cases, 2D basic moments are defined by:
  \f[m_{ij} = \int \int_{O} x^j y^j dx dy\f]

  Lastly, as presented in fromVector() you can consider a discrete set of \e n points. In that last case, the basic moments are defined by
  \f[m_{ij} = \sum_{k=1}^{n} x_k^j y_k^j \f]

  With setType() method you can specify the object type.


  \attention Be careful with the object order. When you specify a maximum order in the vpMomentObject::vpMomentObject constructor (see its detailed description),
    it will compute all moment orders up to the order you specified. If you want to access the values \f$ m_{ij} \f$ with the vpMomentObject::get method, you can
    do object.get()[j*(order+1)+i].

    A few tips about which orders to use in different situations:
    - moment based visual servoing: use vpMomentObject(6). This will compute moment values up to order 6 which will enable vpFeatureMoments up to order 5 which is the maximum order required for common moments.
    - computing gravity center: use vpMomentObject(1). You only need \f$ m_{00},m_{01},m_{10} \f$. You should compute moments up to order 1.
    - computing gravity center interaction matrix (vpFeatureMomentGravityCenter): use vpMomentObject(2). This will compute moment values till order 2 since they are needed for the interaction matrix of vpFeatureMoments of order 1.


  The following example shows how to create a moment object from 4 discrete points
  locate on a plane one meter in front of the camera. It shows also how to get the basic
  moments that are computed and how to compute other classical moments such as the gravity
  center or the centered moments.
  \code
#include <visp/vpMomentObject.h>
#include <visp/vpMomentCommon.h>
#include <visp/vpPoint.h>

int main()
{
  // Define an object as 4 clockwise points on a plane (Z=0)
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the 4 points

  p.setWorldCoordinates(-0.2, 0.1, 0.0); // values in meters
  vec_p.push_back(p);
  p.setWorldCoordinates(+0.3, 0.1, 0.0); // values in meters
  vec_p.push_back(p);
  p.setWorldCoordinates(+0.2,-0.1, 0.0); // values in meters
  vec_p.push_back(p);
  p.setWorldCoordinates(-0.2,-0.15, 0.0); // values in meters
  vec_p.push_back(p);

  // These points are observed by a camera
  vpHomogeneousMatrix cMo(0, 0, 1, 0, 0, 0); // We set the camera to be 1m far the object
  // ... update cMo from an image processing

  // Apply the perspective projection to update the points coordinates in the camera plane
  for(unsigned int i=0; i<vec_p.size(); ++i)
    vec_p[i].project(cMo);

  std::cout << "Considered points: " << std::endl;
  for(unsigned int i=0; i<vec_p.size(); ++i)
    std::cout << "point " << i << ": " << vec_p[i].get_x() << ", " << vec_p[i].get_y() << std::endl;

  // Define an image moment object from the previous points
  vpMomentObject obj(5); // use moments up to order 5
  obj.setType(vpMomentObject::DISCRETE); // initialize the object as constituted by discrete points
  obj.fromVector(vec_p); // init the object from the points

  std::vector<double> moment = obj.get();
  std::cout << std::endl << "Basic moment available: " << std::endl;
  for(unsigned int j=0; j<obj.getOrder(); ++j) {
    for(unsigned int i=0; i<obj.getOrder(); ++i)
      std::cout << "m" << i << j << "=" << moment[j*obj.getOrder() + i] << "  ";
    std::cout << std::endl;
  }

  std::cout << std::endl << "Basic moment avalaible: ";
  std::cout << obj << std::endl;

  std::cout << std::endl << "Direct acces to some basic moments: " << std::endl;
  std::cout << "m00: " << obj.get(0, 0) << std::endl;
  std::cout << "m10: " << obj.get(1, 0) << std::endl;
  std::cout << "m01: " << obj.get(0, 1) << std::endl;
  std::cout << "m22: " << obj.get(2, 2) << std::endl;
  std::cout << "m20: " << obj.get(2, 0) << std::endl;
  std::cout << "m02: " << obj.get(0, 2) << std::endl;

  // Get common moments computed using basic moments
  double m00 = vpMomentCommon::getSurface(obj); // surface = m00
  double alpha = vpMomentCommon::getAlpha(obj); // orientation
  std::vector<double> mu_3 = vpMomentCommon::getMu3(obj); // centered moment up to 3rd order

  std::cout << std::endl << "Common moments computed using basic moments:" << std::endl;
  std::cout << "Surface: " << m00 << std::endl;
  std::cout << "Alpha: " << alpha << std::endl;
  std::cout << "Centered moments (mu03, mu12, mu21, mu30): ";
  for(unsigned int i=0; i<mu_3.size(); ++i)
    std::cout << mu_3[i] << " ";
  std::cout << std::endl;

  return 0;
}
  \endcode

  This example produces the following results:
  \code
Considered points: 
point 0: -0.2, 0.1
point 1: 0.3, 0.1
point 2: 0.2, -0.1
point 3: -0.2, -0.15

Basic moment available: 
m00=4  m10=0.1  m20=0.21  m30=0.019  m40=0.0129  
m01=0.00211  m11=-0.05  m21=0.02  m31=0.003  m41=0.0023  
m02=0.00057  m12=0  m22=0.0525  m32=-0.0015  m42=0.0026  
m03=9e-05  m13=0  m23=0  m33=-0.002375  m43=0.000575  
m04=-4.5e-05  m14=0  m24=0  m34=0  m44=0.00080625  

Basic moment avalaible: 
4	0.1	0.21	0.019	0.0129	0.00211	
-0.05	0.02	0.003	0.0023	0.00057	x	
0.0525	-0.0015	0.0026	9e-05	x	x	
-0.002375	0.000575	-4.5e-05	x	x	x	
0.00080625	-7.125e-05	x	x	x	x	
-6.59375e-05	x	x	x	x	x	

Direct acces to some basic moments: 
m00: 4
m10: 0.1
m01: -0.05
m22: 0.0026
m20: 0.21
m02: 0.0525

Common moments computed using basic moments:
Surface: 0.259375
Alpha: 0.133296
Centered moments (mu03, mu12, mu21, mu30): 0.003375 0.0045625 -0.00228125 -0.000421875 
  \endcode

  Note that in the continuous case, the moment object \f$m_{00}\f$ corresponds to the surface \f$a\f$ of the object.
  In the discrete case, it is the number of discrete points \f$n\f$.
*/
class VISP_EXPORT vpMomentObject{
public:
  /*!
    Type of object that will be considered.
  */
  typedef enum{
    DENSE_FULL_OBJECT = 0, /*!< A set of points (typically from an image) which are interpreted as being dense. */
    DENSE_POLYGON = 1, /*!< A set of points (stored in clockwise order) describing a polygon. It will be treated as dense. */
    DISCRETE = 2, /*!< A cloud of points. Treated as discrete. */
  } vpObjectType;
  vpMomentObject(unsigned int order);
  void fromImage(const vpImage<unsigned char>& image,unsigned char threshold, const vpCameraParameters& cam);
  void fromVector(std::vector<vpPoint>& points);
  std::vector<double>& get();
  double get(unsigned int i,unsigned int j) const;
  /*!
    \return The type of object that is considered.
  */
  vpObjectType getType() const {return type;}
  /*!
    \return The maximal order. The basic moments \f$m_{ij}\f$ that will be computed
    are for  \f$i+j \in [0:\mbox{order}]\f$.
  */
  unsigned int getOrder() const {return order-1;}
  /*!
    Specifies the type of the input data.
    \param type : An input type.
  */
  void setType(vpObjectType type){this->type=type;}
  friend VISP_EXPORT std::ostream & operator<<(std::ostream & os, const vpMomentObject& v);

private:
  unsigned int order;
  vpObjectType type;
  std::vector<double> values;
  void cacheValues(std::vector<double>& cache,double x, double y);
  double calc_mom_polygon(unsigned int p, unsigned int q, const std::vector<vpPoint>& points);

};

#endif
