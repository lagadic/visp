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
 * Object input structure used by moments.
 *
 * Authors:
 * Filip Novotny
 * Manikandan Bakthavatchalam
 *****************************************************************************/
/*!
  \file vpMomentObject.h
  \brief Object input structure used by moments.
*/
#ifndef __MOMENTOBJECT_H__
#define __MOMENTOBJECT_H__

#include <cstdlib>
#include <utility>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMoment.h>
#include <visp3/core/vpPoint.h>

class vpCameraParameters;

/*!
  \class vpMomentObject

  \ingroup group_core_moments

  \brief Class for generic objects.

  It contains all basic moments often described by \f$m_{ij}\f$ of order
\f$i+j\f$ going from \f$m_{00}\f$ to the order used as parameter in
vpMomentObject() constructor. All other moments implemented in ViSP (gravity
center, alpha orientation, centered moments...) use this moment object as a
combination of its different values.

  When constructing a vpMomentObject() you need first to specify the maximum
used moment order as parameter.

  Then there are three ways to initialize a vpMomentObject. Firstly using
fromImage() you can considerer a dense object \e O defined by a binary image.
Secondly, as described in fromVector() you can also define a dense object \e O
by a closed contour. In these two cases, 2D basic moments are defined by:
  \f[m_{ij} = \int \int_{O} x^i y^j dx dy\f]

  Lastly, as presented in fromVector() you can consider a discrete set of \e n
points. In that last case, the basic moments are defined by \f[m_{ij} =
\sum_{k=1}^{n} x_k^i y_k^j \f]

  With setType() method you can specify the object type.


  \attention Be careful with the object order. When you specify a maximum
order in the vpMomentObject::vpMomentObject constructor (see its detailed
description), it will compute all moment orders up to the order you specified.
If you want to access the values \f$ m_{ij} \f$ with the vpMomentObject::get
method, you can do object.get()[j*(order+1)+i].

    A few tips about which orders to use in different situations:
    - moment based visual servoing: use vpMomentObject(6). This will compute
moment values up to order 6 which will enable vpFeatureMoments up to order 5
which is the maximum order required for common moments.
    - computing gravity center: use vpMomentObject(1). You only need \f$
m_{00},m_{01},m_{10} \f$. You should compute moments up to order 1.
    - computing gravity center interaction matrix
(vpFeatureMomentGravityCenter): use vpMomentObject(2). This will compute
moment values till order 2 since they are needed for the interaction matrix of
vpFeatureMoments of order 1.


  The following example shows how to create a moment object from 4 discrete
points locate on a plane one meter in front of the camera. It shows also how
to get the basic moments that are computed and how to compute other classical
moments such as the gravity center or the centered moments. \code #include
<visp3/core/vpMomentCommon.h> #include <visp3/core/vpMomentObject.h> #include
<visp3/core/vpPoint.h>

int main()
{
  // Define an object as 4 clockwise points on a plane (Z=0)
  std::vector<vpPoint> vec_p; // vector that contains the 4 points

  vec_p.push_back( vpPoint(-0.2, 0.1,  0.0) ); // values in meters
  vec_p.push_back( vpPoint(+0.3, 0.1,  0.0) ); // values in meters
  vec_p.push_back( vpPoint(+0.2,-0.1,  0.0) ); // values in meters
  vec_p.push_back( vpPoint(-0.2,-0.15, 0.0) ); // values in meters

  // These points are observed by a camera
  vpHomogeneousMatrix cMo(0, 0, 1, 0, 0, 0); // We set the camera to be 1m far
the object
  // ... update cMo from an image processing

  // Apply the perspective projection to update the points coordinates in the
camera plane for(unsigned int i=0; i<vec_p.size(); ++i) vec_p[i].project(cMo);

  std::cout << "Considered points: " << std::endl;
  for(unsigned int i=0; i<vec_p.size(); ++i)
    std::cout << "point " << i << ": " << vec_p[i].get_x() << ", " <<
vec_p[i].get_y() << std::endl;

  // Define an image moment object from the previous points
  vpMomentObject obj(5); // use moments up to order 5
  obj.setType(vpMomentObject::DISCRETE); // initialize the object as
constituted by discrete points obj.fromVector(vec_p); // init the object from
the points

  // --- Access the computed moments by querying the moment object

  // 1. Getting a vector of doubles
  std::vector<double> moment = obj.get();
  std::cout << std::endl << "Basic moment available (from vector of doubles) "
<< std::endl; for(unsigned int k=0; k<=obj.getOrder(); k++) { for(unsigned int
l=0; l<(obj.getOrder()+1)-k; l++){ std::cout << "m" << l << k << "=" <<
moment[k*(momobj.getOrder()+1)+ l] << "\t";
        }
        std::cout<<std::endl;
    }

  // 2. Print the contents of moment object directly
  std::cout << std::endl << "Basic moment available: ";
  std::cout << obj << std::endl;

  // 3. Directly indexing the moment object
  std::cout << std::endl << "Direct acces to some basic moments: " <<
std::endl; std::cout << "m00: " << obj.get(0, 0) << std::endl; std::cout <<
"m10: " << obj.get(1, 0) << std::endl; std::cout << "m01: " << obj.get(0, 1)
<< std::endl; std::cout << "m22: " << obj.get(2, 2) << std::endl; std::cout <<
"m20: " << obj.get(2, 0) << std::endl; std::cout << "m02: " << obj.get(0, 2)
<< std::endl;

  // Get common moments computed using basic moments
  double m00 = vpMomentCommon::getSurface(obj); // surface = m00
  double alpha = vpMomentCommon::getAlpha(obj); // orientation
  std::vector<double> mu_3 = vpMomentCommon::getMu3(obj); // centered moment
up to 3rd order

  std::cout << std::endl << "Common moments computed using basic moments:" <<
std::endl; std::cout << "Surface: " << m00 << std::endl; std::cout << "Alpha:
" << alpha << std::endl; std::cout << "Centered moments (mu03, mu12, mu21,
mu30): "; for(unsigned int i=0; i<mu_3.size(); ++i) std::cout << mu_3[i] << "
"; std::cout << std::endl;

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

Basic moment available (from vector of doubles):
m00=4   m10=0.1 m20=0.21        m30=0.019       m40=0.0129      m50=0.00211
m01=-0.05       m11=0.02        m21=0.003       m31=0.0023      m41=0.00057
m02=0.0525      m12=-0.0015     m22=0.0026      m32=9e-05
m03=-0.002375   m13=0.000575    m23=-4.5e-05
m04=0.00080625  m14=-7.125e-05
m05=-6.59375e-05

Basic moment available:
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

  Note that in the continuous case, the moment object \f$m_{00}\f$ corresponds
to the surface \f$a\f$ of the object. In the discrete case, it is the number
of discrete points \f$n\f$.
*/
class VISP_EXPORT vpMomentObject
{
public:
  /*!
    Type of object that will be considered.
  */
  typedef enum {
    DENSE_FULL_OBJECT = 0, /*!< A set of points (typically from an image)
                              which are interpreted as being dense. */
    DENSE_POLYGON = 1,     /*!< A set of points (stored in clockwise order)
                              describing a polygon. It will be treated as dense.
                            */
    DISCRETE = 2,          /*!< A cloud of points. Treated as discrete. */
  } vpObjectType;

  /*!
     Type of camera image background.
   */
  typedef enum {
    BLACK = 0, /*! Black background */
    WHITE = 1, /*! No functionality as of now */
  } vpCameraImgBckGrndType;

  bool flg_normalize_intensity; // To scale the intensity of each individual
                                // pixel in the image by the maximum intensity
                                // value present in it

  // Constructors
  explicit vpMomentObject(unsigned int order);
  vpMomentObject(const vpMomentObject &srcobj);
  /*!
  Virtual destructor to allow polymorphic usage.
  For instance,
  \code
  vpMomentObject* obj = new vpWeightedMomentObject(weightfunc,ORDER);
  \endcode
  where vpWeightedMomentObject is child class of vpMomentObject
  */
  virtual ~vpMomentObject();

  void fromImage(const vpImage<unsigned char> &image, unsigned char threshold,
                 const vpCameraParameters &cam); // Binary version
  void fromImage(const vpImage<unsigned char> &image, const vpCameraParameters &cam, vpCameraImgBckGrndType bg_type,
                 bool normalize_with_pix_size = true); // Photometric version

  void fromVector(std::vector<vpPoint> &points);
  const std::vector<double> &get() const;
  double get(unsigned int i, unsigned int j) const;

  /*!
    \return The type of object that is considered.
  */
  vpObjectType getType() const { return type; }

  /*!
    \return The maximal order. The basic moments \f$m_{ij}\f$ that will be
    computed are for  \f$i+j \in [0:\mbox{order}]\f$.
  */
  unsigned int getOrder() const { return order - 1; }

  // Constructor helpers
  void init(unsigned int orderinp);
  void init(const vpMomentObject &objin);

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentObject &v);
  /*!
    Outputs raw moments in indexed form like m[1,1] = value of moment m11
    \param momobj : A vpMomentObject
    \param os : Output stream.
   */
  static void printWithIndices(const vpMomentObject &momobj, std::ostream &os);
  /*!
    Specifies the type of the input data.
    \param input_type : An input type.
  */
  void setType(vpObjectType input_type) { this->type = input_type; }

  /*!
    Converts the raw moments contained in vpMomentObject to a vpMatrix
    \param momobj : A vpMomentObject
   */
  static vpMatrix convertTovpMatrix(const vpMomentObject &momobj);

protected:
  unsigned int order;
  vpObjectType type;
  std::vector<double> values;
  void set(unsigned int i, unsigned int j, const double &value_ij);
  void cacheValues(std::vector<double> &cache, double x, double y);

private:
  void cacheValues(std::vector<double> &cache, double x, double y, double IntensityNormalized);
  double calc_mom_polygon(unsigned int p, unsigned int q, const std::vector<vpPoint> &points);
};

#endif
