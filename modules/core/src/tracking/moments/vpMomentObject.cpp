/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 *
 *****************************************************************************/

#include <stdexcept>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include <cmath>
#include <limits>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif
#include <cassert>

/*!
  Computes moments from a vector of points describing a polygon.
  The points must be stored in a clockwise order. Used internally.
  \param p : moment order (first index)
  \param q : moment order (second index)
  \param points : vector of points in a clockwise order

  \return moment value
*/
double vpMomentObject::calc_mom_polygon(unsigned int p, unsigned int q, const std::vector<vpPoint> &points)
{
  unsigned int i, k, l;
  double den, mom;
  double x_p_k;
  double y_l;
  double y_q_l;

  den = static_cast<double>((p + q + 2) * (p + q + 1) * vpMath::comb(p + q, p));

  mom = 0.0;
  for (i = 1; i <= points.size() - 1; i++) {
    double s = 0.0;
    double x_k = 1;
    for (k = 0; k <= p; k++) {
      y_l = 1;
      x_p_k = pow(points[i - 1].get_x(), (int)(p - k));
      for (l = 0; l <= q; l++) {
        y_q_l = pow(points[i - 1].get_y(), (int)(q - l));

        s += static_cast<double>(vpMath::comb(k + l, l) * vpMath::comb(p + q - k - l, q - l) * x_k * x_p_k * y_l *
                                 y_q_l);

        y_l *= points[i].get_y();
      }
      x_k *= points[i].get_x();
    }

    s *= ((points[i - 1].get_x()) * (points[i].get_y()) - (points[i].get_x()) * (points[i - 1].get_y()));
    mom += s;
  }
  mom /= den;
  return (mom);
}

/*!
  Caching to avoid redundant multiplications.

  \param cache : Lookup table that contains the order by order values. For
  example, if the order is 3, cache will contain:
\code
  1   x     x^2
  y   x*y   x^2*y
  y^2 x*y^2 x^2*y^2
\endcode

  \param x, y : Coordinates of a point.
*/
void vpMomentObject::cacheValues(std::vector<double> &cache, double x, double y)
{
  cache[0] = 1;

  for (unsigned int i = 1; i < order; i++)
    cache[i] = cache[i - 1] * x;

  for (unsigned int j = order; j < order * order; j += order)
    cache[j] = cache[j - order] * y;

  for (unsigned int j = 1; j < order; j++) {
    for (unsigned int i = 1; i < order - j; i++) {
      cache[j * order + i] = cache[j * order] * cache[i];
    }
  }
}

/*!
 * Manikandan.B
 * Need to cache intensity along with the coordinates for photometric moments
 */
void vpMomentObject::cacheValues(std::vector<double> &cache, double x, double y, double IntensityNormalized)
{

  cache[0] = IntensityNormalized;

  double invIntensityNormalized = 0.;
  if (std::fabs(IntensityNormalized) >= std::numeric_limits<double>::epsilon())
    invIntensityNormalized = 1.0 / IntensityNormalized;

  for (unsigned int i = 1; i < order; i++)
    cache[i] = cache[i - 1] * x;

  for (unsigned int j = order; j < order * order; j += order)
    cache[j] = cache[j - order] * y;

  for (unsigned int j = 1; j < order; j++) {
    for (unsigned int i = 1; i < order - j; i++) {
      cache[j * order + i] = cache[j * order] * cache[i] * invIntensityNormalized;
    }
  }
}

/*!
  Computes basic moments from a vector of points.
  There are two cases:
  -# Dense case specified by setType(vpMomentObject::DENSE_POLYGON):
     - The points are the vertices describing a closed contour polygon.
     - They must be stored in a clockwise order.
     - The first and the last points should be the same to close the contour.
  -# Discrete case specified by setType(vpMomentObject::DISCRETE)
     - The points will be interpreted as a discrete point cloud.

  \param points : Vector of points.

  The code below shows how to use this function to consider a dense object
defined by a closed contour.

  \code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>

int main()
{
  // Define the contour of an object by a 5 clockwise vertices on a plane
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices of the contour polygon

  p.set_x(-0.2); p.set_y(0.1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(+0.3); p.set_y(0.1); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(+0.2); p.set_y(-0.1); // coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(-0.15); // coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(0.1); // close the contour (vertex 5 = vertex 1)
  vec_p.push_back(p);

  vpMomentObject obj(4); // Create an image moment object with 4 as maximum order
  obj.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a countour polygon
  obj.fromVector(vec_p); // Init the dense object with the polygon

  return 0;
}

  \endcode

  This other example shows how to consider an object as a discrete set of four
points.

  \code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>

int main()
{
  // Define 4 discrete points on a plane
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the 4 points

  p.set_x(-0.2); p.set_y(0.1); // coordinates in meters in the image plane (point 1)
  vec_p.push_back(p);
  p.set_x(+0.3); p.set_y(0.1); // coordinates in meters in the image plane (point 2)
  vec_p.push_back(p);
  p.set_x(+0.2); p.set_y(-0.1); // coordinates in meters in the image plane (point 3)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(-0.15); // coordinates in meters in the image plane (point 4)
  vec_p.push_back(p);

  vpMomentObject obj(4); // Create an image moment object with 4 as maximum order
  obj.setType(vpMomentObject::DISCRETE); // The object is constituted by discrete points
  obj.fromVector(vec_p); // Init the dense object with the points

  return 0;
}

  \endcode
*/
void vpMomentObject::fromVector(std::vector<vpPoint> &points)
{
  if (type == vpMomentObject::DENSE_POLYGON) {
    if (std::fabs(points.rbegin()->get_x() - points.begin()->get_x()) > std::numeric_limits<double>::epsilon() ||
        std::fabs(points.rbegin()->get_y() - points.begin()->get_y()) > std::numeric_limits<double>::epsilon()) {
      points.resize(points.size() + 1);
      points[points.size() - 1] = points[0];
    }
    for (unsigned int j = 0; j < order * order; j++) {
      values[j] = calc_mom_polygon(j % order, j / order, points);
    }
  } else {
    std::vector<double> cache(order * order, 0.);
    values.assign(order * order, 0);
    for (unsigned int i = 0; i < points.size(); i++) {
      cacheValues(cache, points[i].get_x(), points[i].get_y());
      for (unsigned int j = 0; j < order; j++) {
        for (unsigned int k = 0; k < order - j; k++) {
          values[j * order + k] += cache[j * order + k];
        }
      }
    }
  }
}

/*!
  Computes basic moments from an image.
  There is no assumption made about whether the input is dense or discrete but
it's more common to use vpMomentObject::DENSE_FULL_OBJECT with this method.

  \param image : Image to consider.
  \param threshold : Pixels with a luminance lower than this threshold will be
considered. \param cam : Camera parameters used to convert pixels coordinates
in meters in the image plane.

  The code below shows how to use this function.
  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMomentObject.h>

int main()
{
  vpCameraParameters cam;             // Camera parameters used for pixel to
meter conversion vpImage<unsigned char> I(288, 384); // Image used to define
the object
  // ... Initialize the image

  unsigned char threshold = 128; // Gray level used to define which part of
the image belong to the dense object

  vpMomentObject obj(3); // Create an image moment object with 3 as maximum
order obj.fromImage(I, threshold, cam); // Initialize the object from the
image

  return 0;
}
  \endcode
*/

void vpMomentObject::fromImage(const vpImage<unsigned char> &image, unsigned char threshold,
                               const vpCameraParameters &cam)
{
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel shared(threshold)
  {
    std::vector<double> curvals(order * order);
    curvals.assign(order * order, 0.);

#pragma omp for nowait // automatically organize loop counter between threads
    for (int i = 0; i < (int)image.getCols(); i++) {
      for (int j = 0; j < (int)image.getRows(); j++) {
        unsigned int i_ = static_cast<unsigned int>(i);
        unsigned int j_ = static_cast<unsigned int>(j);
        if (image[j_][i_] > threshold) {
          double x = 0;
          double y = 0;
          vpPixelMeterConversion::convertPoint(cam, i_, j_, x, y);

          double yval = 1.;
          for (unsigned int k = 0; k < order; k++) {
            double xval = 1.;
            for (unsigned int l = 0; l < order - k; l++) {
              curvals[(k * order + l)] += (xval * yval);
              xval *= x;
            }
            yval *= y;
          }
        }
      }
    }

#pragma omp master // only set this variable in master thread
    {
      values.assign(order * order, 0.);
    }

#pragma omp barrier
    for (unsigned int k = 0; k < order; k++) {
      for (unsigned int l = 0; l < order - k; l++) {
#pragma omp atomic
        values[k * order + l] += curvals[k * order + l];
      }
    }
  }
#else
  std::vector<double> cache(order * order, 0.);
  values.assign(order * order, 0);
  for (unsigned int i = 0; i < image.getCols(); i++) {
    for (unsigned int j = 0; j < image.getRows(); j++) {
      if (image[j][i] > threshold) {
        double x = 0;
        double y = 0;
        vpPixelMeterConversion::convertPoint(cam, i, j, x, y);
        cacheValues(cache, x, y);
        for (unsigned int k = 0; k < order; k++) {
          for (unsigned int l = 0; l < order - k; l++) {
            values[k * order + l] += cache[k * order + l];
          }
        }
      }
    }
  }
#endif

  // Normalisation equivalent to sampling interval/pixel size delX x delY
  double norm_factor = 1. / (cam.get_px() * cam.get_py());
  for (std::vector<double>::iterator it = values.begin(); it != values.end(); ++it) {
    *it = (*it) * norm_factor;
  }
}

/*!
 * Manikandan. B
 * Photometric moments v2
 * Intended to be used by 'vpMomentObject's of type DENSE_FULL_OBJECT
 * @param image                   : Grayscale image
 * @param cam                     : Camera parameters (to change to )
 * @param bg_type                 : White/Black background surrounding the
 * image
 * @param normalize_with_pix_size : This flag if SET, the moments, after
 * calculation are normalized w.r.t  pixel size available from camera
 * parameters
 */
void vpMomentObject::fromImage(const vpImage<unsigned char> &image, const vpCameraParameters &cam,
                               vpCameraImgBckGrndType bg_type, bool normalize_with_pix_size)
{
  std::vector<double> cache(order * order, 0.);
  values.assign(order * order, 0);

  // (x,y) - Pixel co-ordinates in metres
  double x = 0;
  double y = 0;
  // for indexing into cache[] and values[]
  unsigned int idx = 0;
  unsigned int kidx = 0;

  double intensity = 0;

  // double Imax = static_cast<double>(image.getMaxValue());

  double iscale = 1.0;
  if (flg_normalize_intensity) { // This makes the image a probability density
                                 // function
    double Imax = 255.;          // To check the effect of gray level change. ISR Coimbra
    iscale = 1.0 / Imax;
  }

  if (bg_type == vpMomentObject::WHITE) {
    /////////// WHITE BACKGROUND ///////////
    for (unsigned int j = 0; j < image.getRows(); j++) {
      for (unsigned int i = 0; i < image.getCols(); i++) {
        x = 0;
        y = 0;
        intensity = (double)(image[j][i]) * iscale;
        double intensity_white = 1. - intensity;

        vpPixelMeterConversion::convertPoint(cam, i, j, x, y);
        cacheValues(cache, x, y, intensity_white); // Modify 'cache' which has
                                                   // x^p*y^q to x^p*y^q*(1 -
                                                   // I(x,y))

        // Copy to "values"
        for (unsigned int k = 0; k < order; k++) {
          kidx = k * order;
          for (unsigned int l = 0; l < order - k; l++) {
            idx = kidx + l;
            values[idx] += cache[idx];
          }
        }
      }
    }
  } else {
    /////////// BLACK BACKGROUND	///////////
    for (unsigned int j = 0; j < image.getRows(); j++) {
      for (unsigned int i = 0; i < image.getCols(); i++) {
        x = 0;
        y = 0;
        intensity = (double)(image[j][i]) * iscale;
        vpPixelMeterConversion::convertPoint(cam, i, j, x, y);

        // Cache values for fast moment calculation
        cacheValues(cache, x, y,
                    intensity); // Modify 'cache' which has x^p*y^q to x^p*y^q*I(x,y)

        // Copy to moments array 'values'
        for (unsigned int k = 0; k < order; k++) {
          kidx = k * order;
          for (unsigned int l = 0; l < order - k; l++) {
            idx = kidx + l;
            values[idx] += cache[idx];
          }
        }
      }
    }
  }

  if (normalize_with_pix_size) {
    // Normalisation equivalent to sampling interval/pixel size delX x delY
    double norm_factor = 1. / (cam.get_px() * cam.get_py());
    for (std::vector<double>::iterator it = values.begin(); it != values.end(); ++it) {
      *it = (*it) * norm_factor;
    }
  }
}

/*!
  Does exactly the work of the default constructor as it existed in the very
  first version of vpMomentObject
 */
void vpMomentObject::init(unsigned int orderinp)
{
  order = orderinp + 1;
  type = vpMomentObject::DENSE_FULL_OBJECT;
  flg_normalize_intensity = true; // By default, the intensity values are normalized
  values.resize((order + 1) * (order + 1));
  values.assign((order + 1) * (order + 1), 0);
}

/*!
  Helper to copy constructor
 */
void vpMomentObject::init(const vpMomentObject &objin)
{
  order = objin.getOrder() + 1;
  type = objin.getType();
  flg_normalize_intensity = objin.flg_normalize_intensity;
  values.resize(objin.values.size());
  values = objin.values;
}

/*!
  Default constructor.
  Initializes the object with the maximum used order. You cannot use higher
  order moments than the order of the moment object. The parameter specified
  is the highest desired included order. All orders up to this values will be
  computed. In other words, a vpMomentObject will compute all \f$ m_{ij} \f$
  moments with \f$ i+j \in [0..order] \f$.

  \param max_order : Maximum reached order (i+j) to be used. All
  considered i+j will be of order smaller or equal than this
  parameter. For example if this parameter is 5, all moment values of
  order 0 to 5 included will be computed.

  Mani : outsourced the constructor work to void init (unsigned int orderinp);
*/
vpMomentObject::vpMomentObject(unsigned int max_order)
  : flg_normalize_intensity(true), order(max_order + 1), type(vpMomentObject::DENSE_FULL_OBJECT), values()
{
  init(max_order);
}

/*!
  Copy constructor
 */
vpMomentObject::vpMomentObject(const vpMomentObject &srcobj)
  : flg_normalize_intensity(true), order(1), type(vpMomentObject::DENSE_FULL_OBJECT), values()
{
  init(srcobj);
}

/*!
  Returns all basic moment values \f$m_{ij}\f$ with \f$i \in
[0:\mbox{order}]\f$ and \f$j \in [0:\mbox{order}]\f$.

  \return Vector of moment values. To access \f$m_{ij}\f$, you have to read
vpMomentObject::get()[j*(order+1)+i].

  For example, if the maximal order is 3, the following values are provided:
  \code
m00 m10 m20 m01 m11 m21 m02 m12 m12 m30 m03
  \endcode

  To access for example to the basic moment m12, you should use this kind of
code:
\code
vpMomentObject obj(3);
// ... initialise the object using fromVector() or fromImage()
std::vector mij = obj.get();
double m12;
m12 = mij[2*(obj.getOrder()+1)+1]; // i=1 and j=2
  \endcode
*/
const std::vector<double> &vpMomentObject::get() const { return values; }

/*!
  Returns the basic moment value \f$m_{ij}\f$ corresponding to i,j indexes

  \param i : First moment index, with \f$i+j \leq order\f$.
  \param j : Second moment index, with \f$i+j \leq order\f$.
*/
double vpMomentObject::get(unsigned int i, unsigned int j) const
{
  assert(i + j <= getOrder());
  if (i + j >= order)
    throw vpException(vpException::badValue, "The requested value has not "
                                             "been computed, you should "
                                             "specify a higher order.");

  return values[j * order + i];
}

/*!
  Sets the basic moment value \f$m_{ij}\f$ corresponding to i,j indexes
  \param i : First moment index, with \f$i+j \leq order\f$.
  \param j : Second moment index, with \f$i+j \leq order\f$.
  \param value_ij : Moment value.
*/
void vpMomentObject::set(unsigned int i, unsigned int j, const double &value_ij)
{
  assert(i + j <= getOrder());
  if (i + j >= order)
    throw vpException(vpException::badValue, "The requested value cannot be set, you should specify "
                                             "a higher order for the moment object.");
  values[j * order + i] = value_ij;
}

/*!
  Outputs the basic moment's values \f$m_{ij}\f$ to a stream presented as a
  matrix. The first line corresponds to \f$m_{0[0:order]}\f$, the second one
  to \f$m_{1[0:order]}\f$ Values in table corresponding to a higher order are
  marked with an "x" and not computed.

  For example, if the maximal order is 3, the following values are provided:

  \code
  m00 m10 m20 m30
  m01 m11 m21 x
  m02 m12  x  x
  m03 x    x  x
  \endcode

*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentObject &m)
{
  for (unsigned int i = 0; i < m.values.size(); i++) {

    if (i % (m.order) == 0)
      os << std::endl;

    if ((i % (m.order) + i / (m.order)) < m.order)
      os << m.values[i];
    else
      os << "x";

    os << "\t";
  }

  return os;
}

/*!
  Outputs the raw moment values \f$m_{ij}\f$ in indexed form.
  The moment values are same as provided by the operator << which outputs x
  for uncalculated moments.
 */
void vpMomentObject::printWithIndices(const vpMomentObject &momobj, std::ostream &os)
{
  std::vector<double> moment = momobj.get();
  os << std::endl << "Order of vpMomentObject: " << momobj.getOrder() << std::endl;
  // Print out values. This is same as printing using operator <<
  for (unsigned int k = 0; k <= momobj.getOrder(); k++) {
    for (unsigned int l = 0; l < (momobj.getOrder() + 1) - k; l++) {
      os << "m[" << l << "," << k << "] = " << moment[k * (momobj.getOrder() + 1) + l] << "\t";
    }
    os << std::endl;
  }
  os << std::endl;
}

/*!
 This function returns a vpMatrix of size (order+1, order+1).
\code
 vpMomentObject obj(8);
 obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
 obj.fromImageWeighted(I, cam, vpMomentObject::BLACK); // cam should have the camera parameters
 vpMatrix Mpq = vpMomentObject::convertTovpMatrix(obj);
\endcode
 Instead of accessing the moment m21 as obj.get(2,1), you can now do
Mpq[2][1]. This is useful when you want to use the functions available in
vpMatrix. One use case i see now is to copy the contents of the matrix to a
file or std::cout. For instance, like
\code
 // Print to console
 Mpq.maplePrint(std::cout);
 // Or write to a file
 std::ofstream fileMpq("Mpq.csv");
 Mpq.maplePrint(fileMpq);
\endcode

The output can be copied and pasted to MAPLE as a matrix.

\warning
The moments that are not calculated have zeros. For instance, for a
vpMomentObject of order 8, the moment m[7,2] is not calculated. It will have 0
by default. User discretion is advised.
*/
vpMatrix vpMomentObject::convertTovpMatrix(const vpMomentObject &momobj)
{
  std::vector<double> moment = momobj.get();
  unsigned int order = momobj.getOrder();
  vpMatrix M(order + 1, order + 1);
  for (unsigned int k = 0; k <= order; k++) {
    for (unsigned int l = 0; l < (order + 1) - k; l++) {
      M[l][k] = moment[k * (order + 1) + l];
    }
  }
  return M;
}

/*!
  Nothing to destruct. This will allow for a polymorphic usage
  For instance,
  \code
  vpMomentObject* obj = new vpWeightedMomentObject(weightfunc,ORDER);
  \endcode
  where vpWeightedMomentObject is child class of vpMomentObject
 */
vpMomentObject::~vpMomentObject()
{
  // deliberate empty
}
