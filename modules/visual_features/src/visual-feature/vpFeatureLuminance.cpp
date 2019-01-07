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
 * Luminance feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include <visp3/visual_features/vpFeatureLuminance.h>

/*!
  \file vpFeatureLuminance.cpp
  \brief Class that defines the image luminance visual feature

  For more details see \cite Collewet08c.
*/

/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void vpFeatureLuminance::init()
{
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // default value Z (1 meters)
  Z = 1;

  firstTimeIn = 0;

  nbr = nbc = 0;
}

void vpFeatureLuminance::init(unsigned int _nbr, unsigned int _nbc, double _Z)
{
  init();

  nbr = _nbr;
  nbc = _nbc;

  if ((nbr < 2 * bord) || (nbc < 2 * bord)) {
    throw vpException(vpException::dimensionError, "border is too important compared to number of row or column.");
  }

  // number of feature = nb column x nb lines in the images
  dim_s = (nbr - 2 * bord) * (nbc - 2 * bord);

  s.resize(dim_s);

  if (pixInfo != NULL)
    delete[] pixInfo;

  pixInfo = new vpLuminance[dim_s];

  Z = _Z;
}

/*!
  Default constructor that build a visual feature.
*/
vpFeatureLuminance::vpFeatureLuminance() : Z(1), nbr(0), nbc(0), bord(10), pixInfo(NULL), firstTimeIn(0), cam()
{
  nbParameters = 1;
  dim_s = 0;
  flags = NULL;

  init();
}

/*!
 Copy constructor.
 */
vpFeatureLuminance::vpFeatureLuminance(const vpFeatureLuminance &f)
  : vpBasicFeature(f), Z(1), nbr(0), nbc(0), bord(10), pixInfo(NULL), firstTimeIn(0), cam()
{
  *this = f;
}

/*!
 Copy operator.
 */
vpFeatureLuminance &vpFeatureLuminance::operator=(const vpFeatureLuminance &f)
{
  Z = f.Z;
  nbr = f.nbr;
  nbc = f.nbc;
  bord = f.bord;
  firstTimeIn = f.firstTimeIn;
  cam = f.cam;
  if (pixInfo)
    delete[] pixInfo;
  pixInfo = new vpLuminance[dim_s];
  for (unsigned int i = 0; i < dim_s; i++)
    pixInfo[i] = f.pixInfo[i];
  return (*this);
}

/*!
  Destructor that free allocated memory.
*/
vpFeatureLuminance::~vpFeatureLuminance()
{
  if (pixInfo != NULL)
    delete[] pixInfo;
}

/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera
  frame.

  \param Z_ : \f$ Z \f$ value to set.
*/
void vpFeatureLuminance::set_Z(const double Z_)
{
  this->Z = Z_;
  flags[0] = true;
}

/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera
  frame.

  \return The value of \f$ Z \f$.
*/
double vpFeatureLuminance::get_Z() const { return Z; }

void vpFeatureLuminance::setCameraParameters(vpCameraParameters &_cam) { cam = _cam; }

/*!

  Build a luminance feature directly from the image
*/

void vpFeatureLuminance::buildFrom(vpImage<unsigned char> &I)
{
  unsigned int l = 0;
  double Ix, Iy;

  double px = cam.get_px();
  double py = cam.get_py();

  if (firstTimeIn == 0) {
    firstTimeIn = 1;
    l = 0;
    for (unsigned int i = bord; i < nbr - bord; i++) {
      //   cout << i << endl ;
      for (unsigned int j = bord; j < nbc - bord; j++) {
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, j, i, x, y);

        pixInfo[l].x = x;
        pixInfo[l].y = y;

        pixInfo[l].Z = Z;

        l++;
      }
    }
  }

  l = 0;
  for (unsigned int i = bord; i < nbr - bord; i++) {
    //   cout << i << endl ;
    for (unsigned int j = bord; j < nbc - bord; j++) {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      Ix = px * vpImageFilter::derivativeFilterX(I, i, j);
      Iy = py * vpImageFilter::derivativeFilterY(I, i, j);

      // Calcul de Z

      pixInfo[l].I = I[i][j];
      s[l] = I[i][j];
      pixInfo[l].Ix = Ix;
      pixInfo[l].Iy = Iy;

      l++;
    }
  }
}

/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is
  made thanks to the values of the luminance features \f$ I \f$
*/
void vpFeatureLuminance::interaction(vpMatrix &L)
{
  L.resize(dim_s, 6);

  for (unsigned int m = 0; m < L.getRows(); m++) {
    double Ix = pixInfo[m].Ix;
    double Iy = pixInfo[m].Iy;

    double x = pixInfo[m].x;
    double y = pixInfo[m].y;
    double Zinv = 1 / pixInfo[m].Z;

    {
      L[m][0] = Ix * Zinv;
      L[m][1] = Iy * Zinv;
      L[m][2] = -(x * Ix + y * Iy) * Zinv;
      L[m][3] = -Ix * x * y - (1 + y * y) * Iy;
      L[m][4] = (1 + x * x) * Ix + Iy * x * y;
      L[m][5] = Iy * x - Ix * y;
    }
  }
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is
  made thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix vpFeatureLuminance::interaction(const unsigned int /* select */)
{
  /* static */ vpMatrix L; // warning C4640: 'L' : construction of local
                           // static object is not thread-safe
  interaction(L);
  return L;
}

/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired

  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
void vpFeatureLuminance::error(const vpBasicFeature &s_star, vpColVector &e)
{
  e.resize(dim_s);

  for (unsigned int i = 0; i < dim_s; i++) {
    e[i] = s[i] - s_star[i];
  }
}

/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired

  \param s_star : Desired visual feature.
  \param select : Not used.

*/
vpColVector vpFeatureLuminance::error(const vpBasicFeature &s_star, const unsigned int /* select */)
{
  /* static */ vpColVector e; // warning C4640: 'e' : construction of local
                              // static object is not thread-safe

  error(s_star, e);

  return e;
}

/*!

  Not implemented.

 */
void vpFeatureLuminance::print(const unsigned int /* select */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void vpFeatureLuminance::display(const vpCameraParameters & /* cam */, const vpImage<unsigned char> & /* I */,
                                 const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void vpFeatureLuminance::display(const vpCameraParameters & /* cam */, const vpImage<vpRGBa> & /* I */,
                                 const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
vpFeatureLuminance *vpFeatureLuminance::duplicate() const
{
  vpFeatureLuminance *feature = new vpFeatureLuminance;
  return feature;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
