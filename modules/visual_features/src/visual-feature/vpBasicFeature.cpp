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
 * Visual feature.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard
 *
 *****************************************************************************/

#include <visp3/visual_features/vpBasicFeature.h>

const unsigned int vpBasicFeature::FEATURE_LINE[32] = {
    (unsigned int)(1 << 0),  (unsigned int)(1 << 1),  (unsigned int)(1 << 2),  (unsigned int)(1 << 3),
    (unsigned int)(1 << 4),  (unsigned int)(1 << 5),  (unsigned int)(1 << 6),  (unsigned int)(1 << 7),
    (unsigned int)(1 << 8),  (unsigned int)(1 << 9),  (unsigned int)(1 << 10), (unsigned int)(1 << 11),
    (unsigned int)(1 << 12), (unsigned int)(1 << 13), (unsigned int)(1 << 14), (unsigned int)(1 << 15),
    (unsigned int)(1 << 16), (unsigned int)(1 << 17), (unsigned int)(1 << 18), (unsigned int)(1 << 19),
    (unsigned int)(1 << 20), (unsigned int)(1 << 21), (unsigned int)(1 << 22), (unsigned int)(1 << 23),
    (unsigned int)(1 << 24), (unsigned int)(1 << 25), (unsigned int)(1 << 26), (unsigned int)(1 << 27),
    (unsigned int)(1 << 28), (unsigned int)(1 << 29), (unsigned int)(1 << 30), (unsigned int)(1 << 31)};

/*!
  \file vpBasicFeature.cpp
  \brief Class that defines what is a visual feature.
*/
/*!
  Default constructor.
*/
vpBasicFeature::vpBasicFeature() : s(), dim_s(0), flags(NULL), nbParameters(0), deallocate(vpBasicFeature::user) {}

/*!
  Destructor that free allocated memory.
*/
vpBasicFeature::~vpBasicFeature()
{
  if (flags != NULL) {
    delete[] flags;
    flags = NULL;
  }
}

/*!
  Copy constructor.
*/
vpBasicFeature::vpBasicFeature(const vpBasicFeature &f)
  : s(), dim_s(0), flags(NULL), nbParameters(0), deallocate(vpBasicFeature::user)
{
  *this = f;
}

/*!
  Copy operator.
*/
vpBasicFeature &vpBasicFeature::operator=(const vpBasicFeature &f)
{
  s = f.s;
  dim_s = f.dim_s;
  nbParameters = f.nbParameters;
  deallocate = f.deallocate;
  if (flags)
    delete[] flags;
  flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = f.flags[i];

  return (*this);
}

//! Get the feature vector dimension.
unsigned int vpBasicFeature::getDimension(unsigned int select) const
{
  unsigned int dim = 0;
  if (dim_s > 31)
    return dim_s;
  for (unsigned int i = 0; i < s.getRows(); i++) {
    //	printf("%x %x %d \n",select, featureLine[i], featureLine[i] & select);
    if (FEATURE_LINE[i] & select)
      dim += 1;
  }
  return dim;
}

//! Get the feature vector  \f$\bf s\f$.
vpColVector vpBasicFeature::get_s(const unsigned int select) const
{
  vpColVector state(0), stateLine(1);
  // if s is higher than the possible selections (photometry), send back the
  // whole vector
  if (dim_s > 31)
    return s;

  for (unsigned int i = 0; i < dim_s; ++i) {
    if (FEATURE_LINE[i] & select) {
      stateLine[0] = s[i];
      state.stack(stateLine);
    }
  }
  return state;
}

void vpBasicFeature::resetFlags()
{
  if (flags != NULL) {
    for (unsigned int i = 0; i < nbParameters; i++)
      flags[i] = false;
  }
}

//! Set feature flags to true to prevent warning when re-computing the
//! interaction matrix without having updated the feature.
void vpBasicFeature::setFlags()
{
  if (flags != NULL) {
    for (unsigned int i = 0; i < nbParameters; i++)
      flags[i] = true;
  }
}

//! Compute the error between two visual features from a subset of the
//! possible features.
vpColVector vpBasicFeature::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0), eLine(1);
  if (dim_s <= 31) {
    for (unsigned int i = 0; i < dim_s; ++i) {
      if (FEATURE_LINE[i] & select) {
        eLine[0] = s[i] - s_star[i];
        e.stack(eLine);
        // std::cout << "dim_s <= 31"<<std::endl;
      }
    }
  } else {
    e.resize(dim_s);
    vpColVector sd = s_star.get_s();
    e = s - sd;
  }

  return e;
}

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
