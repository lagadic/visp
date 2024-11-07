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
 * Python bindings.
 */

#ifndef VISP_PYTHON_VISUAL_FEATURES_HPP
#define VISP_PYTHON_VISUAL_FEATURES_HPP

#include <visp3/visual_features/vpBasicFeature.h>

#include <pybind11/pybind11.h>


class TrampolineBasicFeature : public vpBasicFeature
{
public:
  using vpBasicFeature::vpBasicFeature;

  TrampolineBasicFeature() : vpBasicFeature() { }
  TrampolineBasicFeature(const vpBasicFeature &f) : vpBasicFeature(f) { }


  virtual void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                         const vpColor &color = vpColor::green, unsigned int thickness = 1) const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpBasicFeature,     /* Parent class */
      display,        /* Name of function in C++ (must match Python name) */
      cam, I, color, thickness
    );
  }
  virtual void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
                       unsigned int thickness = 1) const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpBasicFeature,     /* Parent class */
      display,        /* Name of function in C++ (must match Python name) */
      cam, I, color, thickness
    );
  }

  virtual void init() VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpBasicFeature,     /* Parent class */
      init,        /* Name of function in C++ (must match Python name) */ // Intended comma at the end
      );
  }

  virtual vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) VP_OVERRIDE
  {
    PYBIND11_OVERRIDE(
      vpColVector,           /* Return type */
      vpBasicFeature,     /* Parent class */
      error,        /* Name of function in C++ (must match Python name) */
      s_star,              /* Argument(s) */
      select
    );
  }


  virtual vpMatrix interaction(unsigned int select = FEATURE_ALL) VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      vpMatrix,           /* Return type */
      vpBasicFeature,     /* Parent class */
      interaction,        /* Name of function in C++ (must match Python name) */
      select              /* Argument(s) */
    );
  }
  virtual void print(unsigned int select = FEATURE_ALL) const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpBasicFeature,     /* Parent class */
      print,        /* Name of function in C++ (must match Python name) */
      select              /* Argument(s) */
    );
  }

  virtual vpBasicFeature *duplicate() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      vpBasicFeature *,           /* Return type */
      vpBasicFeature,     /* Parent class */
      duplicate,        /* Name of function in C++ (must match Python name) */ // Intended comma at the end
      );
  }

};




#endif
