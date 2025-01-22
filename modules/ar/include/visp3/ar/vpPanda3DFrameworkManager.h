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
 */

#ifndef VP_PANDA3D_FRAMEWORK_MANAGER_H
#define VP_PANDA3D_FRAMEWORK_MANAGER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <pandaFramework.h>
#include <pandaSystem.h>
#include "load_prc_file.h"

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d
 *
 * \brief Base class for a panda3D renderer. This class handles basic functionalities,
 * such as loading object, changing camera parameters.
 *
 * For a subclass to have a novel behaviour (e.g, display something else) These methods should be overriden:
 *
 * - setupScene: This is where you should apply your shaders.
 * - setupCamera: This is where cameras are created and intrinsics parameters are applied
 * - setupRenderTarget: This is where you should create the texture buffers, where the render results should be stored.
*/
class VISP_EXPORT vpPanda3DFrameworkManager
{
private:

  vpPanda3DFrameworkManager() : m_frameworkIsOpen(false)
  { }

  virtual ~vpPanda3DFrameworkManager()
  { }


public:

  void initFramework()
  {
    if (!m_frameworkIsOpen) {
      load_prc_file_data("", "textures-power-2 none");
      load_prc_file_data("", "gl-version 3 2");
      load_prc_file_data("", "no-singular-invert");
      m_framework.open_framework();
      m_frameworkIsOpen = true;
    }

  }

  void exit()
  {
    m_framework.close_framework();
  }

  PandaFramework &getFramework() { return m_framework; }

  void registerDisabledWindow(PointerTo<WindowFramework> wf)
  {

    m_disabledWindows.push_back(wf);
  }

  void disableAllOtherRenderers(PointerTo<WindowFramework> &active)
  {
    for (int i = 0; i < m_framework.get_num_windows(); ++i) {
      PointerTo<WindowFramework> fi = m_framework.get_window(i);
      if (fi != active) {
        fi->get_graphics_output()->get_gsg()->set_active(false);
      }
    }
  }

  void enableAllRenderers()
  {
    for (int i = 0; i < m_framework.get_num_windows(); ++i) {
      WindowFramework *fi = m_framework.get_window(i);
      if (std::find(m_disabledWindows.begin(), m_disabledWindows.end(), fi) == m_disabledWindows.end()) {
        fi->get_graphics_output()->get_gsg()->set_active(true);
      }
    }
  }



  static vpPanda3DFrameworkManager &getInstance()
  {
    static vpPanda3DFrameworkManager instance;
    return instance;
  }

protected:

  PandaFramework m_framework; //! Panda Rendering framework
  bool m_frameworkIsOpen;
  std::vector<PointerTo<WindowFramework>> m_disabledWindows;

};

END_VISP_NAMESPACE
#endif //VISP_HAVE_PANDA3D
#endif
