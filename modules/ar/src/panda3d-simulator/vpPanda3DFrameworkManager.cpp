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

#include <visp3/ar/vpPanda3DFrameworkManager.h>

#if defined(VISP_HAVE_PANDA3D)

#include "load_prc_file.h"

BEGIN_VISP_NAMESPACE

void vpPanda3DFrameworkManager::initFramework()
{
  if (!m_frameworkIsOpen) {
    load_prc_file_data("",
      "gl-version 3 2\n"
      "textures-power-2 none\n"
      "back-buffers 0\n"
      "auto-flip 1\n"
      // "pstats-gpu-timing 1\n"
      // "gl-finish 1\n"
      // "texture-minfilter mipmap\n"
      "no-singular-invert\n"
      "load-file-type p3assimp\n"
      "audio-library-name null\n"
      "model-cache-dir\n");
    m_framework.open_framework();
    m_frameworkIsOpen = true;
  }
}

void vpPanda3DFrameworkManager::exit()
{
  m_framework.close_framework();
}

void vpPanda3DFrameworkManager::registerDisabledWindow(PointerTo<WindowFramework> wf)
{
  m_disabledWindows.push_back(wf);
}

void vpPanda3DFrameworkManager::disableAllOtherRenderers(PointerTo<WindowFramework> &active)
{
  GraphicsStateGuardian *activeGsg = active->get_graphics_output()->get_gsg();

  for (int i = 0; i < m_framework.get_num_windows(); ++i) {
    PointerTo<WindowFramework> fi = m_framework.get_window(i);
    GraphicsStateGuardian *g = fi->get_graphics_output()->get_gsg();
    g->set_active(g == activeGsg);

  }
}

void vpPanda3DFrameworkManager::enableAllRenderers()
{
  for (int i = 0; i < m_framework.get_num_windows(); ++i) {
    WindowFramework *fi = m_framework.get_window(i);
    if (std::find(m_disabledWindows.begin(), m_disabledWindows.end(), fi) == m_disabledWindows.end()) {

      fi->get_graphics_output()->get_gsg()->set_active(true);
    }
  }
}

END_VISP_NAMESPACE

#endif
