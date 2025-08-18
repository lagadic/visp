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

class vpPanda3DBaseRenderer;

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

public:
  virtual ~vpPanda3DFrameworkManager()
  { }
  void initFramework();

  void exit();

  PandaFramework &getFramework() { return m_framework; }

  void enableSingleRenderer(vpPanda3DBaseRenderer &renderer);

  void registerDisabledWindow(PointerTo<WindowFramework> wf);

  void disableAllOtherRenderers(PointerTo<WindowFramework> &active);

  void enableAllRenderers();

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
