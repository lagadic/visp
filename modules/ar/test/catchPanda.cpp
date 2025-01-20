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
 * Test vpCameraParameters JSON parse / save.
 */

/*!
  \example catchPanda.cpp

  Test saving and parsing JSON configuration for Panda 3D renderer.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D) && defined(VISP_HAVE_CATCH2)
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DRGBRenderer.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <visp3/core/vpCameraParameters.h>
#include <catch_amalgamated.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#include <random>

vpPanda3DRenderParameters defaultRenderParams()
{
  vpCameraParameters cam(600, 600, 160, 120);
  return vpPanda3DRenderParameters(cam, 240, 320, 0.001, 1.0);
}

// SCENARIO("Instanciating multiple Panda3D renderers", "[Panda3D]")
// {
//   GIVEN("A single renderer")
//   {
//     vpPanda3DGeometryRenderer r1(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
//     r1.setRenderParameters(defaultRenderParams());
//     r1.initFramework();
//     r1.renderFrame();
//     vpImage<float> depth;
//     r1.getRender(depth);

//     THEN("Creating another, uncoupled renderer is ok and its destruction does not raise an error")
//     {
//       std::cout << "Creating r2" << std::endl;
//       vpPanda3DGeometryRenderer r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
//       r2.setRenderParameters(defaultRenderParams());
//       r2.initFramework();
//       std::cout << "After r2 initialization" << std::endl;
//       r2.renderFrame();
//       std::cout << "After r2 frame" << std::endl;


//     }
//     std::cout << "Calling r1.renderFrame after r2 destruction" << std::endl;
//     r1.renderFrame();
//     std::cout << "Fetching depth after r2 destruction" << std::endl;

//     r1.getRender(depth);
//   }
// }


SCENARIO("Sequentially instanciating and destroying Panda3D renderers", "[Panda3D]")
{
  vpPanda3DGeometryRenderer r3(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
  r3.setRenderParameters(defaultRenderParams());
  r3.initFramework();
  r3.renderFrame();
  vpImage<float> depth;
  r3.getRender(depth);

  {
    vpPanda3DGeometryRenderer r1(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
    r1.setRenderParameters(defaultRenderParams());
    r1.initFramework();
    r1.renderFrame();
    r1.getRender(depth);
  }

  {
    std::cout << "Creating r2" << std::endl;
    vpPanda3DGeometryRenderer r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
    r2.setRenderParameters(defaultRenderParams());
    r2.initFramework();
    vpImage<float> depth;
    r2.renderFrame();
    r2.getRender(depth);
    std::cout << "After r2 initialization" << std::endl;

  }
}

// SCENARIO("Using multiple panda3d renderers in parallel", "[Panda3D]")
// {


//   vpPanda3DGeometryRenderer r1(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
//   r1.setRenderParameters(defaultRenderParams());
//   r1.initFramework();
//   r1.renderFrame();
//   vpImage<float> depth;
//   r1.getRender(depth);

//   vpPanda3DGeometryRenderer r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
//   r2.setRenderParameters(defaultRenderParams());
//   r2.initFramework();
//   r2.renderFrame();
//   r2.getRender(depth);


//   r1.renderFrame();
//   r1.getRender(depth);

// }


int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

#include <stdlib.h>

int main() { return EXIT_SUCCESS; }

#endif
