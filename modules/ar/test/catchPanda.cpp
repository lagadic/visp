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
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DRGBRenderer.h>
#include <visp3/ar/vpPanda3DFrameworkManager.h>
#include <visp3/ar/vpPanda3DRendererSet.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <catch_amalgamated.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#include <random>

const std::string objCube =
"o Cube\n"
"v -0.050000 -0.050000 0.050000\n"
"v -0.050000 0.050000 0.050000\n"
"v -0.050000 -0.050000 -0.050000\n"
"v -0.050000 0.050000 -0.050000\n"
"v 0.050000 -0.050000 0.050000\n"
"v 0.050000 0.050000 0.050000\n"
"v 0.050000 -0.050000 -0.050000\n"
"v 0.050000 0.050000 -0.050000\n"
"f 2/4/1 3/8/1 1/1/1\n"
"f 4/9/2 7/13/2 3/8/2\n"
"f 8/14/3 5/11/3 7/13/3\n"
"f 6/12/4 1/2/4 5/11/4\n"
"f 7/13/5 1/3/5 3/7/5\n"
"f 4/10/6 6/12/6 8/14/6\n"
"f 2/4/1 4/9/1 3/8/1\n"
"f 4/9/2 8/14/2 7/13/2\n"
"f 8/14/3 6/12/3 5/11/3\n"
"f 6/12/4 2/5/4 1/2/4\n"
"f 7/13/5 5/11/5 1/3/5\n"
"f 4/10/6 2/6/6 6/12/6\n";


std::string createObjFile()
{
  const std::string tempDir = vpIoTools::makeTempDirectory("visp_test_rbt_obj");
  const std::string objFile = vpIoTools::createFilePath(tempDir, "cube.obj");
  std::ofstream f(objFile);
  std::cout << objFile << std::endl;
  f << objCube;
  f.close();

  return objFile;
}

vpPanda3DRenderParameters defaultRenderParams()
{
  vpCameraParameters cam(600, 600, 160, 120);
  return vpPanda3DRenderParameters(cam, 240, 320, 0.001, 1.0);
}

bool opt_no_display = false; // If true, disable display or tests requiring display

SCENARIO("Instanciating multiple Panda3D renderers", "[Panda3D]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
  }
  else {
    GIVEN("A single renderer")
    {
      vpPanda3DGeometryRenderer r1(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
      r1.setRenderParameters(defaultRenderParams());
      r1.initFramework();
      r1.renderFrame();
      vpImage<float> depth;
      r1.getRender(depth);

      THEN("Creating another, uncoupled renderer is ok and its destruction does not raise an error")
      {
        vpPanda3DGeometryRenderer r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
        r2.setRenderParameters(defaultRenderParams());
        r2.initFramework();
        r2.renderFrame();
      }
      r1.renderFrame();

      r1.getRender(depth);
    }
  }
}


SCENARIO("Sequentially instanciating and destroying Panda3D renderers", "[Panda3D]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
  }
  else {
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
      vpPanda3DGeometryRenderer r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
      r2.setRenderParameters(defaultRenderParams());
      r2.initFramework();
      vpImage<float> depth;
      r2.renderFrame();
      r2.getRender(depth);

    }
  }
}

SCENARIO("Sequentially instanciating and destroying Panda3D renderer sets", "[Panda3D]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
  }
  else {
    {
      vpPanda3DRendererSet r1(defaultRenderParams());
      r1.addSubRenderer(std::make_shared<vpPanda3DGeometryRenderer>(vpPanda3DGeometryRenderer::CAMERA_NORMALS));
      r1.addSubRenderer(std::make_shared<vpPanda3DRGBRenderer>(true));
      r1.initFramework();
      r1.renderFrame();
    }

    {
      vpPanda3DRendererSet r1(defaultRenderParams());
      r1.addSubRenderer(std::make_shared<vpPanda3DGeometryRenderer>(vpPanda3DGeometryRenderer::CAMERA_NORMALS));
      r1.addSubRenderer(std::make_shared<vpPanda3DRGBRenderer>(true));
      r1.initFramework();
      r1.renderFrame();
    }
  }
}

SCENARIO("Using multiple panda3d renderers in parallel", "[Panda3D]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
  }
  else {
    vpPanda3DRGBRenderer r3(true);
    r3.setRenderParameters(defaultRenderParams());
    r3.initFramework();
    r3.renderFrame();

    vpPanda3DGeometryRenderer r1(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
    r1.setRenderParameters(defaultRenderParams());
    r1.initFramework();
    r1.renderFrame();
    vpImage<float> depth;
    r1.getRender(depth);

    vpPanda3DGeometryRenderer r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS);
    r2.setRenderParameters(defaultRenderParams());
    r2.initFramework();
    r2.renderFrame();
    r2.getRender(depth);


    r1.renderFrame();
    r1.getRender(depth);
  }
}

TEST_CASE("Testing Geometry renderer", "[panda3D]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
    return;
  }
  vpPanda3DGeometryRenderer r1(vpPanda3DGeometryRenderer::CAMERA_NORMALS, false), r2(vpPanda3DGeometryRenderer::CAMERA_NORMALS, true);
  vpPanda3DGeometryRenderer r3(vpPanda3DGeometryRenderer::OBJECT_NORMALS, false), r4(vpPanda3DGeometryRenderer::OBJECT_NORMALS, true);


  std::vector<vpPanda3DGeometryRenderer *> renderers = { &r1, &r2, &r3, &r4 };

  std::vector<vpHomogeneousMatrix> cameraPoses = {
    vpHomogeneousMatrix(0.0, 0.0, 0.5, 0.0, vpMath::rad(15), 0.0),
    vpHomogeneousMatrix(0.05, 0.0, 0.4, vpMath::rad(5), 0.0, 0.0),
  };

  for (const vpHomogeneousMatrix &p: cameraPoses) {
    std::vector<vpImage<vpRGBf>> normalsImages;
    std::vector<vpImage<float>> depthImages;

    vpPanda3DRenderParameters params = defaultRenderParams();
    const std::string objId = "object";
    for (const auto r: renderers) {
      r->setRenderParameters(params);
      r->initFramework();
      r->addNodeToScene(r->loadObject(objId, createObjFile()));
      r->setNodePose(objId, vpHomogeneousMatrix(0, 0, 0, 0, 0, 0));
      r->setCameraPose(p.inverse());
      r->renderFrame();

      vpImage<float> depth, depthBB, depthSolo;
      vpImage<vpRGBf> normals(params.getImageHeight(), params.getImageWidth(), vpRGBf(0, 0, 0)), normalsBB, normalsSolo;

      r->getRender(normals, depth);
      r->getRender(normalsSolo);
      r->getRender(depthSolo);
      r->getRender(normalsBB, depthBB, vpRect(0, 0, params.getImageWidth(), params.getImageHeight()), params.getImageHeight(), params.getImageWidth());
      // Using different getter lead to the same output
      REQUIRE((depth.getMinValue() == 0 && depth.getMaxValue() > 0.0));
      REQUIRE(depth == depthSolo);
      REQUIRE(depth == depthBB);
      REQUIRE((normals.getSum() != 0.0));
      REQUIRE(normals == normalsSolo);
      REQUIRE(normals == normalsBB);
      normalsImages.push_back(normals);
      depthImages.push_back(depth);
    }
    // Fast rendering difference is small enough
    const float thresholdN = 1 / 127.5f;
    const float thresholdDepth = params.getFarClippingDistance() / 255.f;
    for (unsigned int i = 0; i < normalsImages.size() - 1; ++i) {
      for (unsigned int j = i + 1; j < normalsImages.size(); ++j) {
        if (renderers[i]->getRenderType() == renderers[j]->getRenderType()) {

          std::cout << "Comparing renderers" << i << " and " << j << std::endl;

          for (unsigned int b = 0; b < depthImages[i].getSize(); ++b) {

            float depthDiff = fabs(depthImages[i].bitmap[b] - depthImages[j].bitmap[b]);
            if (depthDiff > thresholdDepth) {
              FAIL("Depth error is too great");
            }
            if (depthImages[i].bitmap[b] > 0.0) {
              const vpRGBf c1 = normalsImages[i].bitmap[b];
              const vpRGBf c2 = normalsImages[j].bitmap[b];


              if (fabs(c1.R - c2.R) > thresholdN ||  fabs(c1.G - c2.G) > thresholdN ||  fabs(c1.B - c2.B) > thresholdN) {
                std::cout << c1 << c2 << std::endl;
                FAIL("Normal error is too great");
              }
            }
          }
        }
      }
    }

  }

}


int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  auto cli = session.cli()
    | Catch::Clara::Opt(opt_no_display)["--no-display"]("Disable display");
  session.cli(cli);

  const int returnCode = session.applyCommandLine(argc, argv);
  if (returnCode != 0) { // Indicates a command line error
    return returnCode;
  }

  const int numFailed = session.run();
  vpPanda3DFrameworkManager::getInstance().exit();

  return numFailed;
}

#else

#include <stdlib.h>

int main() { return EXIT_SUCCESS; }

#endif
