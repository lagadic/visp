#include <iostream>
#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_PANDA3D)

#include <visp3/core/vpException.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>

int main()
{
  vpPanda3DRenderParameters renderParams(vpCameraParameters(300, 300, 160, 120), 240, 320, 0.001, 1.0);
  vpPanda3DBaseRenderer renderer("basic");
  renderer.setRenderParameters(renderParams);
  renderer.initFramework(true);
  NodePath object = renderer.loadObject("cube", "/home/sfelton/software/visp-sfelton/tutorial/ar/data/simple_cube.obj");
  renderer.addNodeToScene(object);
  renderer.setCameraPose(vpHomogeneousMatrix(0.0, 0.0, -0.75, 0.0, 0.0, 0.0));
  while (true) {
    renderer.renderFrame();
  }
  return 0;
}

#else

int main()
{

  std::cerr << "Recompile ViSP with Panda3D as a third party to run this tutorial" << std::endl;
  return EXIT_FAILURE;
}

#endif
