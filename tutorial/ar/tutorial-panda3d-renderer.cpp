#include <iostream>
#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_PANDA3D)

#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DRendererSet.h>

#include <ambientLight.h>

int main()
{
  vpPanda3DRenderParameters renderParams(vpCameraParameters(300, 300, 160, 120), 240, 320, 0.001, 10.0);
  vpPanda3DRendererSet renderer(renderParams);
  renderer.setRenderParameters(renderParams);

  std::shared_ptr<vpPanda3DBaseRenderer> base = std::shared_ptr<vpPanda3DBaseRenderer>(new vpPanda3DBaseRenderer("base"));
  renderer.addSubRenderer(base);
  std::cout << "Initializing framework" << std::endl;
  renderer.initFramework(true);
  std::cout << "Loading object" << std::endl;
  NodePath object = renderer.loadObject("cube", "/home/sfelton/software/visp-sfelton/tutorial/ar/data/simple_cube.obj");
  std::cout << "Adding node to scene" <<std::endl;
  renderer.addNodeToScene(object);

  // PT(AmbientLight) alight = new AmbientLight("alight");
  // alight->set_color(LColor(0.2, 0.2, 0.2, 1.0));
  // NodePath alnp = renderer.getRenderRoot().attach_new_node(alight);
  // renderer.getRenderRoot().set_light(alnp);
  std::cout << "Setting camera pose" << std::endl;
  renderer.setCameraPose(vpHomogeneousMatrix(0.0, -5.0, 0.0, 0.0, 0.0, 0.0));
  while (true) {
    const double beforeRender = vpTime::measureTimeMs();
    renderer.renderFrame();
    const double afterRender = vpTime::measureTimeMs();
    const double delta = (afterRender - beforeRender) / 1000.0;
    vpHomogeneousMatrix wTo = renderer.getNodePose("cube");
    vpHomogeneousMatrix oToo = vpExponentialMap::direct(vpColVector({ 0.0, 0.0, 0.0, 0.0, 0.0, vpMath::rad(20.0) }), delta);
    renderer.setNodePose("cube", wTo * oToo);
    std::cout << "Rendering took: " << std::fixed << std::setprecision(2) << afterRender - beforeRender << "ms" << std::endl;
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
