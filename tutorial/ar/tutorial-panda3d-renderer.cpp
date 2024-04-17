#include <iostream>
#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_PANDA3D)

#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DRendererSet.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include "load_prc_file.h"
#include <ambientLight.h>

#include <visp3/gui/vpDisplayX.h>

void convertToDisplay(const vpImage<vpRGBf> &normalsImage, const vpImage<float> &depthImage,
                      vpImage<vpRGBa> &normalDisplayImage, vpImage<unsigned char> &depthDisplayImage, float minDepth, float maxDepth)
{
  std::cout << depthImage[0][0] << std::endl;
#pragma omp parallel for simd
  for (unsigned int i = 0; i < normalsImage.getSize(); ++i) {
    normalDisplayImage.bitmap[i].R = static_cast<unsigned char>((normalsImage.bitmap[i].R + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].G = static_cast<unsigned char>((normalsImage.bitmap[i].G + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].B = static_cast<unsigned char>((normalsImage.bitmap[i].B + 1.0) * 127.5f);
    depthDisplayImage.bitmap[i] = static_cast<unsigned char>(((depthImage.bitmap[i] - minDepth) / (maxDepth - minDepth)) * 255.f);
  }
}


int main()
{
  vpPanda3DRenderParameters renderParams(vpCameraParameters(300, 300, 160, 120), 240, 320, 0.01, 10.0);
  vpPanda3DRendererSet renderer(renderParams);
  renderer.setRenderParameters(renderParams);

  load_prc_file_data("", "sync-video false");
  load_prc_file_data("", "assert-abort 1");
  std::shared_ptr<vpPanda3DGeometryRenderer> geometryRenderer = std::shared_ptr<vpPanda3DGeometryRenderer>(new vpPanda3DGeometryRenderer("geom"));

  renderer.addSubRenderer(geometryRenderer);
  std::cout << "Initializing framework" << std::endl;
  renderer.initFramework(false);

  std::cout << "Loading object" << std::endl;
  NodePath object = renderer.loadObject("cube", "/home/sfelton/software/visp-sfelton/tutorial/ar/data/simple_object.obj");
  std::cout << "Adding node to scene" <<std::endl;
  renderer.addNodeToScene(object);


  // PT(AmbientLight) alight = new AmbientLight("alight");
  // alight->set_color(LColor(0.2, 0.2, 0.2, 1.0));
  // NodePath alnp = base->getRenderRoot().attach_new_node(alight);
  // base->getRenderRoot().set_light(alnp);
  std::cout << "Setting camera pose" << std::endl;
  renderer.setCameraPose(vpHomogeneousMatrix(0.0, -5.0, 0.0, 0.0, 0.0, 0.0));

  vpImage<vpRGBf> normalsImage;
  vpImage<float> depthImage;
  vpImage<vpRGBa> normalDisplayImage(renderParams.getImageHeight(), renderParams.getImageWidth());
  vpImage<unsigned char> depthDisplayImage(renderParams.getImageHeight(), renderParams.getImageWidth());

  vpDisplayX dNormals(normalDisplayImage, 0, 0, "normals");
  vpDisplayX dDepth(depthDisplayImage, renderParams.getImageWidth(), 0, "depth");



  while (true) {
    const double beforeRender = vpTime::measureTimeMs();
    renderer.renderFrame();
    renderer.getRenderer<vpPanda3DGeometryRenderer>()->getRender(normalsImage, depthImage);

    convertToDisplay(normalsImage, depthImage, normalDisplayImage, depthDisplayImage, 4, 8.0);

    const double afterRender = vpTime::measureTimeMs();
    const double delta = (afterRender - beforeRender) / 1000.0;
    vpDisplay::display(normalDisplayImage);
    vpDisplay::flush(normalDisplayImage);

    vpDisplay::display(depthDisplayImage);
    vpDisplay::flush(depthDisplayImage);


    std::cout << "Before get pose" << std::endl;
    vpHomogeneousMatrix wTo = renderer.getNodePose("cube");
    std::cout << wTo << std::endl;
    vpHomogeneousMatrix oToo = vpExponentialMap::direct(vpColVector({ 0.0, 0.0, 0.0, 0.0, 0.0, vpMath::rad(20.0) }), delta);
    renderer.setNodePose("cube", wTo * oToo);
    std::cout << object.get_scale() << std::endl;
    std::cout << "After set pose " << std::endl;
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
