#include <iostream>
#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_PANDA3D) && defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGTK.h>



#include <visp3/ar/vpPanda3DRGBRenderer.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <visp3/ar/vpPanda3DRendererSet.h>

#include <ambientLight.h>
#include <directionalLight.h>


void displayNormals(const vpImage<vpRGBf> &normalsImage,
                    vpImage<vpRGBa> &normalDisplayImage)
{
#pragma omp parallel for simd
  for (unsigned int i = 0; i < normalsImage.getSize(); ++i) {
    normalDisplayImage.bitmap[i].R = static_cast<unsigned char>((normalsImage.bitmap[i].R + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].G = static_cast<unsigned char>((normalsImage.bitmap[i].G + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].B = static_cast<unsigned char>((normalsImage.bitmap[i].B + 1.0) * 127.5f);
  }
  vpDisplay::display(normalDisplayImage);
  vpDisplay::flush(normalDisplayImage);
}
void displayDepth(const vpImage<float> &depthImage,
                  vpImage<unsigned char> &depthDisplayImage)
{
#pragma omp parallel for simd
  for (unsigned int i = 0; i < depthImage.getSize(); ++i) {
    depthDisplayImage.bitmap[i] = static_cast<unsigned char>(depthImage.bitmap[i] * 255.f);
  }
  vpDisplay::display(depthDisplayImage);
  vpDisplay::flush(depthDisplayImage);
}

int main()
{
  vpPanda3DRenderParameters renderParams(vpCameraParameters(300, 300, 160, 120), 480, 640, 0.01, 1.0);
  vpPanda3DRendererSet renderer(renderParams);
  renderer.setRenderParameters(renderParams);
  const std::string objectName = "object";



  std::shared_ptr<vpPanda3DGeometryRenderer> geometryRenderer = std::shared_ptr<vpPanda3DGeometryRenderer>(new vpPanda3DGeometryRenderer(vpPanda3DGeometryRenderer::vpRenderType::WORLD_NORMALS));

  std::shared_ptr<vpPanda3DGeometryRenderer> cameraRenderer = std::shared_ptr<vpPanda3DGeometryRenderer>(new vpPanda3DGeometryRenderer(vpPanda3DGeometryRenderer::vpRenderType::CAMERA_NORMALS));
  std::shared_ptr<vpPanda3DRGBRenderer> rgbRenderer = std::shared_ptr<vpPanda3DRGBRenderer>(new vpPanda3DRGBRenderer());

  renderer.addSubRenderer(geometryRenderer);
  renderer.addSubRenderer(cameraRenderer);
  renderer.addSubRenderer(rgbRenderer);

  renderer.setVerticalSyncEnabled(false);
  renderer.setAbortOnPandaError(true);

  std::cout << "Initializing framework" << std::endl;
  renderer.initFramework(false);

  std::cout << "Loading object" << std::endl;
  NodePath object = renderer.loadObject(objectName, "/home/sfelton/cube.bam");
  std::cout << "Adding node to scene" <<std::endl;

  PT(Material) mat = new Material();
  mat->set_specular(LColor(1.0, 1.0, 1.0, 1.0));
  mat->set_base_color(LColor(1.0));
  mat->set_shininess(5.0);
  mat->set_metallic(5.0);
  object.set_material(mat);


  renderer.addNodeToScene(object);


  // PT(DirectionalLight) d_light;
  // d_light = new DirectionalLight("my d_light");
  // d_light->set_color(LColor(10.0, 1.0, 1.0, 1));
  // d_light->set_direction(LVector3(0.5, 0.5, 0.0).normalized());
  // NodePath dlnp = rgbRenderer->getRenderRoot().attach_new_node(d_light);
  // d_light->set_point(LPoint3(-5, -5, 0));
  // // dlnp.set_hpr(-30, -60, 0);
  // rgbRenderer->getRenderRoot().set_light(dlnp);
  PT(AmbientLight) a_light = new AmbientLight("my a_light");
  a_light->set_color(LColor(1.0, 5.0, 0.0, 1.0));
  NodePath alnp = rgbRenderer->getRenderRoot().attach_new_node(a_light);
  rgbRenderer->getRenderRoot().set_light(alnp);

  std::cout << "Setting camera pose" << std::endl;
  renderer.setCameraPose(vpHomogeneousMatrix(0.0, -0.4, 0.0, 0.0, 0.0, 0.0));

  vpImage<vpRGBf> normalsImage;
  vpImage<vpRGBf> cameraNormalsImage;

  vpImage<float> depthImage;
  vpImage<vpRGBa> colorImage(renderParams.getImageHeight(), renderParams.getImageWidth());
  vpImage<vpRGBa> normalDisplayImage(renderParams.getImageHeight(), renderParams.getImageWidth());
  vpImage<vpRGBa> cameraNormalDisplayImage(renderParams.getImageHeight(), renderParams.getImageWidth());

  vpImage<unsigned char> depthDisplayImage(renderParams.getImageHeight(), renderParams.getImageWidth());

#if defined(VISP_HAVE_GTK)
  using DisplayCls = vpDisplayGTK;
#elif defined(VISP_HAVE_X11)
  using DisplayCls = vpDisplayX;
#elif defined(HAVE_OPENCV_HIGHGUI)
  using DisplayCls = vpDisplayOpenCV;
#elif defined(VISP_HAVE_GDI)
  using DisplayCls = vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  using DisplayCls = vpDisplayD3D;
#endif

  DisplayCls dNormals(normalDisplayImage, 0, 0, "normals in world space");
  DisplayCls dNormalsCamera(cameraNormalDisplayImage, 0, 0, "normals in camera space");
  DisplayCls dDepth(depthDisplayImage, renderParams.getImageWidth(), 0, "depth");
  DisplayCls dColor(colorImage, renderParams.getImageWidth() * 2, 0, "color");

  while (true) {
    float near = 0, far = 0;
    const double beforeComputeBB = vpTime::measureTimeMs();
    rgbRenderer->computeNearAndFarPlanesFromNode(objectName, near, far);
    renderParams.setClippingDistance(near, far);
    renderer.setRenderParameters(renderParams);
    std::cout << "Update clipping plane took " << vpTime::measureTimeMs() - beforeComputeBB << std::endl;
    const double beforeRender = vpTime::measureTimeMs();
    renderer.renderFrame();
    const double beforeFetch = vpTime::measureTimeMs();
    renderer.getRenderer<vpPanda3DGeometryRenderer>(geometryRenderer->getName())->getRender(normalsImage, depthImage);
    renderer.getRenderer<vpPanda3DGeometryRenderer>(cameraRenderer->getName())->getRender(cameraNormalsImage, depthImage);
    renderer.getRenderer<vpPanda3DRGBRenderer>()->getRender(colorImage);

    const double beforeConvert = vpTime::measureTimeMs();

    displayNormals(normalsImage, normalDisplayImage);
    displayNormals(cameraNormalsImage, cameraNormalDisplayImage);
    displayDepth(depthImage, depthDisplayImage);
    vpDisplay::display(colorImage);
    vpDisplay::flush(colorImage);

    const double afterAll = vpTime::measureTimeMs();
    const double delta = (afterAll - beforeRender) / 1000.0;
    vpHomogeneousMatrix wTo = renderer.getNodePose(objectName);
    vpHomogeneousMatrix oToo = vpExponentialMap::direct(vpColVector({ 0.0, 0.0, 0.0, 0.0, 0.0, vpMath::rad(20.0) }), delta);
    renderer.setNodePose(objectName, wTo * oToo);
    std::cout << "Rendering took: " << std::fixed << std::setprecision(2) << beforeFetch - beforeRender << "ms" << std::endl;
    std::cout << "Copying to vpImage took: " << std::fixed << std::setprecision(2) << beforeConvert - beforeFetch << "ms" << std::endl;
    std::cout << "display took: " << std::fixed << std::setprecision(2) << afterAll - beforeConvert << "ms" << std::endl;

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
