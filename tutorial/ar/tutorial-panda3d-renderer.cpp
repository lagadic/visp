//! \example tutorial-panda3d-renderer.cpp
#include <iostream>
#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_PANDA3D) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_MODULE_IO)

#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGTK.h>

#include <visp3/io/vpParseArgv.h>

#include <visp3/ar/vpPanda3DRGBRenderer.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <visp3/ar/vpPanda3DRendererSet.h>

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
                  vpImage<unsigned char> &depthDisplayImage, float near, float far)
{
#pragma omp parallel for simd
  for (unsigned int i = 0; i < depthImage.getSize(); ++i) {
    float val = std::max(0.f, (depthImage.bitmap[i] - near) / (far - near));
    depthDisplayImage.bitmap[i] = static_cast<unsigned char>(val * 255.f);
  }
  vpDisplay::display(depthDisplayImage);
  vpDisplay::flush(depthDisplayImage);
}



int main(int argc, const char **argv)
{
  bool invertTexture = false;
  bool stepByStep = false;
  char *modelPathCstr = nullptr;
  vpParseArgv::vpArgvInfo argTable[] =
  {
    {"-invert", vpParseArgv::ARGV_CONSTANT_BOOL, 0, (char *)&invertTexture,
     "Whether to force Texture inversion. Use this if the model is upside down."},
    {"-model", vpParseArgv::ARGV_STRING, (char *) nullptr, (char *)&modelPathCstr,
     "Path to the model to load."},
    {"-step", vpParseArgv::ARGV_CONSTANT_BOOL, (char *) nullptr, (char *)&stepByStep,
     "Show frames step by step."},
    {"-h", vpParseArgv::ARGV_HELP, (char *) nullptr, (char *) nullptr,
     "Print the help."},
    {(char *) nullptr, vpParseArgv::ARGV_END, (char *) nullptr, (char *) nullptr, (char *) nullptr} };

  // Read the command line options
  if (vpParseArgv::parse(&argc, argv, argTable,
                         vpParseArgv::ARGV_NO_LEFTOVERS |
                         vpParseArgv::ARGV_NO_ABBREV |
                         vpParseArgv::ARGV_NO_DEFAULTS)) {
    return (false);
  }

  std::string modelPath;
  if (modelPathCstr) {
    modelPath = modelPathCstr;
  }
  else {
    modelPath = "data/deformed_sphere.bam";
  }
  vpPanda3DRenderParameters renderParams(vpCameraParameters(300, 300, 160, 120), 240, 320, 0.01, 1.0);
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
  if (invertTexture) {
    renderer.setForcedInvertTextures(true);
  }

  std::cout << "Initializing Panda3D rendering framework" << std::endl;
  renderer.initFramework(true);

  std::cout << "Loading object " << modelPath << std::endl;
  NodePath object = renderer.loadObject(objectName, modelPath);
  std::cout << "Adding node to scene" <<std::endl;

  renderer.addNodeToScene(object);

  // rgbRenderer->getRenderRoot().set_shader_auto(100);
  vpPanda3DAmbientLight alight("Ambient", vpRGBf(0.2));
  renderer.addLight(alight);
  vpPanda3DPointLight plight("Point", vpRGBf(2.0), vpColVector({ 0.0, -0.3, -0.0 }));
  renderer.addLight(plight);

  rgbRenderer->printStructure();
  std::cout << "Setting camera pose" << std::endl;
  renderer.setCameraPose(vpHomogeneousMatrix(0.0, 0.0, -0.5, 0.0, 0.0, 0.0));

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
  DisplayCls dNormalsCamera(cameraNormalDisplayImage, 0, renderParams.getImageHeight() + 80, "normals in camera space");
  DisplayCls dDepth(depthDisplayImage, renderParams.getImageWidth() + 80, 0, "depth");
  DisplayCls dColor(colorImage, renderParams.getImageWidth() * 3 + 90, 0, "color");
  renderer.renderFrame();
  bool end = false;
  bool firstFrame = true;
  std::vector<double> renderTime, fetchTime, displayTime;
  while (!end) {
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
    renderer.getRenderer<vpPanda3DGeometryRenderer>(cameraRenderer->getName())->getRender(cameraNormalsImage);
    renderer.getRenderer<vpPanda3DRGBRenderer>()->getRender(colorImage);

    const double beforeConvert = vpTime::measureTimeMs();

    displayNormals(normalsImage, normalDisplayImage);
    displayNormals(cameraNormalsImage, cameraNormalDisplayImage);
    displayDepth(depthImage, depthDisplayImage, near, far);
    vpDisplay::display(colorImage);
    vpDisplay::displayText(colorImage, 15, 15, "Click to quit", vpColor::red);

    if (stepByStep) {
      vpDisplay::displayText(colorImage, 50, 15, "Next frame: space", vpColor::red);
    }
    if (vpDisplay::getClick(colorImage, false)) {
      end = true;
    }
    vpDisplay::flush(colorImage);
    const double endDisplay = vpTime::measureTimeMs();
    renderTime.push_back(beforeFetch - beforeRender);
    fetchTime.push_back(beforeConvert - beforeFetch);
    displayTime.push_back(endDisplay - beforeConvert);
    std::string s;
    if (stepByStep) {
      bool next = false;
      while (!next) {
        vpDisplay::getKeyboardEvent(colorImage, s, true);
        if (s == " ") {
          next = true;
        }
      }
    }
    // if (firstFrame) {
    //   renderParams.setImageResolution(renderParams.getImageHeight() * 0.5, renderParams.getImageWidth() * 0.5);
    //   vpCameraParameters orig = renderParams.getCameraIntrinsics();
    //   vpCameraParameters newCam(orig.get_px() * 0.5, orig.get_py() * 0.5, orig.get_u0() * 0.5, orig.get_v0() * 0.5);
    //   renderParams.setCameraIntrinsics(newCam);
    //   std::cout << renderParams.getImageHeight() << std::endl;
    //   //dDepth.setDownScalingFactor(0.5);
    //   renderer.setRenderParameters(renderParams);
    // }
    // firstFrame = false;
    const double afterAll = vpTime::measureTimeMs();
    const double delta = (afterAll - beforeRender) / 1000.0;
    const vpHomogeneousMatrix wTo = renderer.getNodePose(objectName);
    const vpHomogeneousMatrix oToo = vpExponentialMap::direct(vpColVector({ 0.0, 0.0, 0.0, 0.0, vpMath::rad(20.0), 0.0 }), delta);
    renderer.setNodePose(objectName, wTo * oToo);
  }
  if (renderTime.size() > 0) {
    std::cout << "Render time: " << vpMath::getMean(renderTime) << "ms +- " << vpMath::getStdev(renderTime) << "ms" << std::endl;
    std::cout << "Panda3D -> vpImage time: " << vpMath::getMean(fetchTime) << "ms +- " << vpMath::getStdev(fetchTime) << "ms" << std::endl;
    std::cout << "Display time: " << vpMath::getMean(displayTime) << "ms +- " << vpMath::getStdev(displayTime) << "ms" << std::endl;
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
