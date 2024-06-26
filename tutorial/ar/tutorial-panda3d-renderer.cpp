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
#include <visp3/io/vpImageIo.h>

#include <visp3/ar/vpPanda3DRGBRenderer.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <visp3/ar/vpPanda3DRendererSet.h>
#include <visp3/ar/vpPanda3DCommonFilters.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void displayNormals(const vpImage<vpRGBf> &normalsImage,
                    vpImage<vpRGBa> &normalDisplayImage)
{
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < normalsImage.getSize(); ++i) {
    normalDisplayImage.bitmap[i].R = static_cast<unsigned char>((normalsImage.bitmap[i].R + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].G = static_cast<unsigned char>((normalsImage.bitmap[i].G + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].B = static_cast<unsigned char>((normalsImage.bitmap[i].B + 1.0) * 127.5f);
  }

  vpDisplay::display(normalDisplayImage);
  vpDisplay::flush(normalDisplayImage);
}

void displayDepth(const vpImage<float> &depthImage,
                  vpImage<unsigned char> &depthDisplayImage, float nearV, float farV)
{
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < depthImage.getSize(); ++i) {
    float val = std::max(0.f, (depthImage.bitmap[i] - nearV) / (farV - nearV));
    depthDisplayImage.bitmap[i] = static_cast<unsigned char>(val * 255.f);
  }
  vpDisplay::display(depthDisplayImage);
  vpDisplay::flush(depthDisplayImage);
}

void displayLightDifference(const vpImage<vpRGBa> &colorImage, const vpImage<vpRGBa> &colorDiffuseOnly, vpImage<unsigned char> &lightDifference)
{
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < colorImage.getSize(); ++i) {
    float I1 = 0.299 * colorImage.bitmap[i].R + 0.587 * colorImage.bitmap[i].G + 0.114 * colorImage.bitmap[i].B;
    float I2 = 0.299 * colorDiffuseOnly.bitmap[i].R + 0.587 * colorDiffuseOnly.bitmap[i].G + 0.114 * colorDiffuseOnly.bitmap[i].B;
    lightDifference.bitmap[i] = static_cast<unsigned char>(round(abs(I1 - I2)));
  }
  vpDisplay::display(lightDifference);
  vpDisplay::flush(lightDifference);
}

void displayCanny(const vpImage<vpRGBf> &cannyRawData,
                  vpImage<unsigned char> &canny)
{
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < cannyRawData.getSize(); ++i) {
    vpRGBf &px = cannyRawData.bitmap[i];
    canny.bitmap[i] = 255 * (px.R * px.R + px.G * px.G > 0);
    //canny.bitmap[i] = static_cast<unsigned char>(127.5f + 127.5f * atan(px.B));
  }

  vpDisplay::display(canny);
  for (unsigned int i = 0; i < canny.getHeight(); i += 8) {
    for (unsigned int j = 0; j < canny.getWidth(); j += 8) {
      bool valid = (pow(cannyRawData[i][j].R, 2.f) + pow(cannyRawData[i][j].G, 2.f)) > 0;
      if (!valid) continue;
      float angle = cannyRawData[i][j].B;
      unsigned x = j + 10 * cos(angle);
      unsigned y = i + 10 * sin(angle);
      vpDisplay::displayArrow(canny, i, j, y, x, vpColor::green);
    }
  }
  vpDisplay::flush(canny);
}

int main(int argc, const char **argv)
{
  bool stepByStep = false;
  bool debug = false;
  bool showLightContrib = false;
  bool showCanny = false;
  char *modelPathCstr = nullptr;
  char *backgroundPathCstr = nullptr;

  vpParseArgv::vpArgvInfo argTable[] =
  {
    {"-model", vpParseArgv::ARGV_STRING, (char *) nullptr, (char *)&modelPathCstr,
     "Path to the model to load."},
    {"-background", vpParseArgv::ARGV_STRING, (char *) nullptr, (char *)&backgroundPathCstr,
     "Path to the background image to load for the rgb renderer."},
    {"-step", vpParseArgv::ARGV_CONSTANT_BOOL, (char *) nullptr, (char *)&stepByStep,
     "Show frames step by step."},
    {"-specular", vpParseArgv::ARGV_CONSTANT_BOOL, (char *) nullptr, (char *)&showLightContrib,
     "Show frames step by step."},
    {"-canny", vpParseArgv::ARGV_CONSTANT_BOOL, (char *) nullptr, (char *)&showCanny,
     "Show frames step by step."},
    {"-debug", vpParseArgv::ARGV_CONSTANT_BOOL, (char *) nullptr, (char *)&debug,
     "Show Opengl/Panda3D debug message."},
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
    modelPath = "data/suzanne.bam";
  }
  std::string backgroundPath;
  if (backgroundPathCstr) {
    backgroundPath = backgroundPathCstr;
  }
  const std::string objectName = "object";

  //! [Renderer set]
  vpPanda3DRenderParameters renderParams(vpCameraParameters(300, 300, 160, 120), 240, 320, 0.01, 10.0);
  vpPanda3DRendererSet renderer(renderParams);
  renderer.setRenderParameters(renderParams);
  renderer.setVerticalSyncEnabled(false);
  renderer.setAbortOnPandaError(true);
  if (debug) {
    renderer.enableDebugLog();
  }
  //! [Renderer set]

  //! [Subrenderers init]
  std::shared_ptr<vpPanda3DGeometryRenderer> geometryRenderer = std::make_shared<vpPanda3DGeometryRenderer>(vpPanda3DGeometryRenderer::vpRenderType::OBJECT_NORMALS);
  std::shared_ptr<vpPanda3DGeometryRenderer> cameraRenderer = std::make_shared<vpPanda3DGeometryRenderer>(vpPanda3DGeometryRenderer::vpRenderType::CAMERA_NORMALS);
  std::shared_ptr<vpPanda3DRGBRenderer> rgbRenderer = std::make_shared<vpPanda3DRGBRenderer>();
  std::shared_ptr<vpPanda3DRGBRenderer> rgbDiffuseRenderer = std::make_shared<vpPanda3DRGBRenderer>(false);
  std::shared_ptr<vpPanda3DLuminanceFilter> grayscaleFilter = std::make_shared<vpPanda3DLuminanceFilter>("toGrayscale", rgbRenderer, false);
  std::shared_ptr<vpPanda3DGaussianBlur> blurFilter = std::make_shared<vpPanda3DGaussianBlur>("blur", grayscaleFilter, false);
  std::shared_ptr<vpPanda3DCanny> cannyFilter = std::make_shared<vpPanda3DCanny>("canny", blurFilter, true, 10.f);
  //! [Subrenderers init]

  //! [Adding subrenderers]
  renderer.addSubRenderer(geometryRenderer);
  renderer.addSubRenderer(cameraRenderer);
  renderer.addSubRenderer(rgbRenderer);
  if (showLightContrib) {
    renderer.addSubRenderer(rgbDiffuseRenderer);
  }
  if (showCanny) {
    renderer.addSubRenderer(grayscaleFilter);
    renderer.addSubRenderer(blurFilter);
    renderer.addSubRenderer(cannyFilter);
  }
  std::cout << "Initializing Panda3D rendering framework" << std::endl;
  renderer.initFramework();
  //! [Adding subrenderers]

  //! [Scene configuration]
  NodePath object = renderer.loadObject(objectName, modelPath);
  renderer.addNodeToScene(object);

  vpPanda3DAmbientLight alight("Ambient", vpRGBf(0.2f));
  renderer.addLight(alight);

  vpPanda3DPointLight plight("Point", vpRGBf(1.0f), vpColVector({ 0.3, -0.4, -0.2 }), vpColVector({ 0.0, 0.0, 1.0 }));
  renderer.addLight(plight);

  vpPanda3DDirectionalLight dlight("Directional", vpRGBf(2.0f), vpColVector({ 1.0, 1.0, 0.0 }));
  renderer.addLight(dlight);

  if (!backgroundPath.empty()) {
    vpImage<vpRGBa> background;
    vpImageIo::read(background, backgroundPath);
    rgbRenderer->setBackgroundImage(background);
  }

  rgbRenderer->printStructure();

  std::cout << "Setting camera pose" << std::endl;
  renderer.setCameraPose(vpHomogeneousMatrix(0.0, 0.0, -0.3, 0.0, 0.0, 0.0));
  //! [Scene configuration]

  unsigned h = renderParams.getImageHeight(), w = renderParams.getImageWidth();
  std::cout << "Creating display and data images" << std::endl;
  vpImage<vpRGBf> normalsImage;
  vpImage<vpRGBf> cameraNormalsImage;
  vpImage<vpRGBf> cannyRawData;
  vpImage<float> depthImage;
  vpImage<vpRGBa> colorImage(h, w);
  vpImage<vpRGBa> colorDiffuseOnly(h, w);
  vpImage<unsigned char> lightDifference(h, w);
  vpImage<unsigned char> cannyImage(h, w);

  vpImage<vpRGBa> normalDisplayImage(h, w);
  vpImage<vpRGBa> cameraNormalDisplayImage(h, w);
  vpImage<unsigned char> depthDisplayImage(h, w);

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
  DisplayCls dNormalsCamera(cameraNormalDisplayImage, 0, h + 80, "normals in camera space");
  DisplayCls dDepth(depthDisplayImage, w + 80, 0, "depth");
  DisplayCls dColor(colorImage, w + 80, h + 80, "color");

  DisplayCls dImageDiff;
  if (showLightContrib) {
    dImageDiff.init(lightDifference, w * 2 + 80, 0, "Specular/reflectance contribution");
  }
  DisplayCls dCanny;
  if (showCanny) {
    dCanny.init(cannyImage, w * 2 + 80, h + 80, "Canny");
  }
  renderer.renderFrame();
  bool end = false;
  bool firstFrame = true;
  std::vector<double> renderTime, fetchTime, displayTime;
  while (!end) {
    float nearV = 0, farV = 0;
    const double beforeComputeBB = vpTime::measureTimeMs();
    rgbRenderer->computeNearAndFarPlanesFromNode(objectName, nearV, farV);
    renderParams.setClippingDistance(nearV, farV);
    renderer.setRenderParameters(renderParams);
    //std::cout << "Update clipping plane took " << vpTime::measureTimeMs() - beforeComputeBB << std::endl;

    const double beforeRender = vpTime::measureTimeMs();
    renderer.renderFrame();
    const double beforeFetch = vpTime::measureTimeMs();
    renderer.getRenderer<vpPanda3DGeometryRenderer>(geometryRenderer->getName())->getRender(normalsImage, depthImage);
    renderer.getRenderer<vpPanda3DGeometryRenderer>(cameraRenderer->getName())->getRender(cameraNormalsImage);
    renderer.getRenderer<vpPanda3DRGBRenderer>(rgbRenderer->getName())->getRender(colorImage);
    if (showLightContrib) {
      renderer.getRenderer<vpPanda3DRGBRenderer>(rgbDiffuseRenderer->getName())->getRender(colorDiffuseOnly);
    }
    if (showCanny) {
      renderer.getRenderer<vpPanda3DCanny>()->getRender(cannyRawData);
    }

    const double beforeConvert = vpTime::measureTimeMs();
    displayNormals(normalsImage, normalDisplayImage);
    displayNormals(cameraNormalsImage, cameraNormalDisplayImage);
    displayDepth(depthImage, depthDisplayImage, nearV, farV);
    if (showLightContrib) {
      displayLightDifference(colorImage, colorDiffuseOnly, lightDifference);
    }
    if (showCanny) {
      displayCanny(cannyRawData, cannyImage);
    }

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
