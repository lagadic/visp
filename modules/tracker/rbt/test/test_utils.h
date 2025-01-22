#ifndef VP_RB_TEST_UTILS_H
#define VP_RB_TEST_UTILS_H

#include <visp3/ar/vpPanda3DRendererSet.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <visp3/ar/vpPanda3DRGBRenderer.h>

#include <visp3/gui/vpDisplayX.h>

#include <vector>

struct TrajectoryData
{
  std::vector<vpImage<vpRGBa>> rgb;
  std::vector<vpImage<float>> depth;
  std::vector<vpHomogeneousMatrix> cTo;
};

TrajectoryData generateTrajectory(const vpPanda3DRenderParameters &renderingParams,
                                  const std::function<void(vpPanda3DRendererSet &)> &makeScene,
                                  std::vector<vpHomogeneousMatrix> &cTw, std::vector<vpHomogeneousMatrix> &oTw)
{
  vpPanda3DRendererSet renderer;
  renderer.setRenderParameters(renderingParams);
  auto depthRenderer = std::make_shared<vpPanda3DGeometryRenderer>(vpPanda3DGeometryRenderer::OBJECT_NORMALS);
  auto rgbRenderer = std::make_shared<vpPanda3DRGBRenderer>(true);
  renderer.addSubRenderer(rgbRenderer);
  renderer.addSubRenderer(depthRenderer);
  renderer.initFramework();
  makeScene(renderer);

  if (cTw.size() != oTw.size()) {
    throw vpException(vpException::dimensionError, "Number of poses don't match");
  }
  TrajectoryData res;
  res.rgb.resize(cTw.size());
  res.depth.resize(cTw.size());
  res.cTo.resize(cTw.size());

  for (unsigned int i = 0; i < cTw.size(); ++i) {
    res.rgb[i].resize(renderingParams.getImageHeight(), renderingParams.getImageWidth());
    res.depth[i].resize(renderingParams.getImageHeight(), renderingParams.getImageWidth());
    renderer.setNodePose("object", oTw[i].inverse());
    renderer.setCameraPose(cTw[i].inverse());

    float nearV = 0.01, farV = 1.0;
    depthRenderer->computeNearAndFarPlanesFromNode("object", nearV, farV, true);
    vpPanda3DRenderParameters renderingParamsFrame = renderingParams;
    renderingParamsFrame.setClippingDistance(nearV, farV);
    renderer.setRenderParameters(renderingParamsFrame);
    std::cout << "Rendering trajectory frame " << i << std::endl;
    renderer.renderFrame();
    renderer.getRenderer<vpPanda3DRGBRenderer>()->getRender(res.rgb[i]);
    renderer.getRenderer<vpPanda3DGeometryRenderer>()->getRender(res.depth[i]);
    res.cTo[i] = cTw[i] * oTw[i].inverse();
  }

  return res;
}

#endif
