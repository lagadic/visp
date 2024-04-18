#ifndef vpPanda3DRendererSet_h
#define vpPanda3DRendererSet_h
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <vector>


#include <visp3/ar/vpPanda3DBaseRenderer.h>

class VISP_EXPORT vpPanda3DRendererSet : public vpPanda3DBaseRenderer
{
public:
  vpPanda3DRendererSet(const vpPanda3DRenderParameters &renderParameters) : vpPanda3DBaseRenderer("set")
  {
    m_renderParameters = renderParameters;
  }

  void initFramework(bool showWindow) vp_override
  {
    if (m_framework.use_count() > 0) {
      throw vpException(vpException::notImplementedError, "Panda3D renderer: Reinitializing is not supported!");
    }
    m_framework = std::shared_ptr<PandaFramework>(new PandaFramework());
    m_framework->open_framework();
    WindowProperties winProps;
    winProps.set_size(LVecBase2i(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight()));
    int flags = showWindow ? 0 : GraphicsPipe::BF_refuse_window;
    m_window = m_framework->open_window(winProps, flags);
    if (m_window == nullptr) {
      throw vpException(vpException::fatalError, "Could not open Panda3D window (hidden or visible)");
    }
    m_window->set_background_type(WindowFramework::BackgroundType::BT_black);
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      renderer->initFromParent(m_framework, m_window);
    }
  }

  void setupScene() vp_override { }

  void setupCamera() vp_override { }


  void setCameraPose(const vpHomogeneousMatrix &wTc) vp_override
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      renderer->setCameraPose(wTc);
    }
  }

  vpHomogeneousMatrix getCameraPose() vp_override
  {
    if (m_subRenderers.size() == 0) {
      throw vpException(vpException::fatalError, "cannot get the pose of an object if no subrenderer is present");
    }
    return m_subRenderers[0]->getCameraPose();
  }

  void setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo) vp_override
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      renderer->setNodePose(name, wTo);
    }
  }

  void setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo) vp_override
  {
    throw vpException(vpException::badValue, "NodePath setNodePose is not supported in renderer set, prefer the string version");
  }

  vpHomogeneousMatrix getNodePose(const std::string &name) vp_override
  {
    if (m_subRenderers.size() == 0) {
      throw vpException(vpException::fatalError, "cannot get the pose of an object if no subrenderer is present");
    }
    return m_subRenderers[0]->getNodePose(name);
  }
  vpHomogeneousMatrix getNodePose(NodePath &object) vp_override
  {
    throw vpException(vpException::badValue, "NodePath getNodePose is not supported in renderer set, prefer the string version");
  }


  void addNodeToScene(const NodePath &object) vp_override
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      renderer->addNodeToScene(object);
    }
  }


  void setRenderParameters(const vpPanda3DRenderParameters &params) vp_override
  {
    m_renderParameters = params;
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      renderer->setRenderParameters(m_renderParameters);
    }
  }

  void addSubRenderer(std::shared_ptr<vpPanda3DBaseRenderer> renderer)
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &otherRenderer: m_subRenderers) {
      if (renderer->getName() == otherRenderer->getName()) {
        throw vpException(vpException::badValue, "Cannot have two subrenderers with the same name");
      }
    }
    m_subRenderers.push_back(renderer);
    renderer->setRenderParameters(m_renderParameters);
    if (m_framework != nullptr) {
      renderer->initFromParent(m_framework, m_window);
      renderer->setCameraPose(getCameraPose());
    }
  }

  template<typename RendererType>
  std::shared_ptr<RendererType> getRenderer()
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      std::shared_ptr<RendererType> rendererCast = std::dynamic_pointer_cast<RendererType>(renderer);
      if (rendererCast != nullptr) {
        return rendererCast;
      }
    }
    return nullptr;
  }

  template<typename RendererType>
  std::shared_ptr<RendererType> getRenderer(const std::string &name)
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      if (renderer->getName() == name) {
        std::shared_ptr<RendererType> rendererCast = std::dynamic_pointer_cast<RendererType>(renderer);
        if (rendererCast != nullptr) {
          return rendererCast;
        }
      }
    }
    return nullptr;
  }

private:
  std::vector<std::shared_ptr<vpPanda3DBaseRenderer>> m_subRenderers;
};

#endif
#endif
