#include <visp3/ar/vpPanda3DFrameworkManager.h>
#include "load_prc_file.h"
void vpPanda3DFrameworkManager::initFramework()
{
  if (!m_frameworkIsOpen) {
    load_prc_file_data("", "textures-power-2 none");
    load_prc_file_data("", "gl-version 3 2");
    load_prc_file_data("", "no-singular-invert");
    m_framework.open_framework();
    m_frameworkIsOpen = true;
  }

}

void vpPanda3DFrameworkManager::exit()
{
  m_framework.close_framework();
}


void vpPanda3DFrameworkManager::registerDisabledWindow(PointerTo<WindowFramework> wf)
{

  m_disabledWindows.push_back(wf);
}

void vpPanda3DFrameworkManager::disableAllOtherRenderers(PointerTo<WindowFramework> &active)
{
  for (int i = 0; i < m_framework.get_num_windows(); ++i) {
    PointerTo<WindowFramework> fi = m_framework.get_window(i);
    if (fi != active) {
      fi->get_graphics_output()->get_gsg()->set_active(false);
    }
  }
}

void vpPanda3DFrameworkManager::enableAllRenderers()
{
  for (int i = 0; i < m_framework.get_num_windows(); ++i) {
    WindowFramework *fi = m_framework.get_window(i);
    if (std::find(m_disabledWindows.begin(), m_disabledWindows.end(), fi) == m_disabledWindows.end()) {
      fi->get_graphics_output()->get_gsg()->set_active(true);
    }
  }
}
