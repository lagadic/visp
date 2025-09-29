#ifndef VP_RB_INITIALIZATION_HELPER_H
#define VP_RB_INITIALIZATION_HELPER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>

#include <string>
#include <iostream>


BEGIN_VISP_NAMESPACE
template <typename T>
class vpImage;

class vpRBTracker;
/**
 * \brief A set of utilities to perform initialization.
 *
 * \ingroup group_rbt_core
*/
class VISP_EXPORT vpRBInitializationHelper
{
public:
  void savePose(const std::string &filename) const;

  vpHomogeneousMatrix getPose() const { return m_cMo; }

  void setCameraParameters(const vpCameraParameters &cam) { m_cam = cam; }

  friend vpRBTracker;

protected:
  void removeCommentsAndEmptyLines(std::ifstream &fileId);

private:
#ifdef VISP_HAVE_MODULE_GUI
  template <typename T>
  void initClick(const vpImage<T> &I, const std::string &initFile, bool displayHelp, vpRBTracker &tracker);
#endif

  std::string m_poseSavingFileName;

  vpHomogeneousMatrix m_cMo;
  vpCameraParameters m_cam;
};

END_VISP_NAMESPACE

#endif
