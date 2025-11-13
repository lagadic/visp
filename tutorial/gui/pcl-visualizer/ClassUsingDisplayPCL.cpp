//! \example ClassUsingDisplayPCL.cpp
#include "ClassUsingDisplayPCL.h"

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_PCL_IO)
// PCL
#include <pcl/io/pcd_io.h>

// Visp
#include <visp3/core/vpTime.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpRobust.h>
#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/io/vpKeyboard.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

//! [Z coordinates computation]
double zFunction(const double &x, const double &y, const unsigned int order)
{
  const double offset(0.5);
  double z(0.);

  for (unsigned int n = 0; n <= order; n++) {
    for (unsigned int k = 0; k <= order - n; k++) {
      if (k + n > 0) {
        z += std::pow(x, n) * std::pow(y, k);
      }
      else {
        z += offset;
      }
    }
  }

  return z;
}
//! [Z coordinates computation]

//! [Constructor]
ClassUsingDisplayPCL::ClassUsingDisplayPCL(std::pair<double, double> xlimits, std::pair<double, double> ylimits, std::pair<unsigned int, unsigned int> nbPoints)
  : m_t(0.1, 0.1, 0.1)
  , m_R(M_PI_4, M_PI_4, M_PI_4)
  , m_cMo(m_t, m_R)
  , m_minX(xlimits.first)
  , m_maxX(xlimits.second)
  , m_n(nbPoints.first)
  , m_minY(ylimits.first)
  , m_maxY(ylimits.second)
  , m_m(nbPoints.second)
  , m_visualizer(0, 0, "Grid of points")
{
  m_dX = (m_maxX - m_minX) / (static_cast<double>(m_n) - 1.);
  m_dY = (m_maxY - m_minY) / (static_cast<double>(m_m) - 1.);
}
//! [Constructor]

ClassUsingDisplayPCL::~ClassUsingDisplayPCL()
{

}

//! [Surface generator]
void ClassUsingDisplayPCL::generateControlPoints(const double &addedNoise, const unsigned int &order, pcl::PointCloud<PointType>::Ptr &base, pcl::PointCloud<PointType>::Ptr &rotated)
{
  // Create control points
  bool initialize_base = (base ? false : true);
  if (initialize_base) {
    base = std::make_shared<pcl::PointCloud<PointType>>(m_n, m_m);
  }
  bool initialize_rotated = (rotated ? false : true);
  if (initialize_rotated) {
    rotated = std::make_shared<pcl::PointCloud<PointType>>(m_n, m_m);
  }

  // Noise generator for the observed points
  vpGaussRand r;
  r.setSigmaMean(addedNoise, 0.);
  r.seed(vpTime::measureTimeMicros());

  for (unsigned int j = 0; j < m_m; j++) {
    for (unsigned int i = 0; i < m_n; i++) {
      // Creating model, expressed in the object frame
      double oX = m_minX + static_cast<double>(i) * m_dX;
      double oY = m_minY + static_cast<double>(j) * m_dY;
      double oZ = zFunction(oX, oY, order);

      // Setting the point coordinates of the first point cloud in
      // the object frame
      std::vector<double> point = { oX, oY, oZ,1. };
      vpColVector oCoords = vpColVector(point);
      if (initialize_base) {
        (*base)(i, j).x = oCoords[0];
        (*base)(i, j).y = oCoords[1];
        (*base)(i, j).z = oCoords[2];
      }

      // Moving the point into another coordinate frame
      vpColVector cCoords = m_cMo * oCoords;
      (*rotated)(i, j).x = cCoords[0];
      (*rotated)(i, j).y = cCoords[1];

      // Potentially adding some noise if the user asked to
      double noise = r();
      (*rotated)(i, j).z = cCoords[2] + noise;
    }
  }
}
//! [Surface generator]

void ClassUsingDisplayPCL::threadedMode(const double &addedNoise, const unsigned int &order)
{
  // Create control points
  pcl::PointCloud<PointType>::Ptr base, rotated;
  generateControlPoints(addedNoise, order, base, rotated);

  //! [Inserting point clouds]
  // Adding a point cloud for which we don't chose the color
  std::mutex mutex_base;
  vpColorBlindFriendlyPalette color_base(vpColorBlindFriendlyPalette::Palette::Yellow);
  m_visualizer.addPointCloud(mutex_base, base, "Base", color_base.to_vpColor());

  // Adding a point cloud for which we chose the color
  std::mutex mutex_rotated;
  vpColorBlindFriendlyPalette color(vpColorBlindFriendlyPalette::Palette::Purple);
  m_visualizer.addPointCloud(mutex_rotated, rotated, "RotatedWithNoise", color.to_vpColor());
  //! [Inserting point clouds]

  //! [Starting display thread]
  m_visualizer.startThread(false);
  //! [Starting display thread]

  vpKeyboard keyboard;
  bool wantToStop = false;
  double t;

  std::cout << "Press any key in the console to stop the program." << std::endl;
  while (!wantToStop) {
    t = vpTime::measureTimeMs();

    //! [Updating point clouds used by display thread]
    {
      std::lock_guard lg_base(mutex_base);
      std::lock_guard lg_rotated(mutex_rotated);
      generateControlPoints(addedNoise, order, base, rotated);
    }
    //! [Updating point clouds used by display thread]

    if (keyboard.kbhit()) {
      wantToStop = true;
    }
    vpTime::wait(t, 40);
  }

}
#else
void dummy_class_using_pcl_visualizer()
{ }
#endif
