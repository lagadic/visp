//! \example ClassUsingPclViewer.cpp
#include "ClassUsingPclViewer.h"

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO)
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
ClassUsingPclViewer::ClassUsingPclViewer(std::pair<double, double> xlimits, std::pair<double, double> ylimits, std::pair<unsigned int, unsigned int> nbPoints)
  : m_t(0.1, 0.1, 0.1)
  , m_R(M_PI_4, M_PI_4, M_PI_4)
  , m_cMo(m_t, m_R)
  , m_minX(xlimits.first)
  , m_maxX(xlimits.second)
  , m_n(nbPoints.first)
  , m_minY(ylimits.first)
  , m_maxY(ylimits.second)
  , m_m(nbPoints.second)
  , m_visualizer("Grid of points with / without robust")
{
  m_dX = (m_maxX - m_minX) / (static_cast<double>(m_n) - 1.);
  m_dY = (m_maxY - m_minY) / (static_cast<double>(m_m) - 1.);
}
//! [Constructor]

ClassUsingPclViewer::~ClassUsingPclViewer()
{

}

//! [Surface generator]
std::pair<vpPclViewer::pclPointCloudPointXYZRGBPtr, vpPclViewer::pclPointCloudPointXYZRGBPtr> ClassUsingPclViewer::generateControlPoints(const double &addedNoise, const unsigned int &order, vpColVector &confidenceWeights)
{
  std::pair<vpPclViewer::pclPointCloudPointXYZRGBPtr, vpPclViewer::pclPointCloudPointXYZRGBPtr> result;

  // Create control points
  vpPclViewer::pclPointCloudPointXYZRGBPtr unrotatedControlPoints(new vpPclViewer::pclPointCloudPointXYZRGB(m_n, m_m));
  vpPclViewer::pclPointCloudPointXYZRGBPtr   rotatedControlPoints(new vpPclViewer::pclPointCloudPointXYZRGB(m_n, m_m));

  // Initializing confindence weights
  confidenceWeights.resize(m_m * m_n);

  // Noise generator for the observed points
  // We deriberately choose to set the standard deviation to be twice the tolerated value to observe rejected points
  vpGaussRand r;
  r.setSigmaMean(addedNoise * 2., 0.);
  r.seed(vpTime::measureTimeMicros());

  // Residual vector to compute the confidence weights
  vpColVector residuals(m_m * m_n);

  for (unsigned int j = 0; j < m_m; j++) {
    for (unsigned int i = 0; i < m_n; i++) {
      // Creating model, expressed in the object frame
      double oX = m_minX + (double)i * m_dX;
      double oY = m_minY + (double)j * m_dY;
      double oZ = zFunction(oX, oY, order);

      // Setting the point coordinates of the first point cloud in
      // the object frame
      std::vector<double> point = { oX, oY, oZ,1. };
      vpColVector oCoords = vpColVector(point);
      (*unrotatedControlPoints)(i, j).x = oCoords[0];
      (*unrotatedControlPoints)(i, j).y = oCoords[1];
      (*unrotatedControlPoints)(i, j).z = oCoords[2];

      // Moving the point into another coordinate frame
      vpColVector cCoords = m_cMo * oCoords;
      (*rotatedControlPoints)(i, j).x = cCoords[0];
      (*rotatedControlPoints)(i, j).y = cCoords[1];

      // Potentially adding some noise if the user asked to
      double noise = r();
      (*rotatedControlPoints)(i, j).z = cCoords[2] + noise;

      // Filling the confidence weights with default value of 1.
      confidenceWeights[j * m_n + i] = 1.;

      // Indicating the residual, here it corresponds to the difference
      // between the theoretical position of the point and the actual one
      residuals[j * m_n + i] = noise;
    }
  }

  if (std::abs(addedNoise) > 0.) {
    // Estimating the confidence weights to remove points suffering too much noise
    // See vpRobust documentation for more information.
    vpRobust robust;
    robust.setMinMedianAbsoluteDeviation(addedNoise);
    robust.MEstimator(vpRobust::TUKEY, residuals, confidenceWeights);
  }

  result.first = unrotatedControlPoints;
  result.second = rotatedControlPoints;
  return result;
}
//! [Surface generator]

void ClassUsingPclViewer::blockingMode(const double &addedNoise, const unsigned int &order)
{
  // Confidence weights, that would be obtained thanks to vpRobust for instance
  vpColVector confWeights;

  //! [Generating point clouds]
  std::pair<vpPclViewer::pclPointCloudPointXYZRGBPtr, vpPclViewer::pclPointCloudPointXYZRGBPtr> grids = generateControlPoints(addedNoise, order, confWeights);
  //! [Generating point clouds]

  //! [Adding point clouds color not chosen]
  // Adding a point cloud for which we don't chose the color
  unsigned int id_ctrlPts = m_visualizer.addSurface(grids.first, "Standard");
  //! [Adding point clouds color not chosen]

  //! [Adding point clouds color chosen]
  // Adding a point cloud for which we chose the color
  vpColorBlindFriendlyPalette color(vpColorBlindFriendlyPalette::Palette::Purple);
  unsigned int id_robust = m_visualizer.addSurface(grids.second, confWeights, "RotatedWithRobust", color.to_RGB());
  //! [Adding point clouds color chosen]

  std::cout << "Press \"q\" while selecting the viewer window to stop the program." << std::endl;
  //! [Displaying point clouds blocking mode]
  m_visualizer.display();
  //! [Displaying point clouds blocking mode]

  (void)id_ctrlPts;
  (void)id_robust;
}

void ClassUsingPclViewer::threadedMode(const double &addedNoise, const unsigned int &order)
{
  // Confidence weights, that would be obtained thanks to vpRobust for instance
  vpColVector confWeights;

  // Create control points
  std::pair<vpPclViewer::pclPointCloudPointXYZRGBPtr, vpPclViewer::pclPointCloudPointXYZRGBPtr> grids = generateControlPoints(addedNoise, order, confWeights);

  // Adding a point cloud for which we don't chose the color
  unsigned int id_ctrlPts = m_visualizer.addSurface(grids.first, "Standard");

  // Adding a point cloud for which we chose the color
  vpColorBlindFriendlyPalette color(vpColorBlindFriendlyPalette::Palette::Purple);
  unsigned int id_robust = m_visualizer.addSurface(grids.second, confWeights, "RotatedWithRobust", color.to_RGB());

  //! [Starting display thread]
  m_visualizer.launchThread();
  //! [Starting display thread]

  m_visualizer.updateSurface(grids.first, id_ctrlPts);
  m_visualizer.updateSurface(grids.second, id_robust, confWeights);

  vpKeyboard keyboard;
  bool wantToStop = false;
  double t;

  std::cout << "Press any key in the console to stop the program." << std::endl;
  while (!wantToStop) {
    t = vpTime::measureTimeMs();
    grids = generateControlPoints(addedNoise, order, confWeights);

    //! [Updating point clouds used by display thread]
    m_visualizer.updateSurface(grids.first, id_ctrlPts);
    m_visualizer.updateSurface(grids.second, id_robust, confWeights);
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
