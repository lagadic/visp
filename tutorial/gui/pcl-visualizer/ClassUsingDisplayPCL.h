#ifndef _ClassUsingPclVisualizer_h_
#define _ClassUsingPclVisualizer_h_

//! \example ClassUsingDisplayPCL.h
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_PCL_IO)

#include<visp3/core/vpColVector.h>
#include<visp3/gui/vpDisplayPCL.h>

class ClassUsingDisplayPCL
{
public:
  using PointType = pcl::PointXYZ;
private:
  VISP_NAMESPACE_ADDRESSING vpTranslationVector m_t; /*!< The translation between the noise-free point cloud and the possibly noisy, translated + rotated one*/
  VISP_NAMESPACE_ADDRESSING vpRotationMatrix m_R; /*!< The rotation between the noise-free point cloud and the possibly noisy, translated + rotated one*/
  VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix m_cMo; /*!< The homogeneous matrix expressing the pose of the noise-free point cloud with regard to the possibly noisy, translated + rotated one.*/

  double m_minX; /*!< The minimum value of the X coordinate, expressed in the noise-free frame.*/
  double m_maxX; /*!< The maximum value of the X coordinate, expressed in the noise-free frame.*/
  unsigned int m_n; /*!< Number of points along the X-axis.*/
  double m_dX; // m_dX = (m_maxX - m_minX)/(m_n-1)
  double m_minY; /*!< The minimum value of the Y coordinate, expressed in the noise-free frame.*/
  double m_maxY; /*!< The maximum value of the Y coordinate, expressed in the noise-free frame.*/
  unsigned int m_m;  /*!< Number of points along the Y-axis.*/
  double m_dY; // m_dY = (m_maxY - m_minY)/(m_m-1)

  VISP_NAMESPACE_ADDRESSING vpDisplayPCL m_visualizer; /*!< The PCL-based visualizer.*/

  /**
   * @brief Generate a noise-free grid of point, and a possibly noisy one, which is translated and rotated with regarded to the noise-free one.
   *
   * @param addedNoise Standard deviation of the noise.
   * @param order The order of the polynomial surface that is generated.
   * @param base The original point cloud, without rotation
   * @param rotated The rotated point cloud, with noise
   */
  void generateControlPoints(const double &addedNoise, const unsigned int &order, pcl::PointCloud<PointType>::Ptr &base, pcl::PointCloud<PointType>::Ptr &rotated);
public:
  /**
   * @brief Construct a new object.
   *
   * @param xlimits A pair defining the <min, max> values of X-coordinates of the generated surface.
   * @param ylimits A pair defining the <min, max> values of Y-coordinates of the generated surface.
   * @param nbPoints The number of points along the <X-axis, Y-axis> that will be generated.
   */
  ClassUsingDisplayPCL(std::pair<double, double> xlimits = { -2.5,2.5 }, std::pair<double, double> ylimits = { -2.5,2.5 }, std::pair<unsigned int, unsigned int> nbPoints = { 50,50 });

  ~ClassUsingDisplayPCL();

  /**
   * @brief Demonstration on how to use a \b vpDisplayPCL in threaded mode.
   *
   * @param addedNoise Standard deviation of the noise added to the moved surface.
   * @param order  The order of the polynomial surface that is generated.
   * @param useMonothread If true, use the monothread version of the viewer.
   */
  void runDemo(const double &addedNoise, const unsigned int &order, const bool &useMonothread);
};
#endif
#endif
