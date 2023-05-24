#ifndef TESTCLASSPCLVISUALIZER_H
#define TESTCLASSPCLVISUALIZER_H

//! \example ClassUsingPclVisualizer.h
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL)

#include<visp3/core/vpColVector.h>
#include<visp3/gui/vpPclVisualizer.h>

class ClassUsingPclVisualizer
{
private:
  vpTranslationVector m_t; /*!< The translation between the noise-free point cloud and the possibly noisy, translated + rotated one*/
  vpRotationMatrix m_R; /*!< The rotation between the noise-free point cloud and the possibly noisy, translated + rotated one*/
  vpHomogeneousMatrix m_cMo; /*!< The homogeneous matrix expressing the pose of the noise-free point cloud with regard to the possibly noisy, translated + rotated one.*/

  double m_minX; /*!< The minimum value of the X coordinate, expressed in the noise-free frame.*/
  double m_maxX; /*!< The maximum value of the X coordinate, expressed in the noise-free frame.*/
  unsigned int m_n; /*!< Number of points along the X-axis.*/
  double m_dX; // m_dX = (m_maxX - m_minX)/(m_n-1)
  double m_minY; /*!< The minimum value of the Y coordinate, expressed in the noise-free frame.*/
  double m_maxY; /*!< The maximum value of the Y coordinate, expressed in the noise-free frame.*/
  unsigned int m_m;  /*!< Number of points along the Y-axis.*/
  double m_dY; // m_dY = (m_maxY - m_minY)/(m_m-1)

  vpPclVisualizer m_visualizer; /*!< The PCL-based visualizer.*/

  /**
   * @brief Generate a noise-free grid of point, and a possibly noisy one, which is translated and rotated with regarded to the noise-free one.
   * 
   * @param addedNoise Standard deviation of the noise.
   * @param order The order of the polynomial surface that is generated.
   * @return std::pair<vpPclVisualizer::pclPointCloudPtr, vpPclVisualizer::pclPointCloudPtr> 
   */
  std::pair<vpPclVisualizer::pclPointCloudPtr, vpPclVisualizer::pclPointCloudPtr> generateControlPoints(const double &addedNoise, const unsigned int &order, vpColVector &confidenceWeights);
public:
  /**
   * @brief Construct a new object.
   * 
   * @param xlimits A pair defining the <min, max> values of X-coordinates of the generated surface.
   * @param ylimits A pair defining the <min, max> values of Y-coordinates of the generated surface.
   * @param nbPoints The number of points along the <X-axis, Y-axis> that will be generated.
   */
  ClassUsingPclVisualizer(std::pair<double, double> xlimits = {-2.5,2.5}, std::pair<double, double> ylimits = {-2.5,2.5}, std::pair<unsigned int, unsigned int> nbPoints = {50,50});

  ~ClassUsingPclVisualizer();

  /**
   * @brief Demonstration on how to use a \b vpPclVisualizer in blocking mode, i.e.
   * we expect an input from the user after call to \b vpPclVisualizer::display 
   * to go forward in the code.
   * @param addedNoise Standard deviation of the noise added to the moved surface. 
   * @param order  The order of the polynomial surface that is generated.
   */
  void blockingMode(const double &addedNoise, const unsigned int& order);

  /**
   * @brief Demonstration on how to use a \b vpPclVisualizer in threaded mode.
   * 
   * @param addedNoise Standard deviation of the noise added to the moved surface. 
   * @param order  The order of the polynomial surface that is generated.
   */
  void threadedMode(const double &addedNoise, const unsigned int& order);
};
#endif
#endif