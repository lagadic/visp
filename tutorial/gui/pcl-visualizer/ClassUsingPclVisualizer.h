#ifndef TESTCLASSPCLVISUALIZER_H
#define TESTCLASSPCLVISUALIZER_H

//! \example example-class-using-pcl-visualizer.h
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL)

#include<visp3/core/vpColVector.h>
#include<visp3/gui/vpPclPointCloudVisualization.h>

class ClassUsingPclVisualizer
{
private:
  vpTranslationVector _t; /*!< The translation between the noise-free point cloud and the possibly noisy, translated + rotated one*/
  vpRotationMatrix _R; /*!< The rotation between the noise-free point cloud and the possibly noisy, translated + rotated one*/
  vpHomogeneousMatrix _rotHunrotated; /*!< The homogeneous matrix expressing the pose of the noise-free point cloud with regard to the possibly noisy, translated + rotated one.*/

  double _minX; /*!< The minimum value of the X coordinate, expressed in the noise-free frame.*/
  double _maxX; /*!< The maximum value of the X coordinate, expressed in the noise-free frame.*/
  unsigned int _n; /*!< Number of points along the X-axis.*/
  double _dX; // _dX = (_maxX - _minX)/(_n-1)
  double _minY; /*!< The minimum value of the Y coordinate, expressed in the noise-free frame.*/
  double _maxY; /*!< The maximum value of the Y coordinate, expressed in the noise-free frame.*/
  unsigned int _m;  /*!< Number of points along the Y-axis.*/
  double _dY; // _dY = (_maxY - _minY)/(_m-1)

  vpPclPointCloudVisualization _visualizer; /*!< The PCL-based visualizer.*/

  /**
   * @brief Generate a noise-free grid of point, and a possibly noisy one, which is translated and rotated with regarded to the noise-free one.
   * 
   * @param addNoise If true, noise will be added to the translated + rotated point cloud.
   * @param order The order of the polynomial surface that is generated.
   * @return std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> 
   */
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> generateControlPoints(const bool &addNoise, const unsigned int &order, vpColVector &confidenceWeights);
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
   * @brief Demonstration on how to use a \b vpPclPointCloudVisualization in blocking mode, i.e.
   * we expect an input from the user after call to \b vpPclPointCloudVisualization::display 
   * to go forward in the code.
   * @param addNoise If true, noise will be added to the translated + rotated surface. 
   * @param order  The order of the polynomial surface that is generated.
   */
  void blockingMode(const bool &addNoise, const unsigned int& order);

  /**
   * @brief Demonstration on how to use a \b vpPclPointCloudVisualization in threaded mode.
   * 
   * @param addNoise If true, noise will be added to the translated + rotated surface. 
   * @param order  The order of the polynomial surface that is generated.
   */
  void threadedMode(const bool &addNoise, const unsigned int& order);
};
#endif
#endif