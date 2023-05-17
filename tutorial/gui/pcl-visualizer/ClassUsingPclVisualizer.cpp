//! \example ClassUsingPclVisualizer.cpp
#include "ClassUsingPclVisualizer.h"

#if defined(VISP_HAVE_PCL)
// PCL
#include <pcl/io/pcd_io.h>

// Visp
#include <visp3/core/vpTime.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpRobust.h>
#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/io/vpKeyboard.h>

//! [Z coordinates computation]
double zFunction(const double &x, const double &y, const unsigned int order)
{
  const double offset(0.5);
  double z(0.);

  for(unsigned int n = 0; n <= order; n++)
  {
    for(unsigned int k = 0; k <= order - n; k++)
    {
      if(k + n > 0)
      {
        z += std::pow(x, n) * std::pow(y, k);
      }
      else
      {
        z += offset;
      }

    }
  }

  return z;
}
//! [Z coordinates computation]

//! [Constructor]
ClassUsingPclVisualizer::ClassUsingPclVisualizer(std::pair<double, double> xlimits, std::pair<double, double> ylimits, std::pair<unsigned int, unsigned int> nbPoints)
  : _t(0.1,0.1,0.1)
  , _R(M_PI_4,M_PI_4,M_PI_4)
  , _cMo(_t,_R)
  , _minX(xlimits.first)
  , _maxX(xlimits.second)
  , _n(nbPoints.first)
  , _minY(ylimits.first)
  , _maxY(ylimits.second)
  , _m(nbPoints.second)
  , _visualizer("Grid of points with / without robust")
{
  _dX = (_maxX - _minX) / (static_cast<double>(_n) - 1.);
  _dY = (_maxY - _minY) / (static_cast<double>(_m) - 1.);
}
//! [Constructor]

ClassUsingPclVisualizer::~ClassUsingPclVisualizer()
{

}

//! [Surface generator]
std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> ClassUsingPclVisualizer::generateControlPoints(const double &addedNoise, const unsigned int &order, vpColVector &confidenceWeights)
{
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> result;

  // Create control points
  vpPclPointCloudVisualization::pclPointCloudPtr unrotatedControlPoints(new vpPclPointCloudVisualization::pclPointCloud(_n,_m));
  vpPclPointCloudVisualization::pclPointCloudPtr   rotatedControlPoints(new vpPclPointCloudVisualization::pclPointCloud(_n,_m));

  // Initializing confindence weights
  confidenceWeights.resize(_m * _n);

  // Noise generator for the observed points
  // We deriberately choose to set the standard deviation to be twice the tolerated value to observe rejected points
   vpGaussRand r;
   r.setSigmaMean(addedNoise * 2., 0.);
   r.seed(vpTime::measureTimeMicros());

  // Residual vector to compute the confidence weights
  vpColVector residuals(_m * _n);

  for(unsigned int j =  0; j< _m; j++){
    for(unsigned int i = 0; i<_n; i++){
      // Creating model, expressed in the object frame
      double oX = _minX + (double)i * _dX;
      double oY = _minY + (double)j * _dY;
      double oZ = zFunction(oX, oY, order);

      // Setting the point coordinates of the first point cloud in
      // the object frame
      std::vector<double> point={oX, oY, oZ,1.};
      vpColVector oCoords = vpColVector(point);
      (*unrotatedControlPoints)(i,j).x = oCoords[0];
      (*unrotatedControlPoints)(i,j).y = oCoords[1];
      (*unrotatedControlPoints)(i,j).z = oCoords[2];

      // Moving the point into another coordinate frame
      vpColVector cCoords = _cMo * oCoords;
      (*rotatedControlPoints)(i,j).x = cCoords[0];
      (*rotatedControlPoints)(i,j).y = cCoords[1];

      // Potentially adding some noise if the user asked to
      double noise = r();
      (*rotatedControlPoints)(i,j).z = cCoords[2] + noise; 

      // Filling the confidence weights with default value of 1.
      confidenceWeights[j * _n + i] = 1.;

      // Indicating the residual, here it corresponds to the difference 
      // between the theoretical position of the point and the actual one
      residuals[j * _n + i] = noise;
    }
  }

  if(std::abs(addedNoise) > 0.)
  {
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

void ClassUsingPclVisualizer::blockingMode(const double &addedNoise, const unsigned int& order)
{
  // Confidence weights, that would be obtained thanks to vpRobust for instance
  vpColVector confWeights;

  //! [Generating point clouds]
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> grids = generateControlPoints(addedNoise, order, confWeights);
  //! [Generating point clouds]

  //! [Adding point clouds color not chosen]
  // Adding a point cloud for which we don't chose the color 
  unsigned int id_ctrlPts = _visualizer.addSurface(grids.first, "Standard"); 
  //! [Adding point clouds color not chosen]

  //! [Adding point clouds color chosen]
  // Adding a point cloud for which we chose the color
  vpColorBlindFriendlyPalette color(vpColorBlindFriendlyPalette::Palette::Purple);
  unsigned int id_robust = _visualizer.addSurface(grids.second, confWeights, "RotatedWithRobust", color.to_RGB());
  //! [Adding point clouds color chosen]

  //! [Displaying point clouds blocking mode]
  _visualizer.display();
  //! [Displaying point clouds blocking mode]
}

void ClassUsingPclVisualizer::threadedMode(const double &addedNoise, const unsigned int& order)
{
  // Confidence weights, that would be obtained thanks to vpRobust for instance
  vpColVector confWeights;

  // Create control points
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> grids = generateControlPoints(addedNoise, order, confWeights);

  // Adding a point cloud for which we don't chose the color 
  unsigned int id_ctrlPts = _visualizer.addSurface(grids.first, "Standard"); 
  
  // Adding a point cloud for which we chose the color
  vpColorBlindFriendlyPalette color(vpColorBlindFriendlyPalette::Palette::Purple);
  unsigned int id_robust = _visualizer.addSurface(grids.second, confWeights, "RotatedWithRobust", color.to_RGB());

  //! [Starting display thread]
  _visualizer.launchThread();
  //! [Starting display thread]
  
  _visualizer.updateSurface(grids.first , id_ctrlPts);
  _visualizer.updateSurface(grids.second , id_robust, confWeights);

  vpKeyboard keyboard;
  bool wantToStop = false;
  double t;
  
  while(!wantToStop){
    t = vpTime::measureTimeMs();
    grids = generateControlPoints(addedNoise, order, confWeights);

    //! [Updating point clouds used by display thread]
    _visualizer.updateSurface(grids.first , id_ctrlPts);
    _visualizer.updateSurface(grids.second , id_robust, confWeights);
    //! [Updating point clouds used by display thread]

    if (keyboard.kbhit()) {
      wantToStop = true;
    }
    vpTime::wait(t, 40);
  }

}
#else
void dummy_class_using_pcl_visualizer()
{
}
#endif