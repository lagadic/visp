//! \example example-class-using-pcl-visualizer.cpp
#include "ClassUsingPclVisualizer.h"

#if defined(VISP_HAVE_PCL)
// PCL
#include <pcl/io/pcd_io.h>

// Visp
#include <visp3/core/vpTime.h>
#include <visp3/core/vpUniRand.h>
#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/io/vpKeyboard.h>

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

ClassUsingPclVisualizer::ClassUsingPclVisualizer(std::pair<double, double> xlimits, std::pair<double, double> ylimits, std::pair<unsigned int, unsigned int> nbPoints)
  : _t(0.,0.,0.)
  , _R(M_PI_4,M_PI_4,M_PI_4)
  , _rotHunrotated(_t,_R)
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

ClassUsingPclVisualizer::~ClassUsingPclVisualizer()
{

}

std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> ClassUsingPclVisualizer::generateControlPoints(const bool &addNoise, const unsigned int &order, vpColVector &confidenceWeights)
{
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> result;

  // Create control points
  vpPclPointCloudVisualization::pclPointCloudPtr unrotatedControlPoints(new vpPclPointCloudVisualization::pclPointCloud(_n,_m));
  vpPclPointCloudVisualization::pclPointCloudPtr   rotatedControlPoints(new vpPclPointCloudVisualization::pclPointCloud(_n,_m));

  // Initializing confindence weights
  confidenceWeights.resize(_m * _n);

  for(unsigned int j =  0; j< _m; j++){
    for(unsigned int i = 0; i<_n; i++){
      double x = _minX + (double)i * _dX;
      double y = _minY + (double)j * _dY;
      double z = zFunction(x,y, order);

      std::vector<double> point={x, y, z,1.};
      vpColVector original = vpColVector(point);
      (*unrotatedControlPoints)(i,j).x = original[0];
      (*unrotatedControlPoints)(i,j).y = original[1];
      (*unrotatedControlPoints)(i,j).z = original[2];

      vpColVector rotated = _rotHunrotated * original;
      (*rotatedControlPoints)(i,j).x = rotated[0];
      (*rotatedControlPoints)(i,j).y = rotated[1];
      (*rotatedControlPoints)(i,j).z = rotated[2];

      confidenceWeights[j * _n + i] = 1.;
    }
  }

  if(addNoise)
  {
    vpUniRand r;
    // Draws the indices where to begin the fake failed tracking
    unsigned  int initXIndex = std::floor(r() *  (double)_n);
    unsigned  int initYIndex = std::floor(r() *  (double)_m);

    // Draws the size of the zone of the fake failed tracking: between 5*5 and 10*10 zone
    unsigned int sizeZone = std::floor(r() * 5. + 5.  );
    for(unsigned int j =  initYIndex; j< _m && j < initYIndex + sizeZone; j++){

      for(unsigned int i = initXIndex; i<_n && i < initXIndex + sizeZone; i++){
        double oX = _minX + (double)i * _dX;
        double oY = _minY + (double)j * _dY;
        double oZ = 0.;
        vpColVector oPos = std::vector<double>({oX, oY, oZ, 1.});

        vpColVector rotated = _rotHunrotated * oPos;
        (*rotatedControlPoints)(i,j).x = rotated[0];
        (*rotatedControlPoints)(i,j).y = rotated[1];
        (*rotatedControlPoints)(i,j).z = 0.;

        // Setting confidence weight to 0, as would do vpRobust
        // See [vpRobust documentation](https://visp-doc.inria.fr/doxygen/visp-daily/classvpRobust.html) for information and examples
        confidenceWeights[j * _n + i] = 0.;
      }
    }
  }

  result.first = unrotatedControlPoints;
  result.second = rotatedControlPoints;
  return result;
}

void ClassUsingPclVisualizer::blockingMode(const bool &addNoise, const unsigned int& order)
{
  // Confidence weights, that would be obtained thanks to vpRobust for instance
  vpColVector confWeights;

  //! [Generating point clouds]
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> grids = generateControlPoints(addNoise, order, confWeights);
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

void ClassUsingPclVisualizer::threadedMode(const bool &addNoise, const unsigned int& order)
{
  // Confidence weights, that would be obtained thanks to vpRobust for instance
  vpColVector confWeights;

  // Create control points
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> grids = generateControlPoints(addNoise, order, confWeights);

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
    grids = generateControlPoints(addNoise, order, confWeights);

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