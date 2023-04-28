#include "ClassUsingPclVisualizer.h"

#if defined(VISP_HAVE_PCL)
// PCL
#include <pcl/io/pcd_io.h>

// Visp
#include <visp3/core/vpTime.h>
#include <visp3/core/vpUniRand.h>
#include <visp3/io/vpKeyboard.h>

double zFunction(const double &x, const double &y, const unsigned int order)
{
  const double offset(0.5);
  double z(0.);

  std::cout << "Z = ";
  for(unsigned int n = 0; n <= order; n++)
  {
    for(unsigned int k = 0; k <= order - n; k++)
    {
      if(k + n > 0)
      {
        z += std::pow(x, n) * std::pow(y, k);
        std::cout << "X^" << n << " x Y^" << k << " + ";
      }
      else
      {
        z += offset;
        std::cout << offset<< " + ";
      }

    }
  }
  std::cout << std::endl << std::flush;

  return z;
}

ClassUsingPclVisualizer::ClassUsingPclVisualizer(std::pair<double, double> xlimits, std::pair<double, double> ylimits, std::pair<unsigned int, unsigned int> nbPoints)
  : _minX(xlimits.first)
  , _maxX(xlimits.second)
  , _n(nbPoints.first)
  , _minY(ylimits.first)
  , _maxY(ylimits.second)
  , _m(nbPoints.second)
  , _t(0.,0.,0.)
  , _R(M_PI_4,M_PI_4,M_PI_4)
  , _rotHunrotated(_t,_R)
  , _visualizer("Grid of points with / without robust")
{
  _dX = (_maxX - _minX) / (static_cast<double>(_n) - 1.);
  _dY = (_maxY - _minY) / (static_cast<double>(_m) - 1.);
}

ClassUsingPclVisualizer::~ClassUsingPclVisualizer()
{

}

std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> ClassUsingPclVisualizer::generateControlPoints(const bool &addNoise, const unsigned int &order)
{
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> result;

  // Create control points
  vpPclPointCloudVisualization::pclPointCloudPtr unrotatedControlPoints(new vpPclPointCloudVisualization::pclPointCloud(_n,_m));
  vpPclPointCloudVisualization::pclPointCloudPtr   rotatedControlPoints(new vpPclPointCloudVisualization::pclPointCloud(_n,_m));
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
        double x = _minX + (double)i * _dX;
        double y = _minY + (double)j * _dY;
        double z = 0.;

        std::vector<double> point={x, y, z,1.};
        vpColVector original = vpColVector(point);
        (*unrotatedControlPoints)(i,j).x = original[0];
        (*unrotatedControlPoints)(i,j).y = original[1];
        (*unrotatedControlPoints)(i,j).z = original[2];

        vpColVector rotated = _rotHunrotated * original;
        (*rotatedControlPoints)(i,j).x = rotated[0];
        (*rotatedControlPoints)(i,j).y = rotated[1];
        (*rotatedControlPoints)(i,j).z = 0.;
      }
    }
  }

  result.first = unrotatedControlPoints;
  result.second = rotatedControlPoints;
  return result;
}

void ClassUsingPclVisualizer::blockingMode(const bool &addNoise, const unsigned int& order)
{
  // Create control points
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> grids = generateControlPoints(addNoise, order);

  unsigned int id_ctrlPts = _visualizer.addSurface(grids.first, "Standard");
  unsigned int id_robust = _visualizer.addSurface(grids.second, "RotatedWithRobust");
  
  _visualizer.display();

}

void ClassUsingPclVisualizer::threadedMode(const bool &addNoise, const unsigned int& order)
{
  // Create control points
  std::pair<vpPclPointCloudVisualization::pclPointCloudPtr, vpPclPointCloudVisualization::pclPointCloudPtr> grids = generateControlPoints(addNoise, order);

  unsigned int id_ctrlPts = _visualizer.addSurface(grids.first, "Standard");
  unsigned int id_robust = _visualizer.addSurface(grids.second, "RotatedWithRobust");
  _visualizer.launchThread();
  
  _visualizer.threadUpdateSurface(grids.first , id_ctrlPts);
  _visualizer.threadUpdateSurface(grids.second , id_robust);

  vpKeyboard keyboard;
  bool wantToStop = false;
  double t;
  
  while(!wantToStop){
    t = vpTime::measureTimeMs();
    grids = generateControlPoints(addNoise, order);

    _visualizer.threadUpdateSurface(grids.first , id_ctrlPts);
    _visualizer.threadUpdateSurface(grids.second , id_robust);

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