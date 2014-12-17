//! \example tutorial-homography-from-points.cpp

//! [Include]
#include <visp/vpHomography.h>
//! [Include]
#include <visp/vpMeterPixelConversion.h>

int main()
{
  //! [Set 3D points]
  double L = 0.1;
  std::vector<vpPoint> oP(4);
  oP[0].setWorldCoordinates( -L,-L,   0);
  oP[1].setWorldCoordinates(2*L,-L,   0);
  oP[2].setWorldCoordinates(  L, 3*L, 0);
  oP[3].setWorldCoordinates( -L, 4*L, 0);
  //! [Set 3D points]

  //! [Simulation]
  vpHomogeneousMatrix bMo(0.1, 0, 1,   0, vpMath::rad(15), 0);
  vpHomogeneousMatrix aMb(0.2, -0.1, 0.1, vpMath::rad(-3), vpMath::rad(20), vpMath::rad(5));
  vpHomogeneousMatrix aMo = aMb*bMo;
  //! [Simulation]

  //! [Image plane coordinates]
  std::vector<vpPoint> aP(4), bP(4);
  std::vector<double> xa(4), ya(4), xb(4), yb(4);
  for(unsigned int i=0 ; i < 4; i++)
  {
    oP[i].project(aMo);
    xa[i] = oP[i].get_x();
    ya[i] = oP[i].get_y();
    oP[i].project(bMo);
    xb[i] = oP[i].get_x();
    yb[i] = oP[i].get_y();
  }
  //! [Image plane coordinates]

  //! [Compute homography]
  vpHomography aHb ;
  vpHomography::DLT(xb, yb, xa, ya, aHb, true);
  std::cout << "Estimated homography using DLT:\n" << aHb/aHb[2][2] << std::endl;

  vpHomography::HLM(xb, yb, xa, ya, true, aHb);
  std::cout << "Estimated homography using HLM:\n" << aHb/aHb[2][2] << std::endl;
  //! [Compute homography]

  //! [Get transformation]
  vpRotationMatrix aRb;
  vpTranslationVector atb;
  vpColVector n;
  aHb.computeDisplacement(aRb, atb, n);
  //! [Get transformation]

  //! [Print results]
  std::cout << "\nEstimated displacement:"  << std::endl;
  std::cout << " atb: " << atb.t() << std::endl;
  vpThetaUVector atub;
  atub.buildFrom(aRb);
  std::cout << " athetaub: ";
  for(unsigned int i=0; i<3; i++)
    std::cout << vpMath::deg(atub[i]) << " ";
  std::cout << std::endl;
  std::cout << " n: " << n.t() << std::endl;
  //! [Print results]

  //! [Get pixel coordinates]
  vpImagePoint iPa, iPb;
  vpCameraParameters cam;
  vpMeterPixelConversion::convertPoint(cam, xb[3], yb[3], iPb);
  vpMeterPixelConversion::convertPoint(cam, xa[3], ya[3], iPa);

  std::cout << "Ground truth: Point 3 in pixels in frame b: " << iPb << std::endl;
  std::cout << "Ground truth: Point 3 in pixels in frame a: " << iPa << std::endl;
  //! [Get pixel coordinates]

  //! [Project]
  // Project the position in pixel of point 3 from the homography
  std::cout << "Estimation from homography: Point 3 in pixels in frame a: "
            << vpHomography::project(cam, aHb, iPb) << std::endl;
  //! [Project]
}
