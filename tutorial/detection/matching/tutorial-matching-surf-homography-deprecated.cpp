//! \example tutorial-matching-surf-homography-deprecated.cpp
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/vision/vpKeyPointSurf.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/io/vpVideoReader.h>

int main(int argc, const char **argv)
{  
#if defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION < 0x030000)
  //! [Select method]
  int method = 0;

  if (argc > 1)
    method = atoi(argv[1]);

  if (method == 0)
    std::cout << "Uses Ransac to estimate the homography" << std::endl;
  else
    std::cout << "Uses a robust scheme to estimate the homography" << std::endl;
  //! [Select method]

  vpImage<unsigned char> I;

  vpVideoReader reader;
  reader.setFileName("video-postcard.mpeg");
  reader.acquire(I);

  vpKeyPointSurf surf;
  surf.buildReference(I);

  vpImage<unsigned char> Idisp;
  Idisp.resize(I.getHeight(), 2*I.getWidth());
  Idisp.insert(I, vpImagePoint(0, 0));
  Idisp.insert(I, vpImagePoint(0, I.getWidth()));

  vpDisplayOpenCV d(Idisp, 0, 0, "Homography from matched Surf keypoints");
  vpDisplay::display(Idisp);
  vpDisplay::flush(Idisp);

  //! [Set coordinates]
  vpImagePoint corner_ref[4];
  corner_ref[0].set_ij(115,  64);
  corner_ref[1].set_ij( 83, 253);
  corner_ref[2].set_ij(282, 307);
  corner_ref[3].set_ij(330,  72);
  //! [Set coordinates]
  //! [Display]
  for (unsigned int i=0; i<4; i++) {
    vpDisplay::displayCross(Idisp, corner_ref[i], 12, vpColor::red);
  }
  vpDisplay::flush(Idisp);
  //! [Display]

  //! [Camera]
  vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);
  //! [Camera]

  vpHomography curHref;
  while ( ! reader.end() )
  {
    reader.acquire(I);
    Idisp.insert(I, vpImagePoint(0, I.getWidth()));
    vpDisplay::display(Idisp);
    vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()), vpColor::white, 2);

    //! [Matching]
    unsigned int nbMatch = surf.matchPoint(I);
    //! [Matching]

    std::vector<vpImagePoint> iPref(nbMatch), iPcur(nbMatch); // Coordinates in pixels (for display only)
    //! [Allocation]
    std::vector<double> mPref_x(nbMatch), mPref_y(nbMatch);
    std::vector<double> mPcur_x(nbMatch), mPcur_y(nbMatch);
    std::vector<bool> inliers(nbMatch);
    //! [Allocation]

    for (unsigned int i = 0; i < nbMatch; i++) {
      vpImagePoint matched_ref, matched_cur;
      surf.getMatchedPoints(i, matched_ref, matched_cur);
      //! [Pixel conversion]
      vpPixelMeterConversion::convertPoint(cam, matched_ref, mPref_x[i], mPref_y[i]);
      vpPixelMeterConversion::convertPoint(cam, matched_cur, mPcur_x[i], mPcur_y[i]);
      //! [Pixel conversion]

      // Store the image coordinates in pixel of the matched points
      iPref[i] = matched_ref;
      iPcur[i] = matched_cur;
    }

    //! [Homography estimation]
    double residual;
    if (method == 0)
      vpHomography::ransac(mPref_x, mPref_y, mPcur_x, mPcur_y, curHref, inliers, residual,
                           (unsigned int)mPref_x.size()/2, 2.0/cam.get_px(), true);
    else
      vpHomography::robust(mPref_x, mPref_y, mPcur_x, mPcur_y, curHref, inliers, residual,
                           0.4, 4, true);
    //! [Homography estimation]

    //! [Projection]
    vpImagePoint corner_cur[4];
    for (int i=0; i< 4; i++) {
      corner_cur[i] = vpHomography::project(cam, curHref, corner_ref[i]);
    }
    //! [Projection]

    //! [Display contour]
    vpImagePoint offset(0, I.getWidth());
    for (int i=0; i< 4; i++) {
      vpDisplay::displayLine(Idisp,
                             corner_cur[i]       + offset,
                             corner_cur[(i+1)%4] + offset,
                             vpColor::blue, 3);
    }
    //! [Display contour]

    //! [Display matches]
    for (unsigned int i = 0; i < nbMatch; i++) {
      if(inliers[i] == true)
        vpDisplay::displayLine(Idisp, iPref[i], iPcur[i] + offset, vpColor::green);
      else
        vpDisplay::displayLine(Idisp, iPref[i], iPcur[i] + offset, vpColor::red);
    }
    //! [Display matches]

    vpDisplay::flush(Idisp);

    if (vpDisplay::getClick(Idisp, false))
      break;
  }

  vpDisplay::getClick(Idisp);
#else
  (void)argc; (void)argv;
#endif
  return 0;
}
