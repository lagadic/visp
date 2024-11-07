//! \example tutorial-munkres-assignment.cpp

#include <functional>

#include <visp3/core/vpConfig.h>
// Display
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/core/vpColor.h>

// Munkres
#include <visp3/core/vpMunkres.h>

// Math
#include <visp3/core/vpUniRand.h>

int main()
{
  // Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#if defined(VISP_HAVE_DISPLAY)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  // Create base img
  vpImage<unsigned char> I(480, 640, 255);

  // Generate random points
  //! [Rand_Img_Pts]
  vpUniRand rand {};
  std::vector<vpImagePoint> rand_ips {};
  while (rand_ips.size() < 10) {
    rand_ips.emplace_back(rand.uniform(10, I.getHeight() - 10), rand.uniform(10, I.getWidth() - 10));
  }
  //! [Rand_Img_Pts]

  try {
    // Init display
    const auto disp_scale_type = vpDisplay::SCALE_AUTO;
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, disp_scale_type);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, disp_scale_type);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, disp_scale_type);
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d(I, disp_scale_type);
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D d(I, disp_scale_type);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Munkres Assignment Algorithm");

    // Local helper to display a point in the image
    auto display_point = [&I](const vpImagePoint &ip, const vpColor &color) {
      I.display->displayCircle(ip, 5, color, true, 1);
      };

    vpDisplay::display(I);

    auto disp_lane { 0 };
    vpDisplay::displayText(I, 15 * ++disp_lane, 15, "Left click to add a point", vpColor::black);
    vpDisplay::displayText(I, 15 * ++disp_lane, 15, "Middle click to continue (run Munkres)", vpColor::black);
    vpDisplay::displayText(I, 15 * ++disp_lane, 15, "Right click to quit", vpColor::black);

    std::for_each(begin(rand_ips), end(rand_ips), std::bind(display_point, std::placeholders::_1, vpColor::red));
    vpDisplay::flush(I);

    // Ask user to clic on point
    //! [User_Img_Pts]
    std::vector<vpImagePoint> user_ips {};
    vpMouseButton::vpMouseButtonType button {};
    while (button != vpMouseButton::button2) {
      vpImagePoint ip {};
      vpDisplay::getClick(I, ip, button, true);
      if (button == vpMouseButton::button1) {
        user_ips.push_back(ip);
      }
      else if (button == vpMouseButton::button3) {
        return EXIT_SUCCESS;
      }

      std::for_each(begin(user_ips), end(user_ips), std::bind(display_point, std::placeholders::_1, vpColor::green));

      vpDisplay::flush(I);
    }
    //! [User_Img_Pts]

    // Prepare Munkres (init cost matrix with random ip / user ip distances)
    //! [Cost_Matrix]
    std::vector<std::vector<double> > cost_matrix(rand_ips.size(), std::vector<double>(user_ips.size()));
    for (auto i = 0u; i < rand_ips.size(); i++) {
      for (auto j = 0u; j < user_ips.size(); j++) {
        cost_matrix.at(i).at(j) = vpImagePoint::distance(rand_ips.at(i), user_ips.at(j));
      }
    }
    //! [Cost_Matrix]

    // Display results
    vpDisplay::display(I);
    std::for_each(begin(rand_ips), end(rand_ips), std::bind(display_point, std::placeholders::_1, vpColor::red));
    std::for_each(begin(user_ips), end(user_ips), std::bind(display_point, std::placeholders::_1, vpColor::green));

    //! [Run]
    for (const auto &[i, j] : vpMunkres::run(cost_matrix)) {
      I.display->displayLine(rand_ips.at(i), user_ips.at(j), vpColor::blue, 1);
    }
    //! [Run]

    vpDisplay::displayText(I, 15, 15, "Click to quit", vpColor::black);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif // defined(VISP_HAVE_DISPLAY)
#endif
  return EXIT_SUCCESS;
}
