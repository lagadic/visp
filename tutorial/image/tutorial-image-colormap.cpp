/*! \example tutorial-image-colormap.cpp */
#include <map>
#include <visp3/core/vpColormap.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpFont.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::map<vpColormap::vpColormapType, std::string> colormaps_str = {
        {vpColormap::COLORMAP_AUTUMN, "Colormap Autumn"},
        {vpColormap::COLORMAP_CIVIDIS, "Colormap Cividis"},
        {vpColormap::COLORMAP_COOL, "Colormap Cool"},
        {vpColormap::COLORMAP_GIST_EARTH, "Colormap Gist Earth"},
        {vpColormap::COLORMAP_GNUPLOT, "Colormap Gnuplot"},
        {vpColormap::COLORMAP_GNUPLOT2, "Colormap Gnuplot2"},
        {vpColormap::COLORMAP_HOT, "Colormap Hot"},
        {vpColormap::COLORMAP_HSV, "Colormap HSV"},
        {vpColormap::COLORMAP_INFERNO, "Colormap Inferno"},
        {vpColormap::COLORMAP_JET, "Colormap Jet"},
        {vpColormap::COLORMAP_MAGMA, "Colormap Magma"},
        {vpColormap::COLORMAP_OCEAN, "Colormap Ocean"},
        {vpColormap::COLORMAP_PLASMA, "Colormap Plasma"},
        {vpColormap::COLORMAP_RAINBOW, "Colormap Rainbow"},
        {vpColormap::COLORMAP_SPRING, "Colormap Spring"},
        {vpColormap::COLORMAP_SUMMER, "Colormap Summer"},
        {vpColormap::COLORMAP_TERRAIN, "Colormap Terrain"},
        {vpColormap::COLORMAP_TURBO, "Colormap Turbo"},
        {vpColormap::COLORMAP_TWILIGHT, "Colormap Twilight"},
        {vpColormap::COLORMAP_TWILIGHT_SHIFTED, "Colormap Twilight Shifted"},
        {vpColormap::COLORMAP_VIRIDIS, "Colormap Viridis"},
        {vpColormap::COLORMAP_WINTER, "Colormap Winter"}
    };

    std::vector<vpColormap::vpColormapType> colormaps = {
        vpColormap::COLORMAP_AUTUMN,   vpColormap::COLORMAP_CIVIDIS,
        vpColormap::COLORMAP_COOL,     vpColormap::COLORMAP_GIST_EARTH,
        vpColormap::COLORMAP_GNUPLOT,  vpColormap::COLORMAP_GNUPLOT2,
        vpColormap::COLORMAP_HOT,      vpColormap::COLORMAP_HSV,
        vpColormap::COLORMAP_INFERNO,  vpColormap::COLORMAP_JET,
        vpColormap::COLORMAP_MAGMA,    vpColormap::COLORMAP_OCEAN,
        vpColormap::COLORMAP_PLASMA,   vpColormap::COLORMAP_RAINBOW,
        vpColormap::COLORMAP_SPRING,   vpColormap::COLORMAP_SUMMER,
        vpColormap::COLORMAP_TERRAIN,  vpColormap::COLORMAP_TURBO,
        vpColormap::COLORMAP_TWILIGHT, vpColormap::COLORMAP_TWILIGHT_SHIFTED,
        vpColormap::COLORMAP_VIRIDIS,  vpColormap::COLORMAP_WINTER
    };

    // Apply a colormap on a 3-channel floating-point image
    {
      vpImage<vpRGBf> Irgbf;
      vpImageIo::readPFM_HDR(Irgbf, "memorial.pfm");

      vpImage<vpRGBa> Icolor(Irgbf.getHeight(), Irgbf.getWidth());
      vpImage<vpRGBa> Icolor2(Irgbf.getHeight() * 2, Irgbf.getWidth() * 2);

#if defined(VISP_HAVE_X11)
      vpDisplayX d(Icolor2, 10, 10, "Memorial");
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d(Icolor2, 10, 10, "Memorial");
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV d(Icolor2, 10, 10, "Memorial");
#else
      std::cout << "No image viewer is available..." << std::endl;
      return EXIT_SUCCESS;
#endif

      vpFont font(20);
      for (size_t i = 0; i < colormaps.size(); i++) {
        vpColormap colormap(colormaps[i]);
        colormap.convert(Irgbf, Icolor);
        vpImageTools::resize(Icolor, Icolor2, vpImageTools::INTERPOLATION_LINEAR);

        font.drawText(Icolor2, colormaps_str[colormaps[i]], vpImagePoint(20, 20), vpColor::black, vpColor::white);

        vpDisplay::display(Icolor2);
        vpDisplay::flush(Icolor2);
        vpDisplay::getClick(Icolor2);
      }
    }

    // Apply a colormap on a 8-bit RGB image
    {
      vpImage<vpRGBa> I;
      vpImageIo::read(I, "monkey.png");

      vpImage<vpRGBa> Icolor(I.getHeight(), I.getWidth());
      vpImage<vpRGBa> Icolor2(I.getHeight() * 2, I.getWidth() * 2);

#if defined(VISP_HAVE_X11)
      vpDisplayX d(Icolor2, 10, 10, "Monkey");
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d(Icolor2, 10, 10, "Monkey");
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV d(Icolor2, 10, 10, "Monkey");
#else
      std::cout << "No image viewer is available..." << std::endl;
      return EXIT_SUCCESS;
#endif

      vpFont font(20);
      for (size_t i = 0; i < colormaps.size(); i++) {
        vpColormap colormap(colormaps[i]);
        colormap.convert(I, Icolor);
        vpImageTools::resize(Icolor, Icolor2, vpImageTools::INTERPOLATION_LINEAR);

        font.drawText(Icolor2, colormaps_str[colormaps[i]], vpImagePoint(20, 20), vpColor::black, vpColor::white);

        vpDisplay::display(Icolor2);
        vpDisplay::flush(Icolor2);
        vpDisplay::getClick(Icolor2);
      }
    }

    // Apply a colormap on a 8-bit RGB image, with normalization to the [0 - 255] range
    {
      vpImage<vpRGBa> I;
      vpImageIo::read(I, "monkey.png");

      vpImage<vpRGBa> Icolor(I.getHeight(), I.getWidth());
      vpImage<vpRGBa> Icolor2(I.getHeight() * 2, I.getWidth() * 2);

#if defined(VISP_HAVE_X11)
      vpDisplayX d(Icolor2, 10, 10, "Monkey");
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d(Icolor2, 10, 10, "Monkey");
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV d(Icolor2, 10, 10, "Monkey");
#else
      std::cout << "No image viewer is available..." << std::endl;
      return EXIT_SUCCESS;
#endif

      vpFont font(20);
      for (size_t i = 0; i < colormaps.size(); i++) {
        vpColormap colormap(colormaps[i]);
        const bool normalise = true;
        colormap.convert(I, Icolor, normalise);
        vpImageTools::resize(Icolor, Icolor2, vpImageTools::INTERPOLATION_LINEAR);

        font.drawText(Icolor2, colormaps_str[colormaps[i]], vpImagePoint(20, 20), vpColor::black, vpColor::white);

        vpDisplay::display(Icolor2);
        vpDisplay::flush(Icolor2);
        vpDisplay::getClick(Icolor2);
      }
    }
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e << std::endl;
  }
#else
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
  return EXIT_SUCCESS;
}
