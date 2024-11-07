//! \example tutorial-flood-fill.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
//! [Bresenham's line algorithm]
vpImagePoint switchToOctantZeroFrom(const int octant, const vpImagePoint &imPt)
{
  vpImagePoint imPt_switched = imPt;

  switch (octant) {
  case 0: // return (x, y)
    imPt_switched.set_uv(imPt.get_u(), imPt.get_v());
    break;

  case 1: // return (y, x)
    imPt_switched.set_uv(imPt.get_v(), imPt.get_u());
    break;

  case 2: // return (y, -x)
    imPt_switched.set_uv(imPt.get_v(), -imPt.get_u());
    break;

  case 3: // return (-x, y)
    imPt_switched.set_uv(-imPt.get_u(), imPt.get_v());
    break;

  case 4: // return (-x, -y)
    imPt_switched.set_uv(-imPt.get_u(), -imPt.get_v());
    break;

  case 5: // return (-y, -x)
    imPt_switched.set_uv(-imPt.get_v(), -imPt.get_u());
    break;

  case 6: // return (-y, x)
    imPt_switched.set_uv(-imPt.get_v(), imPt.get_u());
    break;

  case 7: // return (x, -y)
    imPt_switched.set_uv(imPt.get_u(), -imPt.get_v());
    break;

  default:
    break;
  }

  return imPt_switched;
}

vpImagePoint switchFromOctantZeroTo(const int octant, const vpImagePoint &imPt)
{
  vpImagePoint imPt_switched = imPt;

  switch (octant) {
  case 0: // return (x, y)
    imPt_switched.set_uv(imPt.get_u(), imPt.get_v());
    break;

  case 1: // return (y, x)
    imPt_switched.set_uv(imPt.get_v(), imPt.get_u());
    break;

  case 2: // return (-y, x)
    imPt_switched.set_uv(-imPt.get_v(), imPt.get_u());
    break;

  case 3: // return (-x, y)
    imPt_switched.set_uv(-imPt.get_u(), imPt.get_v());
    break;

  case 4: // return (-x, -y)
    imPt_switched.set_uv(-imPt.get_u(), -imPt.get_v());
    break;

  case 5: // return (-y, -x)
    imPt_switched.set_uv(-imPt.get_v(), -imPt.get_u());
    break;

  case 6: // return (y, -x)
    imPt_switched.set_uv(imPt.get_v(), -imPt.get_u());
    break;

  case 7: // return (x, -y)
    imPt_switched.set_uv(imPt.get_u(), -imPt.get_v());
    break;

  default:
    break;
  }

  return imPt_switched;
}

int getOctant(const vpImagePoint &imPt1, const vpImagePoint &imPt2)
{
  double dx = imPt2.get_u() - imPt1.get_u();
  double dy = imPt2.get_v() - imPt1.get_v();

  if (dx >= 0 && dy >= 0) {
    if (dy >= dx) {
      return 1;
    }
    else {
      return 0;
    }
  }
  else if (dx < 0 && dy >= 0) {
    if (-dx >= dy) {
      return 3;
    }
    else {
      return 2;
    }
  }
  else if (dx < 0 && dy < 0) {
    if (dy <= dx) {
      return 5;
    }
    else {
      return 4;
    }
  }
  else {
    if (dx >= -dy) {
      return 7;
    }
    else {
      return 6;
    }
  }
}

void drawLine(vpImage<unsigned char> &I, const unsigned char value, const vpImagePoint &imPt1_,
              const vpImagePoint &imPt2_)
{
  vpImagePoint imPt1((int)imPt1_.get_v(), (int)imPt1_.get_u());
  vpImagePoint imPt2((int)imPt2_.get_v(), (int)imPt2_.get_u());

  int octant = getOctant(imPt1, imPt2);
  imPt1 = switchToOctantZeroFrom(octant, imPt1);
  imPt2 = switchToOctantZeroFrom(octant, imPt2);

  double dx = imPt2.get_u() - imPt1.get_u();
  double dy = imPt2.get_v() - imPt1.get_v();
  double D = 2 * dy - dx;
  double y = imPt1.get_v();

  for (int x = (int)imPt1.get_u(); x <= (int)imPt2.get_u(); x++) {
    vpImagePoint currentPt(y, x);
    currentPt = switchFromOctantZeroTo(octant, currentPt);

    unsigned int i = std::min<unsigned int>(I.getHeight() - 1, (unsigned int)std::max<double>(0.0, currentPt.get_i()));
    unsigned int j = std::min<unsigned int>(I.getWidth() - 1, (unsigned int)std::max<double>(0.0, currentPt.get_j()));
    I[i][j] = value;

    if (D >= 0) {
      y++;
      D -= dx;
    }

    D += dy;
  }
}
//! [Bresenham's line algorithm]
} // namespace

#endif

int main()
{
//! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]

  //! [Create bitmap]
  vpImage<vpRGBa> I(480, 640, vpRGBa());
  //! [Create bitmap]

#ifdef VISP_HAVE_X11
  vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d;
#endif
  d.init(I, 0, 0, "Paint");

  //! [Draw polygons]
  std::vector<vpPolygon> polygons;
  for (int i = 0; i < 3; i++) {
    vpDisplay::display(I);
    std::stringstream ss;
    ss << "Left click to draw polygon " << i + 1 << "/3"
      << ", right click to close the shape.";
    vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
    vpDisplay::flush(I);

    vpPolygon polygon;
    polygon.initClick(I);
    polygons.push_back(polygon);

    vpDisplay::display(I);
    vpDisplay::displayLine(I, polygon.getCorners(), true, vpColor::red);
    vpDisplay::flush(I);

    // Update the lines draw internally in the current image
    vpDisplay::getImage(I, I);
  }
  //! [Draw polygons]

  //! [Draw polygon lines]
  vpImage<unsigned char> mask(I.getHeight(), I.getWidth(), 0);
  for (size_t i = 0; i < polygons.size(); i++) {
    if (polygons[i].getCorners().size() <= 1)
      continue;

    for (size_t j = 0; j < polygons[i].getCorners().size() - 1; j++)
      drawLine(mask, 255, polygons[i].getCorners()[j], polygons[i].getCorners()[j + 1]);

    drawLine(mask, 255, polygons[i].getCorners().front(), polygons[i].getCorners().back());
  }
  //! [Draw polygon lines]

  bool quit = false;
  while (!quit) {
    vpDisplay::display(I);
    vpDisplay::displayText(I, 20, 20,
                           "Left click on a pixel location to fill the "
                           "shape, right click to quit.",
                           vpColor::red);
    vpDisplay::flush(I);

    //! [Seed point click]
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, ip, button, false))
    //! [Seed point click]
    {
      switch (button) {
      case vpMouseButton::button1:
        //! [Flood fill]
        VISP_NAMESPACE_NAME::floodFill(mask, ip, 0, 255, vpImageMorphology::CONNEXITY_4);
        //! [Flood fill]

        //! [Bucket fill]
        for (unsigned int cpt = 0; cpt < mask.getSize(); cpt++) {
          if (mask.bitmap[cpt])
            I.bitmap[cpt] = vpColor::red;
        }
        //! [Bucket fill]
        break;

      case vpMouseButton::button3:
        quit = true;
        break;

      default:
        break;
      }
    }
  }
#endif

  return EXIT_SUCCESS;
}
