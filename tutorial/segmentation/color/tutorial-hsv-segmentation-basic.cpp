#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<vpRGBa> I;
  vpImageIo::read(I, "ballons.jpg");

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);

  vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap),
                            reinterpret_cast<unsigned char *>(H.bitmap),
                            reinterpret_cast<unsigned char *>(S.bitmap),
                            reinterpret_cast<unsigned char *>(V.bitmap), I.getSize());

  //! [Set HSV range]
  int h = 14, s = 255, v = 209;
  int offset = 30;
  int h_low = std::max<int>(0, h - offset), h_high = std::min<int>(h + offset, 255);
  int s_low = std::max<int>(0, s - offset), s_high = std::min<int>(s + offset, 255);
  int v_low = std::max<int>(0, v - offset), v_high = std::min<int>(v + offset, 255);
  std::vector<int> hsv_range;
  hsv_range.push_back(h_low);
  hsv_range.push_back(h_high);
  hsv_range.push_back(s_low);
  hsv_range.push_back(s_high);
  hsv_range.push_back(v_low);
  hsv_range.push_back(v_high);
  //! [Set HSV range]

  //! [Create HSV mask]
  vpImage<unsigned char> mask(height, width);
  vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                        reinterpret_cast<unsigned char *>(S.bitmap),
                        reinterpret_cast<unsigned char *>(V.bitmap),
                        hsv_range,
                        reinterpret_cast<unsigned char *>(mask.bitmap),
                        mask.getSize());
  //! [Create HSV mask]

  vpImage<vpRGBa> I_segmented(height, width);
  vpImageTools::inMask(I, mask, I_segmented);

#if defined(VISP_HAVE_X11)
  vpDisplayX d_I(I, 0, 0, "Current frame");
  vpDisplayX d_mask(mask, I.getWidth()+75, 0, "HSV mask");
  vpDisplayX d_I_segmented(I_segmented, 2*mask.getWidth()+80, 0, "Segmented frame");

  vpDisplay::display(I);
  vpDisplay::display(mask);
  vpDisplay::display(I_segmented);
  vpDisplay::flush(I);
  vpDisplay::flush(mask);
  vpDisplay::flush(I_segmented);
  vpDisplay::getClick(I);
#endif
}
