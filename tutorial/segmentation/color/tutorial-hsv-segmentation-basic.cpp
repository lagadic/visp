#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHSV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayFactory.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  //! [Load image]
  vpImage<vpRGBa> I;
  vpImageIo::read(I, "ballons.jpg");
  //! [Load image]

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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  //! [Converting image]
  vpImage<vpHSV<unsigned char, true>> Ihsv;
  vpImageConvert::convert(I, Ihsv);
  //! [Converting image]

  //! [Create HSV mask]
  vpImage<unsigned char> maskHSV;
  vpImageConvert::convert(I, Ihsv);
  vpImageTools::inRange(Ihsv, hsv_range, maskHSV);
  //! [Create HSV mask]

  //! [Filter image]
  vpImage<vpRGBa> I_segmented_from_HSV;
  vpImageTools::inMask(I, maskHSV, I_segmented_from_HSV);
  //! [Filter image]
#endif

  //! [Without vpHSV]
  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();
  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);

  vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap),
                            reinterpret_cast<unsigned char *>(H.bitmap),
                            reinterpret_cast<unsigned char *>(S.bitmap),
                            reinterpret_cast<unsigned char *>(V.bitmap), I.getSize());

  vpImage<unsigned char> mask(height, width);
  vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                        reinterpret_cast<unsigned char *>(S.bitmap),
                        reinterpret_cast<unsigned char *>(V.bitmap),
                        hsv_range,
                        reinterpret_cast<unsigned char *>(mask.bitmap),
                        mask.getSize());

  vpImage<vpRGBa> I_segmented(height, width);
  vpImageTools::inMask(I, mask, I_segmented);
  //! [Without vpHSV]

#if defined(VISP_HAVE_DISPLAY)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d_I = vpDisplayFactory::createDisplay(I);
  std::shared_ptr<vpDisplay> d_mask = vpDisplayFactory::createDisplay(mask, I.getWidth()+75, 0, "HSV mask");
  std::shared_ptr<vpDisplay> d_I_segmented = vpDisplayFactory::createDisplay(I_segmented, 2*mask.getWidth()+80, 0, "Segmented frame");
  std::shared_ptr<vpDisplay> d_mask_hsv = vpDisplayFactory::createDisplay(maskHSV, I.getWidth()+75, mask.getHeight() + 80, "HSV mask using vpHSV");
  std::shared_ptr<vpDisplay> d_I_segmented_hsv = vpDisplayFactory::createDisplay(I_segmented_from_HSV, 2*mask.getWidth()+80, mask.getHeight() + 80, "Segmented frame using vpHSV");
#else
  vpDisplay *d_I = vpDisplayFactory::allocateDisplay(I, 0, 0, "Current frame");
  vpDisplay *d_mask = vpDisplayFactory::allocateDisplay(mask, I.getWidth()+75, 0, "HSV mask");
  vpDisplay *d_I_segmented = vpDisplayFactory::allocateDisplay(I_segmented, 2*mask.getWidth()+80, 0, "Segmented frame");
#endif

  vpDisplay::display(I);
  vpDisplay::display(mask);
  vpDisplay::display(I_segmented);
  vpDisplay::flush(I);
  vpDisplay::flush(mask);
  vpDisplay::flush(I_segmented);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpDisplay::display(I_segmented_from_HSV);
  vpDisplay::flush(I_segmented_from_HSV);
  vpDisplay::display(maskHSV);
  vpDisplay::flush(maskHSV);
#endif
  vpDisplay::getClick(I);

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (d_I != nullptr) {
    delete d_I;
  }

  if (d_mask != nullptr) {
    delete d_mask;
  }

  if (d_I_segmented != nullptr) {
    delete d_I_segmented;
  }
#endif
#endif
  return EXIT_SUCCESS;
}
