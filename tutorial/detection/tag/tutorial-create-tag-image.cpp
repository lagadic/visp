//! \example tutorial-create-tag-image.cpp
#include <visp3/core/vpConfig.h>
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

void usage(const char **argv, int error);

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--output, -o <filename>]"
    << " [--tag-size <size>]"
    << " [--tag-family <family>]"
    << " [--tag-id <id>]"
#if defined(VISP_HAVE_DISPLAY)
    << " [--display, -d]"
#endif
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Create AprilTag or ArUco marker image." << std::endl
    << std::endl
    << "  --output, -o <filename>" << std::endl
    << "    Output image." << std::endl
    << "    Default: tag.png" << std::endl
    << std::endl
    << "  --tag-size <size>" << std::endl
    << "    Tag size in pixels." << std::endl
    << "    Default: 400" << std::endl
    << std::endl
    << "  --tag-family <family>" << std::endl
    << "    Apriltag family. Supported values are:" << std::endl
    << "       0: TAG_36h11" << std::endl
    << "       1: TAG_36h10 (DEPRECATED)" << std::endl
    << "       2: TAG_36ARTOOLKIT (DEPRECATED)" << std::endl
    << "       3: TAG_25h9" << std::endl
    << "       4: TAG_25h7 (DEPRECATED)" << std::endl
    << "       5: TAG_16h5" << std::endl
    << "       6: TAG_CIRCLE21h7" << std::endl
    << "       7: TAG_CIRCLE49h12" << std::endl
    << "       8: TAG_CUSTOM48h12" << std::endl
    << "       9: TAG_STANDARD41h12" << std::endl
    << "      10: TAG_STANDARD52h13" << std::endl
    << "      11: TAG_ARUCO_4x4_50" << std::endl
    << "      12: TAG_ARUCO_4x4_100" << std::endl
    << "      13: TAG_ARUCO_4x4_250" << std::endl
    << "      14: TAG_ARUCO_4x4_1000" << std::endl
    << "      15: TAG_ARUCO_5x5_50" << std::endl
    << "      16: TAG_ARUCO_5x5_100" << std::endl
    << "      17: TAG_ARUCO_5x5_250" << std::endl
    << "      18: TAG_ARUCO_5x5_1000" << std::endl
    << "      19: TAG_ARUCO_6x6_50" << std::endl
    << "      20: TAG_ARUCO_6x6_100" << std::endl
    << "      21: TAG_ARUCO_6x6_250" << std::endl
    << "      22: TAG_ARUCO_6x6_1000" << std::endl
    << "      23: TAG_ARUCO_7x7_50" << std::endl
    << "      24: TAG_ARUCO_7x7_100" << std::endl
    << "      25: TAG_ARUCO_7x7_250" << std::endl
    << "      26: TAG_ARUCO_7x7_1000" << std::endl
    << "      27: TAG_ARUCO_MIP_36h12" << std::endl
    << "    Default: 0 (36h11)" << std::endl
    << std::endl
    << "  --tag-id <id>" << std::endl
    << "    Marker id. " << std::endl
    << "    Default: 0" << std::endl
    << std::endl
#if defined(VISP_HAVE_DISPLAY)
    << "  --display, -d" << std::endl
    << "    Display generated marker." << std::endl
    << "    Default: disabled" << std::endl
    << std::endl
#endif
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_APRILTAG)

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;

  std::string opt_filename = "tag.png";
  bool opt_display_tag = false;
  unsigned int opt_pix_size = 400;
  int opt_tag_id = 0;

  for (int i = 1; i < argc; ++i) {
    if (((std::string(argv[i]) == "--output") || (std::string(argv[i]) == "-o")) && (i + 1 < argc)) {
      opt_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_pix_size = static_cast<unsigned int>(atoi(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      opt_tag_family = static_cast<vpDetectorAprilTag::vpAprilTagFamily>(atoi(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-id" && i + 1 < argc) {
      opt_tag_id = atoi(argv[++i]);
    }
#if defined(VISP_HAVE_DISPLAY)
    else if ((std::string(argv[i]) == "--display") || (std::string(argv[i]) == "-d")) {
      opt_display_tag = true;
    }
#endif
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i);
      return EXIT_FAILURE;
    }
  }

  std::cout << "Create marker" << std::endl;
  std::cout << "  Family: " << opt_tag_family << std::endl;
  std::cout << "  Id    : " << opt_tag_id << std::endl;
  std::cout << "  Size  : " << opt_pix_size << " x " << opt_pix_size << " pixels" << std::endl;

  vpDetectorAprilTag detector(opt_tag_family);

  vpImage<unsigned char> Ismall, I(opt_pix_size, opt_pix_size);
  if (detector.getTagImage(Ismall, opt_tag_id)) {
    vpImageTools::resize(Ismall, I, vpImageTools::INTERPOLATION_NEAREST);

    std::cout << "Saved in: " << opt_filename << std::endl;
    vpImageIo::write(I, opt_filename);

    if (opt_display_tag) {
      vpDisplay *display = vpDisplayFactory::allocateDisplay(I, -1, -1, "Created marker");
      vpDisplay::display(I);
      vpDisplay::flush(I);
      std::cout << "Click in the image to quit..." << std::endl;
      vpDisplay::getClick(I);
      delete display;
    }
  }

  return EXIT_SUCCESS;

#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
