//! \example tutorial-video-manipulation.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpVideoWriter.h>

void usage(const char *argv[], int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0] << " [--in <video name>]"
    << " [--display-fps <framerate>]"
    << " [--out <video name>]"
    << " [--out-first-frame <index>]"
    << " [--out-gray]"
    << " [--out-stride <value>]"
    << " [--verbose] [-v]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  --in <video name> " << std::endl
    << "    Input video to manipulate. " << std::endl
    << "    Supported image formats: pgm,ppm,jpg,jpeg,png,tiff,bmp,ras,jp2" << std::endl
    << "    Example: " << std::endl
    << "    - I%03d.jpg : to read a sequence of images (I008.jpg, I009.jpg, I010.jpg) " << std::endl
    << std::endl
    << "  --display-fps <framerate>" << std::endl
    << "    Framerate used to display the video. When set to -1 display video as fast as possible." << std::endl
    << "    Default: 30 (fps)" << std::endl
    << std::endl
    << "  --out <video name>" << std::endl
    << "    Renamed video." << std::endl
    << std::endl
    << "  --out-first-frame <index>" << std::endl
    << "    Renamed video first image index." << std::endl
    << "    When set to -1, use same image numbering as input video." << std::endl
    << "    Default: -1" << std::endl
    << std::endl
    << "  --out-gray" << std::endl
    << "    Associated to --out option, convert input images to Y8 gray level image." << std::endl
    << std::endl
    << "  --out-stride <value>" << std::endl
    << "    Associated to --out option, allows to subsample the resulting output video" << std::endl
    << "    keeping one over <value> images. For example, when set to 2, the ouput video" << std::endl
    << "    has two times less images than the input video." << std::endl
    << "    Default: 1." << std::endl
    << std::endl
    << "  --select, -s" << std::endl
    << "    Associated to --out option, allows the user to select by mouse" << std::endl
    << "    click which images will be saved in the output video." << std::endl
    << std::endl
    << "  --verbose, -v" << std::endl
    << "    Display extra messages." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  if (argc == 1) {
    usage(argv, 0);
    return EXIT_SUCCESS;
  }

  std::string opt_video_in = "";
  double opt_display_fps = 30;
  std::string opt_video_out = "";
  int opt_video_out_first_frame = -1;
  bool opt_video_out_gray = false;
  int opt_video_out_stride = 1;

  bool opt_verbose = false;
  bool opt_select_frame = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--in" && i + 1 < argc) {
      opt_video_in = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--display-fps" && i + 1 < argc) {
      opt_display_fps = std::atof(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--out" && i + 1 < argc) {
      opt_video_out = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--out-first-frame" && i + 1 < argc) {
      opt_video_out_first_frame = std::atoi(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--out-gray") {
      opt_video_out_gray = true;
    }
    else if (std::string(argv[i]) == "--out-stride" && i + 1 < argc) {
      opt_video_out_stride = std::atoi(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--select" || std::string(argv[i]) == "-s") {
      opt_select_frame = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i);
      return EXIT_FAILURE;
    }
  }

  // Option ckeck
  if (opt_video_in.empty()) {
    usage(argv, 0);
    std::cout << "Error: " << std::endl << "  No input video set using --in <video name> option." << std::endl;
    return EXIT_FAILURE;
  }
  if (opt_select_frame && opt_video_out.empty()) {
    usage(argv, 0);
    std::cout << "Error: " << std::endl
      << "  --select option is enabled but no output video name is specified using --out <video> option."
      << std::endl;
    return EXIT_FAILURE;
  }
  if (opt_video_out_gray && opt_video_out.empty()) {
    usage(argv, 0);
    std::cout << "Error: " << std::endl
      << "  --out-gray option is enabled but no output video name is specified using --out <video> option."
      << std::endl;
    return EXIT_FAILURE;
  }
  if ((opt_video_out_stride > 1) && opt_video_out.empty()) {
    usage(argv, 0);
    std::cout << "Error: " << std::endl
      << "  --out-stride option is enabled but no output video name is specified using --out <video> option."
      << std::endl;
    return EXIT_FAILURE;
  }
  if ((opt_video_out_stride > 1) && opt_select_frame && !opt_video_out.empty()) {
    usage(argv, 0);
    std::cout << "Error: " << std::endl
      << "  --out-stride option is enabled but this option doesn't make sense with --select option."
      << std::endl;
    return EXIT_FAILURE;
  }

  vpImage<vpRGBa> I;
  vpImage<unsigned char> Igray;
  vpVideoReader g;
  g.setFileName(opt_video_in);
  g.open(I);
  std::cout << "Input video" << std::endl;
  std::cout << "  Video name     : " << opt_video_in << std::endl;
  std::cout << "  Video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;
  std::cout << "  First image    : " << g.getFirstFrameIndex() << std::endl;
  std::cout << "  Last image     : " << g.getLastFrameIndex() << std::endl;
  std::cout << "  Framerate (fps): " << opt_display_fps << std::endl;

  vpVideoWriter writer;
  if (!opt_video_out.empty()) {
    writer.setFileName(opt_video_out);
    int first_frame =
      (opt_video_out_first_frame < 0 ? static_cast<int>(g.getFirstFrameIndex()) : opt_video_out_first_frame);
    writer.setFirstFrameIndex(first_frame);
    if (opt_video_out_gray) {
      vpImageConvert::convert(I, Igray);
      writer.open(Igray);
    }
    else {
      writer.open(I);
    }
    std::cout << "Output video" << std::endl;
    std::cout << "  Video name     : " << opt_video_out << std::endl;
    std::cout << "  First image    : " << first_frame << std::endl;
    std::cout << "  Stride         : " << opt_video_out_stride << std::endl;
    std::cout << "  Y8 gray images : " << (opt_video_out_gray ? "yes" : "no (same as input)") << std::endl;
  }

  std::cout << "Other settings" << std::endl;
  std::cout << "  Verbose        : " << (opt_verbose ? "enabled" : "disabled") << std::endl;
  std::cout << "  Select frames  : " << (opt_select_frame ? "enabled" : "disabled") << std::endl;

  try {
    //! [vpDisplay construction]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D d(I, vpDisplay::SCALE_AUTO);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    int scale = vpDisplay::getDownScalingFactor(I);
    int cpt_stride = 1;

    while (!g.end()) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);

      std::stringstream ss;
      ss << "Image " << g.getFrameIndex();
      vpDisplay::setTitle(I, ss.str());
      vpDisplay::display(I);
      vpDisplay::displayText(I, 15 * scale, 15 * scale, "Right click to quit", vpColor::red);
      if (opt_select_frame) {
        vpDisplay::displayText(I, 30 * scale, 15 * scale, "Left click to select frame", vpColor::red);
      }
      vpDisplay::flush(I);

      bool selected_frame = (opt_select_frame ? false : true);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1 && opt_select_frame) {
          selected_frame = true;
        }
        else if (button == vpMouseButton::button3) {
          break;
        }
      }

      if (!opt_video_out.empty() && selected_frame) {
        if (cpt_stride == opt_video_out_stride) {
          cpt_stride = 1;
          if (opt_video_out_gray) {
            vpImageConvert::convert(I, Igray);
            writer.saveFrame(Igray);
          }
          else {
            writer.saveFrame(I);
          }
          if (opt_verbose) {
            std::cout << "Save " << writer.getFrameName() << std::endl;
          }
        }
        else {
          cpt_stride++;
        }
      }

      if (opt_display_fps > 0) {
        vpTime::wait(t, 1000. / opt_display_fps);
      }
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
