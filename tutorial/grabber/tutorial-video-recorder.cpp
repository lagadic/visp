/*! \example tutorial-video-recorder.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpVideoWriter.h>
#include <visp/vpV4l2Grabber.h>

/*!
 This example allows to record a video from a camera.
 It only requires that ViSP is build with OpenCV.

 Example: ./tutorial-video-recorder --device 0 --name myvideo.mpeg
 */
int main(int argc, const char *argv[])
{
#if ((defined(VISP_HAVE_V4L2) || (VISP_HAVE_OPENCV_VERSION >= 0x020100)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)))
  std::string opt_videoname = "video-recorded.mpg";
  int opt_device = 0;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--device")
      opt_device = atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--name")
      opt_videoname = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--device <device number>] [--name <video name>] [--help]\n" << std::endl;
      return 0;
    }
  }

  std::cout << "Use device: " << opt_device << std::endl;
  std::cout << "Record video in: " << opt_videoname << std::endl;

  try {
    //vpImage<vpRGBa> I; // for color images
    vpImage<unsigned char> I; // for gray images

#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.open(I);
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    cv::VideoCapture g(opt_device);
    if(!g.isOpened()) { // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d;
#endif
    d.init(I, 0, 0, "Camera view");
    vpVideoWriter writer;

#ifdef VISP_HAVE_FFMPEG
    // Set up the bit rate
    writer.setBitRate(1000000);
    // Set up the codec to use
#  if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
    writer.setCodec(CODEC_ID_MPEG2VIDEO);
#  else
    writer.setCodec(AV_CODEC_ID_MPEG2VIDEO);
#  endif
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
    writer.setCodec( cv::VideoWriter::fourcc('P','I','M','1') ); // MPEG-1 codec
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    writer.setCodec( CV_FOURCC('P','I','M','1') );
#endif
    writer.setFileName(opt_videoname);
    writer.open(I);
    bool recording = false;

    for(;;) {
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      if (recording == false) {
        vpDisplay::displayText(I, 10, 10, "A click to start recording", vpColor::green);
        if (vpDisplay::getClick(I, false))
          recording = true;
      }
      else {
        writer.saveFrame(I);
        vpDisplay::displayText(I, 10, 10, "Recording: A click to stop and exit", vpColor::red);
        if (vpDisplay::getClick(I, false))
          break;
      }

      vpDisplay::flush(I);
    }
    std::cout << "The video was recorded in \"" << opt_videoname << "\"" << std::endl;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
