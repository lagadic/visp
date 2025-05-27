#include <visp3/rbt/vpRBInitializationHelper.h>
#include <visp3/rbt/vpRBTracker.h>

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>

#ifdef VISP_HAVE_MODULE_GUI
#include <visp3/gui/vpDisplayFactory.h>
#endif

BEGIN_VISP_NAMESPACE

void vpRBInitializationHelper::removeComment(std::ifstream &fileId)
{
  char c;

  fileId.get(c);
  while (!fileId.fail() && (c == '#')) {
    fileId.ignore(std::numeric_limits<std::streamsize>::max(), fileId.widen('\n'));
    fileId.get(c);
  }
  if (fileId.fail()) {
    throw(vpException(vpException::ioError, "Reached end of file"));
  }
  fileId.unget();
}

void vpRBInitializationHelper::savePose(const std::string &filename) const
{
  vpPoseVector init_pos;
  std::fstream finitpos;
  finitpos.open(filename.c_str(), std::ios::out);

  init_pos.buildFrom(m_cMo);
  finitpos << init_pos;
  finitpos.close();
}

#ifdef VISP_HAVE_MODULE_GUI

template <typename T>
void vpRBInitializationHelper::initClick(const vpImage<T> &I, const std::string &initFile, bool displayHelp, vpRBTracker &tracker)
{
  std::cout << "Starting init click!" << std::endl;
  vpHomogeneousMatrix last_cMo;
  vpPoseVector init_pos;
  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

  std::string ext = ".init";
  std::string str_pose = "";
  size_t pos = initFile.rfind(ext);

  // Load the last poses from files
  std::fstream finitpos;
  std::ifstream finit;
  std::stringstream ss;
  std::string poseSavingFilename;
  if (poseSavingFilename.empty()) {
    if (pos != std::string::npos)
      str_pose = initFile.substr(0, pos) + ".0.pos";
    else
      str_pose = initFile + ".0.pos";

    finitpos.open(str_pose.c_str(), std::ios::in);
    ss << str_pose;
  }
  else {
    finitpos.open(poseSavingFilename.c_str(), std::ios::in);
    ss << poseSavingFilename;
  }
  if (finitpos.fail()) {
    std::cout << "Cannot read " << ss.str() << std::endl << "cMo set to identity" << std::endl;
    last_cMo.eye();
  }
  else {
    for (unsigned int i = 0; i < 6; i += 1) {
      finitpos >> init_pos[i];
    }

    finitpos.close();
    last_cMo.buildFrom(init_pos);

    std::cout << "Tracker initial pose read from " << ss.str() << ": " << std::endl << last_cMo << std::endl;

    vpDisplay::display(I);
    vpDisplay::displayFrame(I, last_cMo, m_cam, 0.05, vpColor::green);
    vpDisplay::flush(I);


    std::cout << "No modification : left click " << std::endl;
    std::cout << "Modify initial pose : right click " << std::endl;


    vpDisplay::displayText(I, 15, 10, "left click to validate, right click to modify initial pose", vpColor::red);

    vpDisplay::flush(I);

    while (!vpDisplay::getClick(I, ip, button)) {
      vpTime::sleepMs(10);
    }
  }

  if (!finitpos.fail() && button == vpMouseButton::button1) {
    m_cMo = last_cMo;
  }
  else {
    vpDisplay *d_help = nullptr;

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpPose pose;

    pose.clearPoint();

    // Clear string stream that previously contained the path to the "object.0.pos" file.
    ss.str(std::string());

    // file parser
    // number of points
    // X Y Z
    // X Y Z
    if (pos != std::string::npos) {
      ss << initFile;
    }
    else {
      ss << initFile;
      ss << ".init";
    }

    std::cout << "Load 3D points from: " << ss.str() << std::endl;
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
    finit.open(ss.str());
#else
    finit.open(ss.str().c_str());
#endif
    if (finit.fail()) {
      std::cout << "Cannot read " << ss.str() << std::endl;
      throw vpException(vpException::ioError, "Cannot open model-based tracker init file %s", ss.str().c_str());
    }

#ifdef VISP_HAVE_MODULE_IO
    // Display window creation and initialisation
    try {
      if (displayHelp) {
        const std::string imgExtVec[] = { ".ppm", ".pgm", ".jpg", ".jpeg", ".png" };
        std::string dispF;
        bool foundHelpImg = false;
        if (pos != std::string::npos) {
          for (size_t i = 0; i < 5 && !foundHelpImg; i++) {
            dispF = initFile.substr(0, pos) + imgExtVec[i];
            foundHelpImg = vpIoTools::checkFilename(dispF);
          }
        }
        else {
          for (size_t i = 0; i < 5 && !foundHelpImg; i++) {
            dispF = initFile + imgExtVec[i];
            foundHelpImg = vpIoTools::checkFilename(dispF);
          }
        }

        if (foundHelpImg) {
          std::cout << "Load image to help initialization: " << dispF << std::endl;

          d_help = vpDisplayFactory::allocateDisplay();

          vpImage<vpRGBa> Iref;
          vpImageIo::read(Iref, dispF);
#if defined(VISP_HAVE_DISPLAY)
          const int winXPos = I.display->getWindowXPosition();
          const int winYPos = I.display->getWindowYPosition();
          unsigned int width = I.getWidth();
          d_help->init(Iref, winXPos +static_cast<int>(width) + 80, winYPos, "Where to initialize...");
          vpDisplay::display(Iref);
          vpDisplay::flush(Iref);
#endif
        }
      }
    }
    catch (...) {
      if (d_help != nullptr) {
        delete d_help;
        d_help = nullptr;
      }
    }
#else  //#ifdef VISP_HAVE_MODULE_IO
    (void)(displayHelp);
#endif //#ifdef VISP_HAVE_MODULE_IO
    // skip lines starting with # as comment
    removeComment(finit);

    unsigned int n3d;
    finit >> n3d;
    finit.ignore(256, '\n'); // skip the rest of the line
    std::cout << "Number of 3D points  " << n3d << std::endl;
    if (n3d > 100000) {
      throw vpException(vpException::badValue, "In %s file, the number of 3D points exceed the max allowed",
                        ss.str().c_str());
    }

    std::vector<vpPoint> P(n3d);
    for (unsigned int i = 0; i < n3d; i++) {
      // skip lines starting with # as comment
      removeComment(finit);

      vpColVector pt_3d(4, 1.0);
      finit >> pt_3d[0];
      finit >> pt_3d[1];
      finit >> pt_3d[2];
      finit.ignore(256, '\n'); // skip the rest of the line

      vpColVector pt_3d_tf = pt_3d;
      std::cout << "Point " << i + 1 << " with 3D coordinates: " << pt_3d_tf[0] << " " << pt_3d_tf[1] << " "
        << pt_3d_tf[2] << std::endl;

      P[i].setWorldCoordinates(pt_3d_tf[0], pt_3d_tf[1], pt_3d_tf[2]); // (X,Y,Z)
    }

    finit.close();

    bool isWellInit = false;
    while (!isWellInit) {
      std::vector<vpImagePoint> mem_ip;
      for (unsigned int i = 0; i < n3d; i++) {
        std::ostringstream text;
        text << "Click on point " << i + 1;

        vpDisplay::display(I);
        vpDisplay::displayText(I, 15, 10, text.str(), vpColor::red);
        for (unsigned int k = 0; k < mem_ip.size(); k++) {
          vpDisplay::displayCross(I, mem_ip[k], 10, vpColor::green, 2);
        }
        vpDisplay::flush(I);

        std::cout << "Click on point " << i + 1 << " ";
        double x = 0, y = 0;

        vpDisplay::getClick(I, ip);
        mem_ip.push_back(ip);
        vpDisplay::flush(I);

        vpPixelMeterConversion::convertPoint(m_cam, ip, x, y);
        P[i].set_x(x);
        P[i].set_y(y);

        std::cout << "with 2D coordinates: " << ip << std::endl;

        pose.addPoint(P[i]); // and added to the pose computation point list
      }
      vpDisplay::flush(I);
      vpDisplay::display(I);

      std::cout << "Before optim: " << m_cam << std::endl;
      if (!pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, m_cMo)) {
        std::cout << "Pose computation from points failed!" << std::endl;
        for (unsigned int i = 0; i < n3d; ++i) {
          std::cout << "Point " << i << ": " << std::endl;
          std::cout << " 3D: " << pose.getPoints()[i].get_oP().t() << std::endl;
          std::cout << "2D: " << pose.getPoints()[i].get_x() << ", " << pose.getPoints()[i].get_y() << std::endl;
        }
      }

      vpRBFeatureTrackerInput frame;
      tracker.updateRender(frame, m_cMo);
      tracker.displaySilhouette(I, frame);

      vpDisplay::displayText(I, 15, 10, "left click to validate, right click to re initialize object", vpColor::red);

      vpDisplay::flush(I);

      button = vpMouseButton::button1;
      while (!vpDisplay::getClick(I, ip, button)) {
      }

      if (button == vpMouseButton::button1) {
        isWellInit = true;
      }
      else {
        pose.clearPoint();
        vpDisplay::display(I);
        vpDisplay::flush(I);
      }

    }
    vpDisplay::displayFrame(I, m_cMo, m_cam, 0.05, vpColor::red);

    // save the pose into file
    if (poseSavingFilename.empty())
      savePose(str_pose);
    else
      savePose(poseSavingFilename);

    if (d_help != nullptr) {
      delete d_help;
      d_help = nullptr;
    }
  }

}
template VISP_EXPORT void vpRBInitializationHelper::initClick<unsigned char>(const vpImage<unsigned char> &I, const std::string &initFile, bool displayHelp, vpRBTracker &tracker);
template VISP_EXPORT void vpRBInitializationHelper::initClick<vpRGBa>(const vpImage<vpRGBa> &I, const std::string &initFile, bool displayHelp, vpRBTracker &tracker);
#endif

END_VISP_NAMESPACE
