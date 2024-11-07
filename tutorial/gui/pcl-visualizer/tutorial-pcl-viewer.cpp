//! \example tutorial-pcl-viewer.cpp
#include <visp3/core/vpConfig.h>

// System include
#include <iostream>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION)

// ViSP include
#include <visp3/core/vpIoTools.h>

//! [Class include]
// Tutorial include
#include "ClassUsingPclViewer.h"
//! [Class include]

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

//! [Enum for mode choice]
/**
 * @brief Enumeration permitting to choose between running the blocking-mode display
 * example, the threaded-mode or both mode consecutively.
 */
typedef enum DisplayMode
{
  BLOCKING = 0, /*!< Only the blocking-mode display example will be run.*/
  THREADED = 1, /*!< Only the threaded-mode display example will be run.*/
  BOTH = 2, /*!< First the blocking-mode display example will be run and then the threaded-mode one.*/
  MODE_COUNT = 3
} DisplayMode;

/**
 * @brief Cast a \b DisplayMode enum value into a \b std::stirng.
 *
 * @param mode The display mode we want to cast into a string.
 * @return std::string The name of the \b DisplayMode enum value.
 */
std::string displayModeToString(const DisplayMode &mode)
{
  switch (mode) {
  case BLOCKING:
    return "blocking";
  case THREADED:
    return "threaded";
  case BOTH:
    return "both";
  default:
    break;
  }
  return "unknown";
}

/**
 * @brief Cast a string into a \b DisplayMode enum value.
 * If \b name is not found, return \b DisplayMode::MODE_COUNT .
 *
 * @param name The name of the display mode.
 * @return DisplayMode The corresponding \b DisplayMode enum value, or \b DisplayMode::MODE_COUNT if not found.
 */
DisplayMode displayModeFromString(const std::string &name)
{
  DisplayMode res = DisplayMode::MODE_COUNT;
  bool wasFound = false;
  std::string lowerCaseName = vpIoTools::toLowerCase(name);
  for (unsigned int i = 0; i < DisplayMode::MODE_COUNT && !wasFound; i++) {
    DisplayMode candidate = (DisplayMode)i;
    if (lowerCaseName == displayModeToString(candidate)) {
      res = candidate;
      wasFound = true;
    }
  }
  return res;
}

/**
 * @brief Create a string that lists the different \b DisplayMode available.
 *
 * @param prefix The string that must prefix the list of modes.
 * @param sep The separator between the different modes.
 * @param suffix The string that must suffix the list of modes.
 * @return std::string The list containing the different modes.
 */
std::string getAvailableDisplayMode(const std::string &prefix = "< ", const std::string &sep = " , ", const std::string &suffix = " >")
{
  std::string modes(prefix);
  for (unsigned int i = 0; i < DisplayMode::MODE_COUNT - 1; i++) {
    DisplayMode candidate = (DisplayMode)i;
    modes += displayModeToString(candidate) + sep;
  }
  DisplayMode candidate = (DisplayMode)(DisplayMode::MODE_COUNT - 1);
  modes += displayModeToString(candidate) + suffix;
  return modes;
}
//! [Enum for mode choice]

int main(int argc, char *argv[])
{
  //! [Default arguments values]
  const double def_addedNoise = 0.; // Standard deviation of the noise added to the points.
  const unsigned int def_order = 2;  // Order of the polynomial surface used for the example.
  const std::pair<double, double> def_xlim = std::pair<double, double>(-2.5, 2.5); // Min and max X-axis coordinates.
  const std::pair<double, double> def_ylim = std::pair<double, double>(-2.5, 2.5); // Min and max Y-axis coordinates.
  const std::pair<unsigned int, unsigned int> def_reso = std::pair<unsigned int, unsigned int>(50, 50); // Number of points along the X-axis and Y-axis reciprocally.
  const DisplayMode def_mode = DisplayMode::BLOCKING; // Display mode that should be used.
  //! [Default arguments values]

  //! [Arguments parser]
  double opt_addedNoise = def_addedNoise;
  unsigned int opt_order = def_order;
  std::pair<double, double> opt_xlim = def_xlim;
  std::pair<double, double> opt_ylim = def_ylim;
  std::pair<unsigned int, unsigned int> opt_reso = def_reso;
  DisplayMode opt_mode = def_mode;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--noise" && i + 1 < argc) {
      opt_addedNoise = atof(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--order" && i + 1 < argc) {
      opt_order = atoi(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--x-lim" && i + 2 < argc) {
      opt_xlim.first = atof(argv[i + 1]);
      opt_xlim.second = atof(argv[i + 2]);
      i += 2;
    }
    else if (std::string(argv[i]) == "--y-lim" && i + 2 < argc) {
      opt_ylim.first = atof(argv[i + 1]);
      opt_ylim.second = atof(argv[i + 2]);
      i += 2;
    }
    else if (std::string(argv[i]) == "--reso" && i + 2 < argc) {
      opt_reso.first = atoi(argv[i + 1]);
      opt_reso.second = atoi(argv[i + 2]);
      i += 2;
    }
    else if (std::string(argv[i]) == "--display-mode" && i + 1 < argc) {
      opt_mode = displayModeFromString(std::string(argv[i + 1]));
      i++;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      //! [Arguments of the program]
      std::cout << "NAME" << std::endl;
      std::cout << "\t" << argv[0] << "  Test programm for the PCL-based point-cloud visualizer." << std::endl
        << std::endl;
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "\t" << argv[0]
        << "\t[--noise <stdev_noise>] (default: " + std::to_string(def_addedNoise) << ")\n"
        << "\t[--order <surface-order>](default: " + std::to_string(def_order) << ")\n"
        << "\t[--x-lim <xmin xmax>](default: [" + std::to_string(def_xlim.first) + ";" + std::to_string(def_xlim.second) << "])\n"
        << "\t[--y-lim <ymin ymax>](default: [" + std::to_string(def_ylim.first) + ";" + std::to_string(def_ylim.second) << "])\n"
        << "\t[--reso <x_resolution y_resolution>](default: [" + std::to_string(def_reso.first) + ";" + std::to_string(def_reso.second) << "])\n"
        << "\t[--display-mode " << getAvailableDisplayMode() << "](default: " << displayModeToString(def_mode) << ")\n"
        << "\t[--help] [-h]" << std::endl
        << std::endl;
      //! [Arguments of the program]
      return EXIT_SUCCESS;
    }
  }
  //! [Arguments parser]

  std::cout << "Parameters:" << std::endl;
  std::cout << "\tSurface order: " << opt_order << std::endl;
  std::cout << "\tX-axis limits: [" << opt_xlim.first << " ; " << opt_xlim.first << "]" << std::endl;
  std::cout << "\tY-axis limits: [" << opt_ylim.first << " ; " << opt_ylim.first << "]" << std::endl;
  std::cout << "\tGrid resolution: [" << opt_reso.first << " x " << opt_reso.first << "]" << std::endl;
  std::cout << "\tNoise standard deviation: " << opt_addedNoise << std::endl;
  std::cout << "\tDisplay mode: " << displayModeToString(opt_mode) << std::endl;

  //! [Running blocking mode]
  if (opt_mode == DisplayMode::BLOCKING || opt_mode == DisplayMode::BOTH) {
    ClassUsingPclViewer demo(opt_xlim, opt_ylim, opt_reso);
    demo.blockingMode(opt_addedNoise, opt_order);
  }
  //! [Running blocking mode]

  //! [Running threaded mode]
  if (opt_mode == DisplayMode::THREADED || opt_mode == DisplayMode::BOTH) {
    ClassUsingPclViewer demo(opt_xlim, opt_ylim, opt_reso);
    demo.threadedMode(opt_addedNoise, opt_order);
  }
  //! [Running threaded mode]

  return 0;
}
#else

int main()
{
  std::cout << "ViSP seems to have been compiled without PCL visualization module." << std::endl;
  std::cout << "Please install PCL visualization module and recompile ViSP." << std::endl;
  return EXIT_SUCCESS;
}
#endif
