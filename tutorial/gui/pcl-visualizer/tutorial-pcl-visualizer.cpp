//! \example tutorial-pcl-visualizer.cpp
#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL)
#include "ClassUsingPclVisualizer.h"

typedef enum DisplayMode
{
  BLOCKING   = 0,
  THREADED   = 1,
  BOTH       = 2,
  MODE_COUNT = 3
}DisplayMode;

std::string displayModeToString(const DisplayMode& mode)
{
  switch(mode)
  {
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

DisplayMode displayModeFromString(const std::string &name)
{
  DisplayMode res = DisplayMode::MODE_COUNT;
  bool wasFound = false;
  std::string lowerCaseName = name; // vpIoTools::toLowerCase(name);
  for(unsigned int i = 0; i < DisplayMode::MODE_COUNT && !wasFound; i++)
  {
    DisplayMode candidate = (DisplayMode) i;
    if(lowerCaseName == lowerCaseName)
    {
      res = candidate;
      wasFound = true; 
    }
  }
  return res;
}

std::string getAvailableDisplayMode(const std::string &prefix = "< ", const std::string &sep = " , ", const std::string &suffix = " >" )
{
  std::string modes(prefix);
  for(unsigned int i = 0; i < DisplayMode::MODE_COUNT - 1; i++)
  {
    DisplayMode candidate = (DisplayMode) i;
    modes += displayModeToString(candidate) + sep;
  }
  DisplayMode candidate = (DisplayMode) (DisplayMode::MODE_COUNT - 1);
  modes += displayModeToString(candidate) + suffix;
  return modes;
}


int main (int argc, char *argv[])
{
  const bool def_addNoise                              = false;
  const unsigned int def_order                         = 2;
  const std::pair<double, double> def_xlim             = std::pair<double, double>(-2.5,2.5);
  const std::pair<double, double> def_ylim             = std::pair<double, double>(-2.5,2.5);
  const std::pair<unsigned int, unsigned int> def_reso = std::pair<unsigned int, unsigned int>(50,50);
  const DisplayMode def_mode                           = DisplayMode::THREADED;  

  bool opt_addNoise                              = def_addNoise;
  unsigned int opt_order                         = def_order;
  std::pair<double, double> opt_xlim             = def_xlim;
  std::pair<double, double> opt_ylim             = def_ylim;
  std::pair<unsigned int, unsigned int> opt_reso = def_reso;
  DisplayMode opt_mode                           = def_mode;

  for(unsigned int i = 1; i < argc; i++)
  {
    if (std::string(argv[i]) == "--no-noise")
    {
      opt_addNoise = false;
    }
    else if (std::string(argv[i]) == "--add-noise")
    {
      opt_addNoise = true;
    }
    else if (std::string(argv[i]) == "--order" && i + 1 < argc)
    {
      opt_order = atoi(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--x-lim" && i + 2 < argc)
    {
      opt_xlim.first = atof(argv[i + 1]);
      opt_xlim.second = atof(argv[i + 2]);
      i+=2;
    }
    else if (std::string(argv[i]) == "--y-lim" && i + 2 < argc)
    {
      opt_ylim.first = atof(argv[i + 1]);
      opt_ylim.second = atof(argv[i + 2]);
      i+=2;
    }
    else if (std::string(argv[i]) == "--reso" && i + 2 < argc)
    {
      opt_reso.first = atoi(argv[i + 1]);
      opt_reso.second = atoi(argv[i + 2]);
      i+=2;
    }
    else if (std::string(argv[i]) == "--display-mode" && i + 1 < argc)
    {
      opt_mode = displayModeFromString(std::string(argv[i + 1]));
      i++;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
    {
      std::cout << "NAME" << std::endl;
      std::cout << "\t" << argv[0] << "  Test programm for the PCL-based point-cloud visualizer." << std::endl
                << std::endl;
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "\t" << argv[0]
                << "\t[--no-noise] || [--add-noise] (default: " + (def_addNoise ? std::string("true") : std::string("false")) << ")\n"
                << "\t[--order <surface-order>](default: " + std::to_string(def_order) << ")\n"
                << "\t[--x-lim <xmin xmax>](default: [" + std::to_string(def_xlim.first) + ";" + std::to_string(def_xlim.second) << "])\n"
                << "\t[--y-lim <ymin ymax>](default: [" + std::to_string(def_ylim.first) + ";" + std::to_string(def_ylim.second) << "])\n"
                << "\t[--reso <x_resolution y_resolution>](default: [" + std::to_string(def_reso.first) + ";" + std::to_string(def_reso.second) << "])\n"
                << "\t[--display-mode " << getAvailableDisplayMode() <<"](default: " << displayModeToString(def_mode) << ")\n"
                << "\t[--help] [-h]" << std::endl
                << std::endl;

      return EXIT_SUCCESS;
    }
  }
  ClassUsingPclVisualizer demo(opt_xlim, opt_ylim, opt_reso);

  if(opt_mode == DisplayMode::BLOCKING || opt_mode == DisplayMode::BOTH)
  {
    demo.blockingMode(opt_addNoise, opt_order);
  }

  if(opt_mode == DisplayMode::THREADED || opt_mode == DisplayMode::BOTH)
  {
    demo.threadedMode(opt_addNoise, opt_order);
  }
  return 0;
}
#else

int main(int argc, char *argv[])
{
  std::cerr << "ERROR: ViSP seems to have been compiled without PCL library" << std::endl;
  std::cerr << "\tPlease install PCL library and recompile ViSP." << std::endl;
}
#endif