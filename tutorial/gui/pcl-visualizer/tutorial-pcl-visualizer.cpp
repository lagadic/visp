//! \example tutorial-pcl-visualizer.cpp
#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL)
#include "ClassUsingPclVisualizer.h"

int main (int argc, char *argv[])
{
  const bool def_addNoise = false;
  const unsigned int def_order = 2;
  const std::pair<double, double> def_xlim             = std::pair<double, double>(-2.5,2.5);
  const std::pair<double, double> def_ylim             = std::pair<double, double>(-2.5,2.5);
  const std::pair<unsigned int, unsigned int> def_reso = std::pair<unsigned int, unsigned int>(50,50);
  

  bool opt_addNoise = def_addNoise;
  unsigned int opt_order = def_order;
  std::pair<double, double> opt_xlim             = def_xlim;
  std::pair<double, double> opt_ylim             = def_ylim;
  std::pair<unsigned int, unsigned int> opt_reso = def_reso;
  unsigned int opt_nbIn = 5;
  unsigned int opt_nbOut = 2;

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
    
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
    {
      std::cout << "NAME" << std::endl;
      std::cout << "\t" << argv[0] << "  Test programm for the PCL-based point-cloud visualizer." << std::endl
                << std::endl;
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "\t" << argv[0]
                << " [--no-noise] || [--add-noise] (default: " + (def_addNoise ? std::string("true") : std::string("false")) << ")\n"
                << " [--order <surface-order>](default: " + std::to_string(def_order) << ")\n"
                << " [--x-lim <xmin xmax>](default: [" + std::to_string(def_xlim.first) + ";" + std::to_string(def_xlim.second) << "])\n"
                << " [--y-lim <ymin ymax>](default: [" + std::to_string(def_ylim.first) + ";" + std::to_string(def_ylim.second) << "])\n"
                << " [--reso <x_resolution y_resolution>](default: [" + std::to_string(def_reso.first) + ";" + std::to_string(def_reso.second) << "])\n"
                << " [--help] [-h]" << std::endl
                << std::endl;

      return EXIT_SUCCESS;
    }
  }
  ClassUsingPclVisualizer demo(opt_xlim, opt_ylim, opt_reso);
  demo.threadedMode(opt_addNoise, opt_order);
  return 0;
}
#else

int main(int argc, char *argv[])
{
  std::cerr << "ERROR: ViSP seems to have been compiled without PCL library" << std::endl;
  std::cerr << "\tPlease install PCL library and recompile ViSP." << std::endl;
}
#endif