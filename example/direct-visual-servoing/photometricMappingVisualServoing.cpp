/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*!
  \example photometricMappingVisualServoing.cpp

  Implemented from \cite Collewet08c, \cite Marchand19a and \cite Marchand20a.
*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpUniRand.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/robot/vpImageSimulator.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/visual_features/vpFeatureLuminanceMapping.h>

#include <stdlib.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif


// List of allowed command line options
#define GETOPTARGS "cdi:n:p:m:k:hl:"

void usage(const char *name, const char *badparam, std::string ipath, int niter, const std::string &method, unsigned numDbImages, const unsigned numComponents, const double lambda);
bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display, int &niter, std::string &method, unsigned &numDbImages, unsigned &numComponents, double &lambda);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param niter : Number of iterations.

*/
void usage(const char *name, const char *badparam, std::string ipath, int niter, const std::string &method, unsigned numDbImages, const unsigned numComponents, const double lambda)
{
  fprintf(stdout, "\n\
Visual servoing with compressed photometric features.\n\
Use either PCA or DCT representations\n\
\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-m pca|dct] [-p <v>] [-c] [-d] [-n <number of iterations>] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"doisneau/doisneau.jpg\"\n\
     images. \n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
  \n\
  -m\n\
     Method to use: either 'PCA' or 'DCT'\n\
     PCA first requires learning a projection from a base of images. see the -p option.\n\
     Default: %s\n\
  -k\n\
     Number of visual servoing features (i.e., PCA or DCT components)\n\
     Default: %d\n\
\n\
  -p\n\
     Number of images to use to compute PCA. If method is DCT, this option is ignored.\n\
     Default: %d\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -n %%d                                               %d\n\
     Number of visual servoing iterations.\n\
\n\
  -l %%f                                               %f\n\
     Number of visual servoing iterations.\n\
\n\
  -h\n\
     Print the help.\n",
          ipath.c_str(), method.c_str(), numComponents, numDbImages, niter, lambda);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \param niter : Number of iterations.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display,
                 int &niter, std::string &method, unsigned &numDbImages, unsigned &numComponents, double &lambda)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'i':
      ipath = optarg_;
      break;
    case 'm':
      method = std::string(optarg_);
      break;
    case 'p':
      numDbImages = atoi(optarg_);
      break;
    case 'k':
      numComponents = atoi(optarg_);
      break;
    case 'n':
      niter = atoi(optarg_);
      break;
    case 'l':
      lambda = atof(optarg_);
      break;
    case 'h':
      usage(argv[0], nullptr, ipath, niter, method, numDbImages, numComponents, lambda);
      return false;

    default:
      usage(argv[0], optarg_, ipath, niter, method, numDbImages, numComponents, lambda);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, niter, method, numDbImages, numComponents, lambda);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV)) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;
    bool opt_click_allowed = true;
    bool opt_display = true;
    int opt_niter = 400;
    std::string opt_method = "dct";
    unsigned opt_numDbImages = 2000;
    unsigned opt_numComponents = 32;
    double opt_lambda = 5.0;

    double mu = 0.01; // mu = 0 : Gauss Newton ; mu != 0  : LM
    double lambdaGN = opt_lambda;



    const double Z = 0.8;
    const unsigned ih = 240;
    const unsigned iw = 320;
    const double scenew = 0.6;
    const double sceneh = 0.42;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display, opt_niter, opt_method,
                   opt_numDbImages, opt_numComponents, opt_lambda) == false) {
      return EXIT_FAILURE;
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path coming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
          << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
          << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], nullptr, ipath, opt_niter, opt_method, opt_numDbImages, opt_numComponents, opt_lambda);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << std::endl;
      return EXIT_FAILURE;
    }

    vpImage<unsigned char> Itexture;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpImageIo::read(Itexture, filename);

    vpColVector X[4];
    for (int i = 0; i < 4; i++)
      X[i].resize(3);
    // Top left corner
    X[0][0] = -(scenew / 2.0);
    X[0][1] = -(sceneh / 2.0);
    X[0][2] = 0;

    // Top right corner
    X[1][0] = (scenew / 2.0);
    X[1][1] = -(sceneh / 2.0);
    X[1][2] = 0;

    // Bottom right corner
    X[2][0] = (scenew / 2.0);
    X[2][1] = (sceneh / 2.0);
    X[2][2] = 0;

    // Bottom left corner
    X[3][0] = -(scenew / 2.0);
    X[3][1] = (sceneh / 2.0);
    X[3][2] = 0;

    vpImageSimulator sim;

    sim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    sim.setCleanPreviousImage(true, vpColor::black);
    sim.init(Itexture, X);
    // ----------------------------------------------------------
    // Create the framegraber (here a simulated image)
    vpImage<unsigned char> I(ih, iw, 0);
    vpImage<unsigned char> Irec(ih - vpFeatureLuminance::DEFAULT_BORDER * 2, iw - vpFeatureLuminance::DEFAULT_BORDER * 2, 0);

    vpImage<unsigned char> Id;
    // camera desired position
    vpHomogeneousMatrix cdMo;
    cdMo[2][3] = Z;


    vpCameraParameters cam(870, 870, 160, 120);
    std::shared_ptr<vpLuminanceMapping> sMapping = nullptr;
    std::shared_ptr<vpLuminanceMapping> sdMapping = nullptr;

    // Setup mapping
    if (opt_method == "pca") {
      vpUniRand random(17);
      std::cout << "Building image database for PCA computation with " << opt_numDbImages << " images" << std::endl;
#if defined(VISP_HAVE_GUI)
#if defined(VISP_HAVE_X11)
      vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d;
#elif defined(VISP_HAVE_GTK)
      vpDisplayGTK d;
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV d;
#endif
      if (opt_display) {
        d.init(I, 0, 0, "Image database (subsample)");
      }
#endif
      std::vector<vpImage<unsigned char>> images(opt_numDbImages);
      for (unsigned i = 0; i < opt_numDbImages; ++i) {
        vpColVector to(3, 0.0), positionNoise(3, 0.0);
        const double noiseDiv = 16.0;
        positionNoise[0] = random.uniform(-scenew / noiseDiv, scenew / noiseDiv);
        positionNoise[1] = random.uniform(-sceneh / noiseDiv, sceneh / noiseDiv);
        positionNoise[2] = random.uniform(0.0, Z / noiseDiv);
        const double noiseDivTo = 16.0;
        to[0] = random.uniform(-scenew / noiseDivTo, scenew / noiseDivTo);
        to[1] = random.uniform(-sceneh / noiseDivTo, sceneh / noiseDivTo);
        const vpColVector from = vpColVector(cdMo.getTranslationVector()) + positionNoise;
        vpRotationMatrix Rrot(0.0, 0.0, vpMath::rad(random.uniform(-10, 10)));
        vpHomogeneousMatrix dbMo = vpMath::lookAt(from, to, Rrot * vpColVector({ 0.0, 1.0, 0.0 }));
        sim.setCameraPosition(dbMo);
        sim.getImage(I, cam);
        images[i] = I;
        if (i % 20 == 0 && opt_display) {
          vpDisplay::display(I);
          vpDisplay::flush(I);
        }
      }
      std::cout << "Computing PCA, this may take some time!" << std::endl;
      // create two distinct objects: if the projection is stateful, using a single mapping could lead to undesired behaviour
      vpLuminancePCA pca = vpLuminancePCA::learn(images, opt_numComponents, vpFeatureLuminance::DEFAULT_BORDER);
      std::cout << "Explained variance: " << pca.getExplainedVariance().sum() * 100.0 << "%" << std::endl;
      sMapping = std::shared_ptr<vpLuminanceMapping>(new vpLuminancePCA(pca));
      sdMapping = std::shared_ptr<vpLuminanceMapping>(new vpLuminancePCA(pca));
    }
    else if (opt_method == "dct") {
      sMapping = std::shared_ptr<vpLuminanceMapping>(new vpLuminanceDCT(opt_numComponents));
      sdMapping = std::shared_ptr<vpLuminanceMapping>(new vpLuminanceDCT(opt_numComponents));
    }
    else {
      throw vpException(vpException::badValue, "Method must be pca or dct!");
    }

    // set the robot at the desired position
    sim.setCameraPosition(cdMo);
    sim.getImage(I, cam); // and aquire the image Id
    Id = I;
#if defined(VISP_HAVE_GUI)
    // display the image
#if defined(VISP_HAVE_X11)
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d;
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d;
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV)
    if (opt_display) {
      d.init(I, 20, 10, "Current image");
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    if (opt_display && opt_click_allowed) {
      std::cout << "Click in the image to continue..." << std::endl;
      vpDisplay::getClick(I);
    }
#endif
#endif

    // ----------------------------------------------------------
    // position the robot at the initial position
    // ----------------------------------------------------------

    // camera desired position
    vpHomogeneousMatrix cMo;
    cMo.build(0.0, 0, Z + 0.2, vpMath::rad(15), vpMath::rad(-5), vpMath::rad(5));
    vpHomogeneousMatrix wMo; // Set to identity
    vpHomogeneousMatrix wMc; // Camera position in the world frame

    // set the robot at the desired position
    sim.setCameraPosition(cMo);
    I = 0u;
    sim.getImage(I, cam); // and aquire the image Id

#if defined(VISP_HAVE_GUI) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK))
    if (opt_display) {
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    if (opt_display && opt_click_allowed) {
      std::cout << "Click in the image to continue..." << std::endl;
      vpDisplay::getClick(I);
    }
#endif

    vpImage<unsigned char> Idiff;
    Idiff = I;

    vpImageTools::imageDifference(I, Id, Idiff);

    // Display image difference
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
#if defined(VISP_HAVE_X11)
    vpDisplayX d1, d2;

#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d1, d2;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d1, d2;
#endif
    if (opt_display) {
      d1.init(Idiff, 40 + static_cast<int>(I.getWidth()), 10, "photometric visual servoing : s-s* ");
      d2.init(Irec, 40 + static_cast<int>(I.getWidth()) * 2, 10, "Reconstructed image");

      vpDisplay::display(Idiff);
      vpDisplay::flush(Idiff);
      vpDisplay::display(Irec);
      vpDisplay::flush(Irec);
    }
#endif
    // create the robot (here a simulated free flying camera)
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.04);
    wMc = wMo * cMo.inverse();
    robot.setPosition(wMc);

    // ------------------------------------------------------
    // Visual feature, interaction matrix, error
    // s, Ls, Lsd, Lt, Lp, etc
    // ------------------------------------------------------

    // current visual feature built from the image
    vpFeatureLuminance luminanceI;
    luminanceI.init(I.getHeight(), I.getWidth(), Z);
    luminanceI.setCameraParameters(cam);
    vpFeatureLuminanceMapping sI(luminanceI, sMapping);
    sI.build(I);
    sI.getMapping()->inverse(sI.get_s(), Irec);

    // desired visual feature built from the image
    vpFeatureLuminance luminanceId;
    luminanceId.init(I.getHeight(), I.getWidth(), Z);
    luminanceId.setCameraParameters(cam);
    vpFeatureLuminanceMapping sId(luminanceId, sdMapping);
    sId.build(Id);

    // set a velocity control mode
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    int iter = 1;
    int iterGN = opt_niter / 8;
    double normError = 0;
    vpColVector v; // camera velocity sent to the robot
    vpColVector error(sI.dimension_s(), 0);

    unsigned int n = 6;
    vpMatrix L;
    vpMatrix Hs(n, n);
    vpMatrix H;
    vpMatrix diagHs(n, n);

    vpChrono chrono;
    chrono.start();
    std::cout << "Starting VS loop" << std::endl;
    do {
      std::cout << "--------------------------------------------" << iter++ << std::endl;

      //  Acquire the new image
      sim.setCameraPosition(cMo);
      sim.getImage(I, cam);
      vpImageTools::imageDifference(I, Id, Idiff);

      // Compute current visual features
      sI.build(I);
      sI.getMapping()->inverse(sI.get_s(), Irec);

      if (iter > iterGN) {
        mu = 0.0001;
        opt_lambda = lambdaGN;
      }
      sI.interaction(L);
      sI.error(sId, error);

      Hs = L.AtA();
      for (unsigned int i = 0; i < n; i++) {
        diagHs[i][i] = Hs[i][i];
      }
      H = ((mu * diagHs) + Hs).inverseByLU();
      // Compute the control law
      v = -opt_lambda * H * L.t() * error;
      normError = error.sumSquare();

      std::cout << " |e| = " << normError << std::endl;
      std::cout << " |v| = " << sqrt(v.sumSquare()) << std::endl;

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
      if (opt_display) {
        vpDisplay::display(I);
        vpDisplay::flush(I);
        vpDisplay::display(Irec);
        vpDisplay::flush(Irec);
        vpDisplay::display(Idiff);
        vpDisplay::flush(Idiff);
      }
#endif

      // send the robot velocity
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      wMc = robot.getPosition();
      cMo = wMc.inverse() * wMo;
    } while (normError > 200 && iter < opt_niter);

    chrono.stop();
    std::cout << "Time to convergence: " << chrono.getDurationMs() << " ms" << std::endl;

    v = 0;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);

    if (normError > 200) {
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
#endif
}
