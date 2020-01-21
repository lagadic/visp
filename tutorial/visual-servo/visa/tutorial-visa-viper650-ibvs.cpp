/*! \example tutorial-visa-viper650-ibvs.cpp */

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpViper650.h>
#include <visp3/robot/vpVisaSocketAdaptor.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#define L 0.03 // to deal with a 6cm by 6cm square

/*!

  Compute the pose \e cMo from the 3D coordinates of the points \e point and
  their corresponding 2D coordinates \e dot. The pose is computed using a Lowe
  non linear method.

  \param point : 3D coordinates of the points.

  \param dot : 2D coordinates of the points.

  \param ndot : Number of points or dots used for the pose estimation.

  \param cam : Intrinsic camera parameters.

  \param cMo : Homogeneous matrix in output describing the transformation
  between the camera and object frame.

  \param cto : Translation in ouput extracted from \e cMo.

  \param cro : Rotation in ouput extracted from \e cMo.

  \param init : Indicates if the we have to estimate an initial pose with
  Lagrange or Dementhon methods.

*/
void compute_pose(vpPoint point[], vpDot2 dot[], int ndot, vpCameraParameters cam, vpHomogeneousMatrix &cMo,
                  vpTranslationVector &cto, vpRxyzVector &cro, bool init)
{
  vpHomogeneousMatrix cMo_dementhon; // computed pose with dementhon
  vpHomogeneousMatrix cMo_lagrange;  // computed pose with dementhon
  vpRotationMatrix cRo;
  vpPose pose;
  vpImagePoint cog;
  for (int i = 0; i < ndot; i++) {

    double x = 0, y = 0;
    cog = dot[i].getCog();
    vpPixelMeterConversion::convertPoint(cam, cog, x,
                                         y); // pixel to meter conversion
    point[i].set_x(x);                       // projection perspective          p
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
    // Compute and return the residual expressed in meter for the pose matrix
    // 'cMo'
    double residual_dementhon = pose.computeResidual(cMo_dementhon);
    pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
    double residual_lagrange = pose.computeResidual(cMo_lagrange);

    // Select the best pose to initialize the lowe pose computation
    if (residual_lagrange < residual_dementhon)
      cMo = cMo_lagrange;
    else
      cMo = cMo_dementhon;

  } else { // init = false; use of the previous pose to initialise LOWE
    cRo.buildFrom(cro);
    cMo.buildFrom(cto, cRo);
  }
  pose.computePose(vpPose::LOWE, cMo);
  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
}

int main(int argc, char **argv)
{
  try {
    std::string opt_hostname = "127.0.0.1";
    unsigned int opt_port = 2408;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--hostname")
        opt_hostname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--port")
        opt_port = std::atoi(argv[i + 1]);
      else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
        std::cout << "SYNOPSIS" << std::endl
                  << "  " << argv[0]
                  << " [--hostname <host name>] [--port <port>] [--help] [-h]\n"
                  << std::endl;
        std::cout << "DESCRIPTION" << std::endl
                  << "  --host <host name>" << std::endl
                  << "    Computer host name running ViSA simulator." << std::endl << std::endl
                  << "  --port <port>" << std::endl
                  << "    Port to connect to ViSA simulator." << std::endl << std::endl
                  << "  --help, -h" << std::endl
                  << "    Print this helper message. " << std::endl << std::endl;
        return 0;
      }
    }

    vpViper650 robot;

    vpHomogeneousMatrix eMc(vpTranslationVector(0, 0, 0), vpRotationMatrix(vpRxyzVector(0, 0, -M_PI/2.)));
    vpVelocityTwistMatrix cVe(eMc.inverse());

    vpServo task;

    // init communication with simulator
    vpVisaSocketAdapter visa;
    visa.connect(opt_hostname, opt_port);

    vpCameraParameters cam = visa.getCameraParameters();

    std::cout << "Focal distances (x,y) = (" << cam.get_px() << ", " << cam.get_py() << ")" << std::endl;
    std::cout << "Principal point = (" << cam.get_u0() << ", " << cam.get_v0() << ")" << std::endl;
    // image capture
    vpImage<unsigned char> I(cam.get_v0()*2, cam.get_u0()*2, 0);

    I = visa.getImageViSP();

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, 100, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDot2 dot[4];
    vpImagePoint cog;

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..." << std::endl;

    for (unsigned int i = 0; i < 4; i++) {
      dot[i].setGraphics(true);
      dot[i].initTracking(I);
      cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }

    // Sets the current position of the visual feature
    vpFeaturePoint p[4];
    for (unsigned int i = 0; i < 4; i++)
      vpFeatureBuilder::create(p[i], cam, dot[i]); // retrieve x,y  of the vpFeaturePoint structure

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    vpPoint point[4];
    point[0].setWorldCoordinates(-L, -L, 0);
    point[1].setWorldCoordinates( L, -L, 0);
    point[2].setWorldCoordinates( L,  L, 0);
    point[3].setWorldCoordinates(-L,  L, 0);

    // Initialise a desired pose to compute s*, the desired 2D point features
    vpHomogeneousMatrix cMo;
    vpTranslationVector cto(0, 0, 0.5); // tz = 0.5 meter
    vpRxyzVector cro(vpMath::rad(0), vpMath::rad(10), vpMath::rad(20));
    vpRotationMatrix cRo(cro); // Build the rotation matrix
    cMo.buildFrom(cto, cRo);   // Build the homogeneous matrix

    // Sets the desired position of the 2D visual feature
    vpFeaturePoint pd[4];
    // Compute the desired position of the features from the desired pose
    for (int i = 0; i < 4; i++) {
      vpColVector cP, p;
      point[i].changeFrame(cMo, cP);
      point[i].projection(cP, p);

      pd[i].set_x(p[0]);
      pd[i].set_y(p[1]);
      pd[i].set_Z(cP[2]);
    }

    // We want to see a point on a point
    for (unsigned int i = 0; i < 4; i++)
      task.addFeature(p[i], pd[i]);

    // Set the proportional gain
    task.setLambda(0.3);

    // Display task information
    task.print();

    // Define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.set_cVe(cVe);
    task.print();

    // Set the Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    bool quit = false;

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;

    std::vector<std::vector<vpImagePoint> > traj_cog(4);

    while (! quit) {
      double t = vpTime::measureTimeMs();

      // Acquire a new image from the camera
      I = visa.getImageViSP();

      // Display this image
      vpDisplay::display(I);

      try {
        // For each point...
        for (unsigned int i = 0; i < 4; i++) {
          // Achieve the tracking of the dot in the image
          dot[i].track(I);
          // Display a green cross at the center of gravity position in the
          // image
          cog = dot[i].getCog();
          vpDisplay::displayCross(I, cog, 10, vpColor::green);
          traj_cog[i].push_back(cog);
        }
      } catch (...) {
        quit = true;
      }

      // During the servo, we compute the pose using LOWE method. For the
      // initial pose used in the non linear minimisation we use the pose
      // computed at the previous iteration.
      compute_pose(point, dot, 4, cam, cMo, cto, cro, false);

      std::cout << "cMo:\n" << cMo << std::endl;
      vpHomogeneousMatrix fMe;
      for (unsigned int i = 0; i < 4; i++) {
        // Update the point feature from the dot location
        vpFeatureBuilder::create(p[i], cam, dot[i]);
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(cMo, cP);

        p[i].set_Z(cP[2]);
      }

      // Get the jacobian of the robot
      std::vector<double> qvec;
      visa.getJointPos(qvec);
      vpColVector q(qvec);
      robot.get_eJe(q, eJe);
      // Update this jacobian in the task structure. It will be used to
      // compute the velocity skew (as an articular velocity) qdot = -lambda *
      // L^+ * cVe * eJe * (s-s*)
      task.set_eJe(eJe);

      // Compute the visual servoing skew vector
      vpColVector v = task.computeControlLaw();
      std::vector<double> v_;
      for (unsigned int i=0; i < v.size(); i++) {
        v_.push_back(v[i]);
      }

      v.rad2deg();
      std::cout << "Send qdot in deg: " << v.t() << std::endl;

      visa.setJointVel(v_);

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);
      for (size_t i = 0; i < traj_cog.size(); i ++) {
        vpDisplay::displayPolygon(I, traj_cog[i], vpColor::green, 1, false);
      }

      std::cout << "Joint vel: " << v.t() << std::endl;

      // Flush the display
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }

      // std::cout << "|| s - s* || = "  << ( task.getError() ).sumSquare() <<
      // std::endl;
      vpTime::wait(t, 40); // Loop time is set to 40 ms, ie 25 Hz
    }

    vpColVector v(6);
    v = 0;
    visa.setJointVel(v); // stop robot

    std::cout << "Display task information: " << std::endl;
    task.print();
    task.kill();
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
