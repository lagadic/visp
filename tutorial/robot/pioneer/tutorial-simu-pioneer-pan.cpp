/*!
  \example tutorial-simu-pioneer-pan.cpp

  Example that shows how to simulate a visual servoing on a Pioneer mobile robot equipped with a camera
  able to move along the pan axis.
  The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:
  - x the abscisse of the point measured at each iteration
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the rotational velocity
  and vx the translational velocity of the mobile platform at point M located at the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.

  */
#include <iostream>

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorPioneerPan.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

int main()
{
  try {
    // Set the position the camera has to reach
    vpHomogeneousMatrix cdMo ;
    cdMo[1][3] = 1.2; // t_y should be different from zero to be non singular
    cdMo[2][3] = 0.5;

    // Set the initial camera position
    vpHomogeneousMatrix cMo;
    cMo[0][3] = 0.3;
    cMo[1][3] = cdMo[1][3];
    cMo[2][3] = 1.;
    vpRotationMatrix cdRo(0, atan2(cMo[0][3], cMo[1][3]), 0);
    cMo.insert(cdRo);

    vpSimulatorPioneerPan robot ;
    robot.setSamplingTime(0.04);
    vpHomogeneousMatrix wMc, wMo;

    // Get robot position world frame
    robot.getPosition(wMc);

    // Compute the position of the object in the world frame
    wMo = wMc * cMo;

    // Define the target
    vpPoint point(0,0,0); // Coordinates in the object frame
    point.track(cMo);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.setLambda(0.2);

    vpVelocityTwistMatrix cVe;
    cVe = robot.get_cVe();
    task.set_cVe(cVe);

    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    // Current and desired visual feature associated later to the x coordinate of the point
    vpFeaturePoint s_x, s_xd;

    // Create the current x visual feature
    vpFeatureBuilder::create(s_x, point);

    // Create the desired x* visual feature
    s_xd.buildFrom(0, 0, cdMo[2][3]);

    // Add the feature
    task.addFeature(s_x, s_xd, vpFeaturePoint::selectX());

    // Create the current and desired log(Z/Z*) visual feature
    vpFeatureDepth s_Z, s_Zd;
    // Initial depth of the target in front of the camera
    double Z = point.get_Z();
    // Desired depth Z* of the target.
    double Zd = cdMo[2][3];
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd));
    s_Zd.buildFrom(0, 0, Zd, 0); // log(Z/Z*) = 0 that's why the last parameter is 0

    // Add the feature
    task.addFeature(s_Z, s_Zd);

#ifdef VISP_HAVE_DISPLAY
    // Create a window (800 by 500) at position (400, 10) with 3 graphics
    vpPlot graph(3, 800, 500, 400, 10, "Curves...");

    // Init the curve plotter
    graph.initGraph(0,3);
    graph.initGraph(1,2);
    graph.initGraph(2,1);
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");
    graph.setTitle(2, "Depth");
    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");
    graph.setLegend(0, 2, "qdot_pan");
    graph.setLegend(1, 0, "x");
    graph.setLegend(1, 1, "log(Z/Z*)");
    graph.setLegend(2, 0, "Z");
#endif

    int iter = 0;
    for (; ;)
    {
      robot.getPosition(wMc) ;
      cMo = wMc.inverse() * wMo;

      point.track(cMo);

      // Update the current x feature
      vpFeatureBuilder::create(s_x, point);

      // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
      Z = point.get_Z() ;
      s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd));

      robot.get_cVe(cVe);
      task.set_cVe(cVe);
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      // Compute the control law. Velocities are computed in the mobile robot reference frame
      vpColVector v = task.computeControlLaw();

      // Send the velocity to the robot
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

#ifdef VISP_HAVE_DISPLAY
      graph.plot(0, iter, v); // plot velocities applied to the robot
      graph.plot(1, iter, task.getError()); // plot error vector
      graph.plot(2, 0, iter, Z); // plot the depth
#endif
      iter ++;

      if (task.getError().sumSquare() < 0.0001) {
        std::cout << "Reached a small error. We stop the loop... " << std::endl;
        break;
      }
    }
#ifdef VISP_HAVE_DISPLAY
    const char *legend = "Click to quit...";
    vpDisplay::displayText(graph.I, (int)graph.I.getHeight()-60, (int)graph.I.getWidth()-150, legend, vpColor::red);
    vpDisplay::flush(graph.I);
    vpDisplay::getClick(graph.I);
#endif

    // Kill the servo task
    task.print();
    task.kill();
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
