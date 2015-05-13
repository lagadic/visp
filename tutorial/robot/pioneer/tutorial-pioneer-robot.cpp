/*! \example tutorial-pioneer-robot.cpp */
#include <iostream>
#include <visp3/robot/vpRobotPioneer.h>

int main()
{
#ifdef VISP_HAVE_PIONEER
  try {
    ArArgumentBuilder args;
    args.add("-rp");
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    args.add("/dev/ttyUSB0");
#else
    args.add("COM3");
#endif

    ArSimpleConnector conn(&args);

    vpRobotPioneer robot;

    if (!conn.connectRobot(&robot))
      return -1;

    robot.useSonar(false);
    vpTime::wait(2000);

    vpColVector v(2);
    v = 0;
    v[0] = 0.10; // Translational velocity in m/s
    robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

    vpTime::wait(1000);
    vpColVector v_mes = robot.getVelocity(vpRobot::REFERENCE_FRAME);
    std::cout << "Measured vel: " << v_mes.t() << std::endl;
    vpTime::wait(1000);

    robot.stopRunning();
    robot.waitForRunExit();
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
