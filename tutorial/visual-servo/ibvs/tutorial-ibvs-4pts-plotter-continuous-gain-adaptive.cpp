/*! \example tutorial-ibvs-4pts-plotter-continuous-gain-adaptive.cpp */
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpPlot.h>

int main()
{
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    vpPoint point[4] ;
    point[0].setWorldCoordinates(-0.1,-0.1, 0);
    point[1].setWorldCoordinates( 0.1,-0.1, 0);
    point[2].setWorldCoordinates( 0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    vpAdaptiveGain lambda(4, 0.4, 30);
    task.setLambda(lambda);

    vpFeaturePoint p[4], pd[4] ;
    for (unsigned int i = 0 ; i < 4 ; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      task.addFeature(p[i], pd[i]);
    }

    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

#ifdef VISP_HAVE_DISPLAY
    vpPlot plotter(2, 250*2, 500, 100, 200, "Real time curves plotter");
    plotter.setTitle(0, "Visual features error");
    plotter.setTitle(1, "Camera velocities");

    plotter.initGraph(0, 8);
    plotter.initGraph(1, 6);

    plotter.setLegend(0, 0, "x1");
    plotter.setLegend(0, 1, "y1");
    plotter.setLegend(0, 2, "x2");
    plotter.setLegend(0, 3, "y2");
    plotter.setLegend(0, 4, "x3");
    plotter.setLegend(0, 5, "y3");
    plotter.setLegend(0, 6, "x4");
    plotter.setLegend(0, 7, "y4");

    plotter.setLegend(1, 0, "v_x");
    plotter.setLegend(1, 1, "v_y");
    plotter.setLegend(1, 2, "v_z");
    plotter.setLegend(1, 3, "w_x");
    plotter.setLegend(1, 4, "w_y");
    plotter.setLegend(1, 5, "w_z");
#endif

    unsigned int iter = 0;
    while(1) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0 ; i < 4 ; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }
      vpColVector v = task.computeControlLaw(iter*robot.getSamplingTime());
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

#ifdef VISP_HAVE_DISPLAY
      plotter.plot(0, iter, task.getError());
      plotter.plot(1, iter, v);
#endif
      if (( task.getError() ).sumSquare() < 0.0001)
        break;

      iter++;
    }
    std::cout << "Convergence in " << iter << " iterations" << std::endl;

    task.kill();

#ifdef VISP_HAVE_DISPLAY
    plotter.saveData(0, "error.dat");
    plotter.saveData(1, "vc.dat");

    vpDisplay::getClick(plotter.I);
#endif
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}


