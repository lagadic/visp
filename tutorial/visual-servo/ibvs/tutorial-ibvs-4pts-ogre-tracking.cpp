/*! \example tutorial-ibvs-4pts-ogre-tracking.cpp */
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_AR
#include <visp3/ar/vpAROgre.h>
#endif
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vision/vpPose.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/blob/vpDot2.h>

void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot, unsigned int thickness);
#if defined(VISP_HAVE_OGRE)
void ogre_get_render_image(vpAROgre &ogre, const vpImage<unsigned char> &background,
                           const vpHomogeneousMatrix &cMo, vpImage<unsigned char> &I);
#endif

void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot, unsigned int thickness)
{
  static std::vector<vpImagePoint> traj[4];
  for (unsigned int i=0; i<4; i++) {
    traj[i].push_back(dot[i].getCog());
  }
  for (unsigned int i=0; i<4; i++) {
    for (unsigned int j=1; j<traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j-1], traj[i][j], vpColor::green, thickness);
    }
  }
}

#if defined(VISP_HAVE_OGRE)
void ogre_get_render_image(vpAROgre &ogre, const vpImage<unsigned char> &background,
                           const vpHomogeneousMatrix &cMo, vpImage<unsigned char> &I)
{
  static vpImage<vpRGBa> Irender; // Image from ogre scene rendering
  ogre.display(background, cMo);
  ogre.getRenderingOutput(Irender, cMo);

  vpImageConvert::convert(Irender, I);
  // Due to the light that was added to the scene, we need to threshold the image
  vpImageTools::binarise(I, (unsigned char)254, (unsigned char)255, (unsigned char)0, (unsigned char)255, (unsigned char)255);
}
#endif

int main()
{
#if defined(VISP_HAVE_OGRE) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  try {
    unsigned int thickness = 3;

    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    // Color image used as background texture.
    vpImage<unsigned char> background(480, 640, 255);

    // Parameters of our camera
    vpCameraParameters cam(840, 840, background.getWidth()/2, background.getHeight()/2);

    // Define the target as 4 points
    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.1,-0.1, 0) );
    point.push_back( vpPoint( 0.1,-0.1, 0) );
    point.push_back( vpPoint( 0.1, 0.1, 0) );
    point.push_back( vpPoint(-0.1, 0.1, 0) );

    // Our object
    // A simulator with the camera parameters defined above,
    // and the background image size
    vpAROgre ogre;
    ogre.setCameraParameters(cam);
    ogre.setShowConfigDialog(false);
    ogre.addResource("./"); // Add the path to the Sphere.mesh resource
    ogre.init(background, false, true);
    //ogre.setWindowPosition(680, 400);

    // Create the scene that contains 4 spheres
    // Sphere.mesh contains a sphere with 1 meter radius
    std::vector<std::string> name(4);
    for (unsigned int i=0; i<4; i++) {
      std::ostringstream s; s << "Sphere" <<  i; name[i] = s.str();
      ogre.load(name[i], "Sphere.mesh");
      ogre.setScale(name[i], 0.02f, 0.02f, 0.02f); // Rescale the sphere to 2 cm radius
      // Set the position of each sphere in the object frame
      ogre.setPosition(name[i], vpTranslationVector(point[i].get_oX(), point[i].get_oY(), point[i].get_oZ()));
      ogre.setRotation(name[i], vpRotationMatrix(M_PI/2, 0, 0));
    }

    // Add an optional point light source
    Ogre::Light * light = ogre.getSceneManager()->createLight();
    light->setDiffuseColour(1, 1, 1); // scaled RGB values
    light->setSpecularColour(1, 1, 1); // scaled RGB values
    light->setPosition((Ogre::Real)cdMo[0][3], (Ogre::Real)cdMo[1][3], (Ogre::Real)(-cdMo[2][3]));
    light->setType(Ogre::Light::LT_POINT);

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    // Image used for the image processing
    vpImage<unsigned char> I;

    // Render the scene at the desired position
    ogre_get_render_image(ogre, background, cdMo, I);

    // Display the image in which we will do the tracking
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Camera view at desired position");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Camera view at desired position");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I, 0, 0, "Camera view at desired position");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpDisplay::display(I);
    vpDisplay::displayText(I, 10, 10, "Click in the 4 dots to learn their positions", vpColor::red);
    vpDisplay::flush(I);

    std::vector<vpDot2> dot(4);
    vpFeaturePoint p[4], pd[4];

    for (unsigned int i = 0 ; i < 4 ; i++) {
      // Compute the desired feature at the desired position
      dot[i].setGraphics(true);
      dot[i].setGraphicsThickness(thickness);
      dot[i].initTracking(I);
      vpDisplay::flush(I);
      vpFeatureBuilder::create(pd[i], cam, dot[i].getCog());
    }

    // Render the scene at the initial position
    ogre_get_render_image(ogre, background, cMo, I);

    vpDisplay::display(I);
    vpDisplay::setTitle(I, "Current camera view");
    vpDisplay::displayText(I, 10, 10, "Click in the 4 dots to initialise the tracking and start the servo", vpColor::red);
    vpDisplay::flush(I);

    for (unsigned int i = 0 ; i < 4 ; i++) {
      // We notice that if we project the scene at a given pose, the pose estimated from
      // the rendered image differs a little. That's why we cannot simply compute the desired
      // feature from the desired pose using the next two lines. We will rather compute the
      // desired position of the features from a learning stage.
      // point[i].project(cdMo);
      // vpFeatureBuilder::create(pd[i], point[i]);

      // Compute the current feature at the initial position
      dot[i].setGraphics(true);
      dot[i].initTracking(I);
      vpDisplay::flush(I);
      vpFeatureBuilder::create(p[i], cam, dot[i].getCog());
    }

    for (unsigned int i = 0 ; i < 4 ; i++) {
      // Set the feature Z coordinate from the pose
      vpColVector cP;
      point[i].changeFrame(cMo, cP) ;
      p[i].set_Z(cP[2]);

      task.addFeature(p[i], pd[i]);
    }

    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    for (; ; ) {
      // From the camera position in the world frame we retrieve the object position
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;

      // Update the scene from the new camera position
      ogre_get_render_image(ogre, background, cMo, I);

      vpDisplay::display(I);

      for (unsigned int i = 0 ; i < 4 ; i++) {
        dot[i].track(I);
        vpFeatureBuilder::create(p[i], cam, dot[i].getCog());
      }

      for (unsigned int i = 0 ; i < 4 ; i++) {
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(cMo, cP) ;
        p[i].set_Z(cP[2]);
      }

      vpColVector v = task.computeControlLaw();

      display_trajectory(I, dot, thickness);
      vpServoDisplay::display(task, cam, I, vpColor::green, vpColor::red, thickness+2) ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;

      vpTime::wait( robot.getSamplingTime() * 1000);
    }
    task.kill();
  }
  catch(vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
  catch(...) {
    std::cout << "Catch an exception " << std::endl;
    return 1;
  }
#endif
}

