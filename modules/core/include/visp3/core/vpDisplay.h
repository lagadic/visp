/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 *
 * Description:
 * Image display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpDisplay_h
#define vpDisplay_h

#include <sstream>
#include <string>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMouseButton.h>
#include <visp3/core/vpRect.h>

/*!
  \file vpDisplay.h
  \brief Generic class for image display, also provide the interface
  with the image.
*/

/*!

  \class vpDisplay

  \ingroup group_core_gui

  \brief Class that defines generic functionnalities for display.

  The \ref tutorial-getting-started is a good starting point to know
  how to use this class to display an image in a window.

  The example below shows how to use this class.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

int main()
{
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
#ifdef _WIN32
  vpImageIo::read(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");
#else
  vpImageIo::read(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
#endif

  vpDisplay *d;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My image");

  // To initialize the video device, it is also possible to replace
  // the 3 previous lines by:
  // d->init(I, 400, 100, "My image");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(50);
  topLeftCorner.set_j(10);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green,
                              true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Get non blocking keyboard events
  std::cout << "Check keyboard events..." << std::endl;
  char key[10]; sprintf(key, "\0");
  bool ret;
  for (int i=0; i< 200; i++) {
    bool ret = vpDisplay::getKeyboardEvent(I, key, false);
    if (ret)
      std::cout << "keyboard event: key: " << "\"" << key
                << "\"" << std::endl;
    vpTime::wait(40);
  }

  // Get a blocking keyboard event
  std::cout << "Wait for a keyboard event..." << std::endl;
  ret = vpDisplay::getKeyboardEvent(I, key, true);
  std::cout << "keyboard event: " << ret << std::endl;
  if (ret)
    std::cout << "key: " << "\"" << key << "\"" << std::endl;

  // Wait for a click in the display window
  std::cout << "Wait for a button click..." << std::endl;
  vpDisplay::getClick(I);

  delete d;
}
  \endcode

  Other examples are available in tutorial-image-viewer.cpp and
  tutorial-viewer.cpp.
*/
class VISP_EXPORT vpDisplay
{
public:
  //! Values that could be applied to a display to down scale the size of the
  //! display.
  typedef enum {
    SCALE_AUTO,   /*!< Auto down scaling factor computed fom the screen
                     resolution. */
    SCALE_1,      /*!< Display and image have the same size. */
    SCALE_2,      /*!< Display width and height is down scaled by 2 wrt the image
                     size. */
    SCALE_3,      /*!< Display width and height is down scaled by 3 wrt the image
                     size. */
    SCALE_4,      /*!< Display width and height is down scaled by 4 wrt the image
                     size. */
    SCALE_5,      /*!< Display width and height is down scaled by 5 wrt the image
                     size. */
    SCALE_6,      /*!< Display width and height is down scaled by 6 wrt the image
                     size. */
    SCALE_7,      /*!< Display width and height is down scaled by 7 wrt the image
                     size. */
    SCALE_8,      /*!< Display width and height is down scaled by 8 wrt the image
                     size. */
    SCALE_9,      /*!< Display width and height is down scaled by 9 wrt the image
                     size. */
    SCALE_10,     /*!< Display width and height is down scaled by 10 wrt the image
                     size. */
    SCALE_DEFAULT /*!< Display and image have the same size. Similar to
                     vpDisplay::SCALE_1. */
  } vpScaleType;

protected:
  //! display has been initialized
  bool m_displayHasBeenInitialized;
  //! display position
  int m_windowXPosition;
  //! display position
  int m_windowYPosition;
  unsigned int m_width;
  unsigned int m_height;
  std::string m_title;
  unsigned int m_scale;
  vpScaleType m_scaleType;

  void setScale(vpScaleType scaleType, unsigned int width, unsigned int height);

public:
  vpDisplay();
  vpDisplay(const vpDisplay &d);
  virtual ~vpDisplay();

  /** @name Inherited functionalities from vpDisplay */
  //@{
  unsigned int computeAutoScale(unsigned int width, unsigned int height);
  /*!
    Return the value of the down scale factor applied to the image in order to
    reduce the size of the window used to display the image.
   */
  unsigned int getDownScalingFactor() { return m_scale; }
  /*!
    Return the display height.
    \sa getWidth()
  */
  inline unsigned int getHeight() const { return m_height; }
  /*!
    Return the display width.
    \sa getHeight()
  */
  inline unsigned int getWidth() const { return m_width; }

  /*!
    Return the position (along the horizontal axis) on the screen of the
    display window. \sa getWindowYPosition()
   */
  int getWindowXPosition() const { return m_windowXPosition; }
  /*!
    Return the position (along the vertical axis) on the screen of the display
    window. \sa getWindowXPosition()
   */
  int getWindowYPosition() const { return m_windowYPosition; }

  /*!
    Check if the display has been initialised

    \return True if the display has been initialised, otherwise False
  */
  inline bool isInitialised() { return m_displayHasBeenInitialized; }
  virtual void setDownScalingFactor(unsigned int scale);
  virtual void setDownScalingFactor(vpScaleType scaleType);
//@}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /** @name vpDisplay pure virtual functions */
  //@{
  /*!
    Set the window backgroud to \e color.
    \param color : Background color.
  */
  virtual void clearDisplay(const vpColor &color = vpColor::white) = 0;
  /*!
    Close the window.
  */
  virtual void closeDisplay() = 0;

  /*!
    Display an arrow from image point \e ip1 to image point \e ip2.
    \param ip1 : Initial image point.
    \param ip2 : Final image point.
    \param color : Arrow color.
    \param w : Arrow width.
    \param h : Arrow height.
    \param thickness : Thickness of the lines used to display the arrow.
  */
  virtual void displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color = vpColor::white,
                            unsigned int w = 4, unsigned int h = 2, unsigned int thickness = 1) = 0;
  /*!
    Display a string at the image point \e ip location.

    To select the font used to display the string, use setFont().

    \param ip : Upper left image point location of the string in the display.
    \param text : String to display in overlay.
    \param color : String color.

    \sa setFont()
  */
  virtual void displayCharString(const vpImagePoint &ip, const char *text, const vpColor &color = vpColor::green) = 0;
  /*!
    Display a circle.
    \param center : Circle center position.
    \param radius : Circle radius.
    \param color : Circle color.
    \param fill : When set to true fill the rectangle.
    \param thickness : Thickness of the circle. This parameter is only useful
    when \e fill is set to false.
  */
  virtual void displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill = false,
                             unsigned int thickness = 1) = 0;
  /*!
    Display a cross at the image point \e ip location.
    \param ip : Cross location.
    \param size : Size (width and height) of the cross.
    \param color : Cross color.
    \param thickness : Thickness of the lines used to display the cross.
  */
  virtual void displayCross(const vpImagePoint &ip, unsigned int size, const vpColor &color,
                            unsigned int thickness = 1) = 0;
  /*!
    Display a dashed line from image point \e ip1 to image point \e ip2.
    \param ip1 : Initial image point.
    \param ip2 : Final image point.
    \param color : Line color.
    \param thickness : Dashed line thickness.
  */
  virtual void displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                              unsigned int thickness = 1) = 0;
  /*!
    Display a line from image point \e ip1 to image point \e ip2.
    \param ip1 : Initial image point.
    \param ip2 : Final image point.
    \param color : Line color.
    \param thickness : Line thickness.
  */
  virtual void displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                           unsigned int thickness = 1) = 0;

  /*!
    Display the gray level image \e I (8bits).

    \warning Display has to be initialized.

    \warning Suppress the overlay drawing.

    \param I : Image to display.

    \sa init(), closeDisplay()
  */
  virtual void displayImage(const vpImage<unsigned char> &I) = 0;

  /*!
    Display the color image \e I in RGBa format (32bits).

    \warning Display has to be initialized.

    \warning Suppress the overlay drawing.

    \param I : Image to display.

    \sa init(), closeDisplay()
  */
  virtual void displayImage(const vpImage<vpRGBa> &I) = 0;

  virtual void displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int width,
                               const unsigned int height) = 0;
  virtual void displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int width,
                               const unsigned int height) = 0;

  /*!
    Display a point at the image point \e ip location.
    \param ip : Point location.
    \param color : Point color.
    \param thickness : Point thickness.
  */
  virtual void displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness = 1) = 0;

  /*!
    Display a rectangle with \e topLeft as the top-left corner and \e
    width and \e height the rectangle size.

    \param topLeft : Top-left corner of the rectangle.
    \param width : Rectangle width.
    \param height : Rectangle height.
    \param color : Rectangle color.
    \param fill : When set to true fill the rectangle.

    \param thickness : Thickness of the four lines used to display the
    rectangle. This parameter is only useful when \e fill is set to
    false.
  */
  virtual void displayRectangle(const vpImagePoint &topLeft, unsigned int width, unsigned int height,
                                const vpColor &color, bool fill = false, unsigned int thickness = 1) = 0;
  /*!
    Display a rectangle with \e topLeft as the top-left corner and \e
    width and \e height the rectangle size.

    \param topLeft : Top-left corner of the rectangle.
    \param bottomRight : Bottom-right corner of the rectangle.
    \param color : Rectangle color.
    \param fill : When set to true fill the rectangle.

    \param thickness : Thickness of the four lines used to display the
    rectangle. This parameter is only useful when \e fill is set to
    false.
  */
  virtual void displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight, const vpColor &color,
                                bool fill = false, unsigned int thickness = 1) = 0;
  /*!
    Display a rectangle with \e topLeft as the top-left corner and \e
    width and \e height the rectangle size.

    \param rectangle : Rectangle characteristics.
    \param color : Rectangle color.
    \param fill : When set to true fill the rectangle.

    \param thickness : Thickness of the four lines used to display the
    rectangle. This parameter is only useful when \e fill is set to
    false.

  */
  virtual void displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill = false,
                                unsigned int thickness = 1) = 0;

  /*!
    Flushes the display.
    It's necessary to use this function to see the results of any drawing.
  */
  virtual void flushDisplay() = 0;

  /*!
    Flushes the display.
    It's necessary to use this function to see the results of any drawing.
  */
  virtual void flushDisplayROI(const vpImagePoint &iP, const unsigned int width, const unsigned int height) = 0;

  /* Simple interface with the mouse event */

  /*!
    Wait for a click from one of the mouse button.

    \param blocking [in] : Blocking behavior.
    - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
    - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

    \return
    - true if a button was clicked. This is always the case if blocking is set
    to \e true.
    - false if no button was clicked. This can occur if blocking is set
    to \e false.
  */
  virtual bool getClick(bool blocking = true) = 0;

  /*!
    Wait for a click from one of the mouse button and get the position
    of the clicked image point.

    \param ip [out] : The coordinates of the clicked image point.

    \param blocking [in] : true for a blocking behaviour waiting a mouse
    button click, false for a non blocking behaviour.

    \return
    - true if a button was clicked. This is always the case if blocking is set
    to \e true.
    - false if no button was clicked. This can occur if blocking is set
    to \e false.
  */
  virtual bool getClick(vpImagePoint &ip, bool blocking = true) = 0;
  /*!
    Wait for a mouse button click and get the position of the clicked
    pixel. The button used to click is also set.

    \param ip [out] : The coordinates of the clicked image point.

    \param button [out] : The button used to click.

    \param blocking [in] :
    - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
    - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

    \return true if a mouse button is pressed, false otherwise. If a
    button is pressed, the location of the mouse pointer is updated in
    \e ip.
  */
  virtual bool getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking = true) = 0;
  /*!
    Wait for a mouse button click release and get the position of the
    image point were the click release occurs.  The button used to click is
    also set. Same method as getClick(unsigned int&, unsigned int&,
    vpMouseButton::vpMouseButtonType &, bool).

    \param ip [out] : Position of the clicked image point.

    \param button [in] : Button used to click.

    \param blocking [in] : true for a blocking behaviour waiting a mouse
    button click, false for a non blocking behaviour.

    \return
    - true if a button was clicked. This is always the case if blocking is set
    to \e true.
    - false if no button was clicked. This can occur if blocking is set
    to \e false.

    \sa getClick(vpImagePoint &, vpMouseButton::vpMouseButtonType &, bool)

  */
  virtual bool getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking = true) = 0;

  /*!
    Get a keyboard event.

    \param blocking [in] : Blocking behavior.
    - When set to true, this method waits until a key is
      pressed and then returns always true.
    - When set to false, returns true only if a key is
      pressed, otherwise returns false.

      \return
      - true if a key was pressed. This is always the case if blocking is set
        to \e true.
      - false if no key was pressed. This can occur if blocking is set
        to \e false.
  */
  virtual bool getKeyboardEvent(bool blocking = true) = 0;
  /*!

    Get a keyboard event.

    \param blocking [in] : Blocking behavior.
    - When set to true, this method waits until a key is
      pressed and then returns always true.
    - When set to false, returns true only if a key is
      pressed, otherwise returns false.

    \param key [out]: If possible, an ISO Latin-1 character
    corresponding to the keyboard key.

    \return
    - true if a key was pressed. This is always the case if blocking is set
      to \e true.
    - false if no key was pressed. This can occur if blocking is set
      to \e false.
  */
  virtual bool getKeyboardEvent(std::string &key, bool blocking = true) = 0;
  /*!
    Get the coordinates of the mouse pointer.

    \param ip [out] : The coordinates of the mouse pointer.

    \return true if a pointer motion event was received, false otherwise.

    \exception vpDisplayException::notInitializedError : If the display
    was not initialized.
  */
  virtual bool getPointerMotionEvent(vpImagePoint &ip) = 0;

  /*!
    Get the coordinates of the mouse pointer.

    \param ip [out] : The coordinates of the mouse pointer.

    \return true.

    \exception vpDisplayException::notInitializedError : If the display
    was not initialized.
  */
  virtual bool getPointerPosition(vpImagePoint &ip) = 0;

  /*!
    Gets the screen vertical resolution in pixel.
   */
  virtual unsigned int getScreenHeight() = 0;
  /*!
    Gets the screen resolution in pixel.
    \param width, height : Screen resolution in pixels.
   */
  virtual void getScreenSize(unsigned int &width, unsigned int &height) = 0;
  /*!
    Gets the screen horizontal resolution in pixel.
   */
  virtual unsigned int getScreenWidth() = 0;

  /*!
    Initialize the display (size, position and title) of a gray level image.

    \param I : Image to be displayed (not that image has to be initialized)
    \param x : Horizontal position of the upper/left window corner.
    \param y : Vertical position of the upper/left window corner.
    \param title : Window title.
  */
  virtual void init(vpImage<unsigned char> &I, int x = -1, int y = -1, const std::string &title = "") = 0;
  /*!
    Initialize the display (size, position and title) of a color
    image in RGBa format.

    \param I : Image to be displayed (not that image has to be initialized)
    \param x : Horizontal position of the upper/left window corner.
    \param y : Vertical position of the upper/left window corner.
    \param title : Window title.
  */
  virtual void init(vpImage<vpRGBa> &I, int x = -1, int y = -1, const std::string &title = "") = 0;

  /*!
    Initialize the display size, position and title.

    \param width : Window width.
    \param height : Window height.
    \param x : Horizontal position of the upper/left window corner.
    \param y : Vertical position of the upper/left window corner.
    \param title : Window title.

    The following example shows how to use this function
    \code
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef VISP_HAVE_DISPLAY
  vpImage<unsigned char> I;
  vpImageIo::read(I, "lena.pgm");

  vpDisplay *d;

#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#else
  std::cout << "Sorry, no video device is available" << std::endl;
  return -1;
#endif

  d->init(I.getWidth(), I.getHeight(), 10, 20, "viewer");

  // Now associate the display to the image
  I.display = d;

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // wait for a mouse clink in the display to exit
  vpDisplay::getClick(I);

  delete d;
#endif
}
    \endcode
  */
  virtual void init(unsigned int width, unsigned int height, int x = -1, int y = -1, const std::string &title = "") = 0;

  /*!
    Set the font used to display a text in overlay. The display is
    performed using displayCharString().

    \param font : The expected font name. The available fonts are given by
    the "xlsfonts" binary. To choose a font you can also use the
    "xfontsel" binary.

    \note Under UNIX, to know all the available fonts, use the
    "xlsfonts" binary in a terminal. You can also use the "xfontsel" binary.

    \sa displayCharString()
  */
  virtual void setFont(const std::string &font) = 0;
  /*!
    Set the window title.
    \param title : Window title.
  */
  virtual void setTitle(const std::string &title) = 0;
  /*!
    Set the window position in the screen.

    \param x : Horizontal position of the upper/left window corner.
    \param y : Vertical position of the upper/left window corner.

  */
  virtual void setWindowPosition(int x, int y) = 0;
//@}
#endif // ifndef DOXYGEN_SHOULD_SKIP_THIS

  /*!
    @name Static public vpDisplay functionalities on gray level images.
  */
  //@{
  static void close(vpImage<unsigned char> &I);
  static void display(const vpImage<unsigned char> &I);
  static void displayArrow(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                           const vpColor &color = vpColor::white, unsigned int w = 4, unsigned int h = 2,
                           unsigned int thickness = 1);
  static void displayArrow(const vpImage<unsigned char> &I, int i1, int j1, int i2, int j2,
                           const vpColor &color = vpColor::white, unsigned int w = 4, unsigned int h = 2,
                           unsigned int thickness = 1);
  static void displayCamera(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                            const vpCameraParameters &cam, double size, const vpColor &color, unsigned int thickness);
  static void displayCharString(const vpImage<unsigned char> &I, const vpImagePoint &ip, const char *string,
                                const vpColor &color);
  static void displayCharString(const vpImage<unsigned char> &I, int i, int j, const char *string,
                                const vpColor &color);
  static void displayCircle(const vpImage<unsigned char> &I, const vpImagePoint &center, unsigned int radius,
                            const vpColor &color, bool fill = false, unsigned int thickness = 1);
  static void displayCircle(const vpImage<unsigned char> &I, int i, int j, unsigned int radius, const vpColor &color,
                            bool fill = false, unsigned int thickness = 1);
  static void displayCross(const vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int size,
                           const vpColor &color, unsigned int thickness = 1);
  static void displayCross(const vpImage<unsigned char> &I, int i, int j, unsigned int size, const vpColor &color,
                           unsigned int thickness = 1);
  static void displayDotLine(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                             const vpColor &color, unsigned int thickness = 1);
  static void displayDotLine(const vpImage<unsigned char> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                             unsigned int thickness = 1);
  static void displayDotLine(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &ips,
                             const bool closeTheShape, const vpColor &color, unsigned int thickness = 1);
  static void displayEllipse(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &coef1,
                             const double &coef2, const double &coef3, bool use_centered_moments, const vpColor &color,
                             unsigned int thickness = 1);
  static void displayEllipse(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &coef1,
                             const double &coef2, const double &coef3, const double &theta1, const double &theta2,
                             bool use_centered_moments, const vpColor &color, unsigned int thickness = 1);
  static void displayFrame(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                           const vpCameraParameters &cam, double size, const vpColor &color = vpColor::none,
                           unsigned int thickness = 1, const vpImagePoint &offset = vpImagePoint(0, 0));
  static void displayLine(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                          const vpColor &color, unsigned int thickness = 1);
  static void displayLine(const vpImage<unsigned char> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                          unsigned int thickness = 1);
  static void displayLine(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &ips,
                          const bool closeTheShape, const vpColor &color, unsigned int thickness = 1);
  static void displayPoint(const vpImage<unsigned char> &I, const vpImagePoint &ip, const vpColor &color,
                           unsigned int thickness = 1);
  static void displayPoint(const vpImage<unsigned char> &I, int i, int j, const vpColor &color,
                           unsigned int thickness = 1);
  static void displayPolygon(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                             const vpColor &color, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<unsigned char> &I, const vpImagePoint &topLeft, unsigned int width,
                               unsigned int height, const vpColor &color, bool fill = false,
                               unsigned int thickness = 1);
  static void displayRectangle(const vpImage<unsigned char> &I, const vpImagePoint &topLeft,
                               const vpImagePoint &bottomRight, const vpColor &color, bool fill = false,
                               unsigned int thickness = 1);
  static void displayRectangle(const vpImage<unsigned char> &I, const vpRect &rectangle, const vpColor &color,
                               bool fill = false, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<unsigned char> &I, const vpImagePoint &center, float angle,
                               unsigned int width, unsigned int height, const vpColor &color,
                               unsigned int thickness = 1);
  static void displayRectangle(const vpImage<unsigned char> &I, int i, int j, unsigned int width, unsigned int height,
                               const vpColor &color, bool fill = false, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<unsigned char> &I, unsigned int i, unsigned int j, float angle,
                               unsigned int width, unsigned int height, const vpColor &color,
                               unsigned int thickness = 1);
  static void displayROI(const vpImage<unsigned char> &I, const vpRect &roi);
  static void displayText(const vpImage<unsigned char> &I, const vpImagePoint &ip, const std::string &s,
                          const vpColor &color);
  static void displayText(const vpImage<unsigned char> &I, int i, int j, const std::string &s, const vpColor &color);

  static void flush(const vpImage<unsigned char> &I);
  static void flushROI(const vpImage<unsigned char> &I, const vpRect &roi);

  static bool getClick(const vpImage<unsigned char> &I, bool blocking = true);
  static bool getClick(const vpImage<unsigned char> &I, vpImagePoint &ip, bool blocking = true);
  static bool getClick(const vpImage<unsigned char> &I, vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button,
                       bool blocking = true);
  static bool getClick(const vpImage<unsigned char> &I, vpMouseButton::vpMouseButtonType &button, bool blocking = true);
  static bool getClickUp(const vpImage<unsigned char> &I, vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button,
                         bool blocking = true);
  static bool getClickUp(const vpImage<unsigned char> &I, vpMouseButton::vpMouseButtonType &button,
                         bool blocking = true);
  static unsigned int getDownScalingFactor(const vpImage<unsigned char> &I);
  static void getImage(const vpImage<unsigned char> &Is, vpImage<vpRGBa> &Id);
  static bool getKeyboardEvent(const vpImage<unsigned char> &I, bool blocking = true);
  static bool getKeyboardEvent(const vpImage<unsigned char> &I, std::string &key, bool blocking = true);
  static bool getKeyboardEvent(const vpImage<unsigned char> &I, char *key, bool blocking = true);
  static bool getPointerMotionEvent(const vpImage<unsigned char> &I, vpImagePoint &ip);
  static bool getPointerPosition(const vpImage<unsigned char> &I, vpImagePoint &ip);

  static void setBackground(const vpImage<unsigned char> &I, const vpColor &color);
  static void setFont(const vpImage<unsigned char> &I, const std::string &font);
  static void setTitle(const vpImage<unsigned char> &I, const std::string &windowtitle);
  static void setWindowPosition(const vpImage<unsigned char> &I, int winx, int winy);
  //@}

  /*!
    @name Static public vpDisplay functionalities on 32 bits color images.
  */
  //@{
  static void close(vpImage<vpRGBa> &I);
  static void display(const vpImage<vpRGBa> &I);
  static void displayArrow(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                           const vpColor &color = vpColor::white, unsigned int w = 4, unsigned int h = 2,
                           unsigned int thickness = 1);
  static void displayArrow(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2,
                           const vpColor &color = vpColor::white, unsigned int w = 4, unsigned int h = 2,
                           unsigned int thickness = 1);
  static void displayCamera(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                            double size, const vpColor &color, unsigned int thickness);
  static void displayCharString(const vpImage<vpRGBa> &I, const vpImagePoint &ip, const char *string,
                                const vpColor &color);
  static void displayCharString(const vpImage<vpRGBa> &I, int i, int j, const char *string, const vpColor &color);
  static void displayCircle(const vpImage<vpRGBa> &I, const vpImagePoint &center, unsigned int radius,
                            const vpColor &color, bool fill = false, unsigned int thickness = 1);
  static void displayCircle(const vpImage<vpRGBa> &I, int i, int j, unsigned int radius, const vpColor &color,
                            bool fill = false, unsigned int thickness = 1);
  static void displayCross(const vpImage<vpRGBa> &I, const vpImagePoint &ip, unsigned int size, const vpColor &color,
                           unsigned int thickness = 1);
  static void displayCross(const vpImage<vpRGBa> &I, int i, int j, unsigned int size, const vpColor &color,
                           unsigned int thickness = 1);
  static void displayDotLine(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                             const vpColor &color, unsigned int thickness = 1);
  static void displayDotLine(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                             unsigned int thickness = 1);
  static void displayDotLine(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &ips, const bool closeTheShape,
                             const vpColor &color, unsigned int thickness = 1);
  static void displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &coef1,
                             const double &coef2, const double &coef3, bool use_centered_moments, const vpColor &color,
                             unsigned int thickness = 1);
  static void displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &coef1,
                             const double &coef2, const double &coef3, const double &theta1, const double &theta2,
                             bool use_centered_moments, const vpColor &color, unsigned int thickness = 1);
  static void displayFrame(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                           double size, const vpColor &color = vpColor::none, unsigned int thickness = 1,
                           const vpImagePoint &offset = vpImagePoint(0, 0));
  static void displayLine(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                          const vpColor &color, unsigned int thickness = 1);
  static void displayLine(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                          unsigned int thickness = 1);
  static void displayLine(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &ips, const bool closeTheShape,
                          const vpColor &color, unsigned int thickness = 1);
  static void displayPoint(const vpImage<vpRGBa> &I, const vpImagePoint &ip, const vpColor &color,
                           unsigned int thickness = 1);
  static void displayPoint(const vpImage<vpRGBa> &I, int i, int j, const vpColor &color, unsigned int thickness = 1);
  static void displayPolygon(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &vip, const vpColor &color,
                             unsigned int thickness = 1);
  static void displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &topLeft, unsigned int width,
                               unsigned int height, const vpColor &color, bool fill = false,
                               unsigned int thickness = 1);
  static void displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &topLeft, const vpImagePoint &bottomRight,
                               const vpColor &color, bool fill = false, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<vpRGBa> &I, const vpRect &rectangle, const vpColor &color,
                               bool fill = false, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &center, float angle, unsigned int width,
                               unsigned int height, const vpColor &color, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<vpRGBa> &I, int i, int j, unsigned int width, unsigned int height,
                               const vpColor &color, bool fill = false, unsigned int thickness = 1);
  static void displayRectangle(const vpImage<vpRGBa> &I, unsigned int i, unsigned int j, float angle,
                               unsigned int width, unsigned int height, const vpColor &color,
                               unsigned int thickness = 1);
  static void displayROI(const vpImage<vpRGBa> &I, const vpRect &roi);
  static void displayText(const vpImage<vpRGBa> &I, const vpImagePoint &ip, const std::string &s, const vpColor &color);
  static void displayText(const vpImage<vpRGBa> &I, int i, int j, const std::string &s, const vpColor &color);

  static void flush(const vpImage<vpRGBa> &I);
  static void flushROI(const vpImage<vpRGBa> &I, const vpRect &roi);

  static bool getClick(const vpImage<vpRGBa> &I, bool blocking = true);
  static bool getClick(const vpImage<vpRGBa> &I, vpImagePoint &ip, bool blocking = true);
  static bool getClick(const vpImage<vpRGBa> &I, vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button,
                       bool blocking = true);
  static bool getClick(const vpImage<vpRGBa> &I, vpMouseButton::vpMouseButtonType &button, bool blocking = true);
  static bool getClickUp(const vpImage<vpRGBa> &I, vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button,
                         bool blocking = true);
  static bool getClickUp(const vpImage<vpRGBa> &I, vpMouseButton::vpMouseButtonType &button, bool blocking = true);
  static unsigned int getDownScalingFactor(const vpImage<vpRGBa> &I);
  static void getImage(const vpImage<vpRGBa> &Is, vpImage<vpRGBa> &Id);
  static bool getKeyboardEvent(const vpImage<vpRGBa> &I, bool blocking = true);
  static bool getKeyboardEvent(const vpImage<vpRGBa> &I, std::string &key, bool blocking = true);
  static bool getKeyboardEvent(const vpImage<vpRGBa> &I, char *key, bool blocking = true);
  static bool getPointerMotionEvent(const vpImage<vpRGBa> &I, vpImagePoint &ip);
  static bool getPointerPosition(const vpImage<vpRGBa> &I, vpImagePoint &ip);

  static void setBackground(const vpImage<vpRGBa> &I, const vpColor &color);
  static void setFont(const vpImage<vpRGBa> &I, const std::string &font);
  static void setTitle(const vpImage<vpRGBa> &I, const std::string &windowtitle);
  static void setWindowPosition(const vpImage<vpRGBa> &I, int winx, int winy);
  //@}

private:
  //! get the window pixmap and put it in vpRGBa image
  virtual void getImage(vpImage<vpRGBa> &I) = 0;
};

#endif
