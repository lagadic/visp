/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 *
 * Description:
 * Windows 32 display base class
 */

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))

#include <string>
#include <visp3/core/vpDisplayException.h>
#include <visp3/gui/vpDisplayWin32.h>

BEGIN_VISP_NAMESPACE

const int vpDisplayWin32::MAX_INIT_DELAY = 5000;

/*!
  Thread entry point.
  Used as a detour to initWindow.
*/
void vpCreateWindow(threadParam *param)
{
  // char* title = param->title;
  (param->vpDisp)->window.initWindow(param->title.c_str(), param->x, param->y, param->w, param->h);
  delete param;
}

/*!
  Constructors.
*/
vpDisplayWin32::vpDisplayWin32(vpWin32Renderer *rend) : iStatus(false), window(rend) { }

vpDisplayWin32::vpDisplayWin32(vpImage<vpRGBa> &I, int winx, int winy, const std::string &title)
  : iStatus(false), window(nullptr)
{
  init(I, winx, winy, title);
}

vpDisplayWin32::vpDisplayWin32(vpImage<unsigned char> &I, int winx, int winy, const std::string &title)
  : iStatus(false), window(nullptr)
{
  init(I, winx, winy, title);
}

/*!
 * Copy constructor.
 */
vpDisplayWin32::vpDisplayWin32(const vpDisplayWin32 &display) : vpDisplay(display)
{
  *this = display;
}

/*!
  * Destructor.
 */
vpDisplayWin32::~vpDisplayWin32() { closeDisplay(); }

/*!
 * Copy operator.
 */
vpDisplayWin32 &vpDisplayWin32::operator=(const vpDisplayWin32 &display)
{
  hThread = display.hThread;
  threadId = display.threadId;
  iStatus = display.iStatus;
  window = display.window;
  roi = display.roi;

  return *this;
}

/*!
  Constructor. Initialize a display to visualize a gray level image (8 bits).

  \param[in] I : Image to be displayed (not that image has to be initialized)
  \param[in] x : Upper left window corner position along the horizontal axis.
  \param[in] y : Upper left window corner position along the vertical axis.
  \param[in] title : Window title.
*/
void vpDisplayWin32::init(vpImage<unsigned char> &I, int x, int y, const std::string &title)
{
  if ((I.getHeight() == 0) || (I.getWidth() == 0)) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "Image not initialized"));
  }

  setScale(m_scaleType, I.getWidth(), I.getHeight());
  init(I.getWidth(), I.getHeight(), x, y, title);
  window.renderer->setWidth(I.getWidth() / m_scale);
  window.renderer->setHeight(I.getHeight() / m_scale);
  window.renderer->setImg(I);

  I.display = this;
}

/*!
  Constructor. Initialize a display to visualize a RGBa level image
  (32 bits).

  \param[in] I : Image to be displayed (not that image has to be initialized).
  \param[in] x : Upper left window corner position along the horizontal axis.
  \param[in] y : Upper left window corner position along the vertical axis.
  \param[in] title : Window title.
*/
void vpDisplayWin32::init(vpImage<vpRGBa> &I, int x, int y, const std::string &title)
{
  if ((I.getHeight() == 0) || (I.getWidth() == 0)) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "Image not initialized"));
  }

  setScale(m_scaleType, I.getWidth(), I.getHeight());
  init(I.getWidth(), I.getHeight(), x, y, title);
  window.renderer->setWidth(I.getWidth() / m_scale);
  window.renderer->setHeight(I.getHeight() / m_scale);
  window.renderer->setImg(I);

  I.display = this;
}

/*!
  Initialize the display size, position and title.

  \param[in] width : Window width.
  \param[in] height : Window height.
  \param[in] x : Upper left window corner position along the horizontal axis.
  \param[in] y : Upper left window corner position along the vertical axis.
  \param[in] title : Window title.
*/
void vpDisplayWin32::init(unsigned int width, unsigned int height, int x, int y, const std::string &title)
{
  if (!title.empty())
    m_title = title;
  else
    m_title = std::string(" ");

  if (x != -1)
    m_windowXPosition = x;
  if (y != -1)
    m_windowYPosition = y;

  // we prepare the window's thread creation
  setScale(m_scaleType, width, height);
  threadParam *param = new threadParam;
  param->x = m_windowXPosition;
  param->y = m_windowYPosition;
  param->w = width / m_scale;
  param->h = height / m_scale;
  param->vpDisp = this;
  param->title = this->m_title;

  // creates the window in a separate thread
  hThread = CreateThread(nullptr, 0, (LPTHREAD_START_ROUTINE)vpCreateWindow, param, 0, &threadId);

  // the initialization worked
  iStatus = (hThread != static_cast<HANDLE>(nullptr));

  m_displayHasBeenInitialized = true;
}

/*!
  If the window is not initialized yet, wait a little (MAX_INIT_DELAY).
  \exception notInitializedError : the window isn't initialized
*/
void vpDisplayWin32::waitForInit()
{
  // if the window is not initialized yet
  if (!window.isInitialized()) {
    // wait
    if (WAIT_OBJECT_0 != WaitForSingleObject(window.semaInit, MAX_INIT_DELAY))
      throw(vpDisplayException(vpDisplayException::notInitializedError, "Window not initialized"));
    // problem : the window is not initialized
  }
}

/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param[in] I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayWin32::displayImage(const vpImage<vpRGBa> &I)
{
  // waits if the window is not initialized
  waitForInit();

  // sets the image to render
  window.renderer->setImg(I);
  // sends a message to the window
  // PostMessage(window.getHWnd(),vpWM_DISPLAY,0,0);
}

/*!
  Display a selection of the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param[in] I : Image to display.

  \param[in] iP : Top left corner of the region of interest.

  \param[in] width : Width of the region of interest.

  \param[in] height : Height of the region of interest.

  \sa init(), closeDisplay()
*/
void vpDisplayWin32::displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, unsigned int width,
                                     unsigned int height)
{
  // waits if the window is not initialized
  waitForInit();

  // sets the image to render
  window.renderer->setImgROI(I, iP, width, height);
  // sends a message to the window
  // PostMessage(window.getHWnd(),vpWM_DISPLAY,0,0);
}

/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param[in] I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayWin32::displayImage(const vpImage<unsigned char> &I)
{
  // wait if the window is not initialized
  waitForInit();

  // sets the image to render
  window.renderer->setImg(I);
  // sends a message to the window
  // PostMessage(window.getHWnd(), vpWM_DISPLAY, 0,0);
}

/*!
  Display a selection of the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param[in] I : Image to display.

  \param[in] iP : Top left corner of the region of interest.

  \param[in] width : Width of the region of interest.

  \param[in] height : Height of the region of interest.

  \sa init(), closeDisplay()
*/
void vpDisplayWin32::displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int width,
                                     unsigned int height)
{
  // waits if the window is not initialized
  waitForInit();

  // sets the image to render
  window.renderer->setImgROI(I, iP, width, height);
  // sends a message to the window
  // PostMessage(window.getHWnd(),vpWM_DISPLAY,0,0);
}

/*!
  Wait for a click from one of the mouse button.

  \param[in] blocking : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return
  - true if a button was clicked. This is always the case if blocking is set to \e true.
  - false if no button was clicked. This can occur if blocking is set to \e false.
*/
bool vpDisplayWin32::getClick(bool blocking)
{
  // wait if the window is not initialized
  waitForInit();
  bool ret = false;
  // sends a message to the window
  //   PostMessage(window.getHWnd(), vpWM_GETCLICK, 0,0);

  // waits for a button to be pressed
  if (blocking) {
    WaitForSingleObject(window.semaClick, 0);
    WaitForSingleObject(window.semaClickUp, 0); // to erase previous events
    WaitForSingleObject(window.semaClick, INFINITE);
    ret = true;
  }
  else {
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClick, 0));
  }

  return ret;
}

/*!
  Wait for a click from one of the mouse button and get the position
  of the clicked image point.

  \param[out] ip : The coordinates of the clicked image point.

  \param[in] blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set to \e true.
  - false if no button was clicked. This can occur if blocking is set to \e false.

*/
bool vpDisplayWin32::getClick(vpImagePoint &ip, bool blocking)
{
  // wait if the window is not initialized
  waitForInit();

  bool ret = false;
  double u, v;
  // tells the window there has been a getclick demand
  //   PostMessage(window.getHWnd(), vpWM_GETCLICK, 0,0);
  // waits for a click
  if (blocking) {
    WaitForSingleObject(window.semaClick, 0);
    WaitForSingleObject(window.semaClickUp, 0); // to erase previous events
    WaitForSingleObject(window.semaClick, INFINITE);
    ret = true;
  }
  else {
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClick, 0));
  }

  u = window.clickX;
  v = window.clickY;
  ip.set_u(u * m_scale);
  ip.set_v(v * m_scale);

  return ret;
}

/*!
  Wait for a mouse button click and get the position of the clicked
  pixel. The button used to click is also set.

  \param[out] ip  : The coordinates of the clicked image point.

  \param[out] button [out] : The button used to click.

  \param[in] blocking :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in \e ip.
*/
bool vpDisplayWin32::getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  // wait if the window is not initialized
  waitForInit();
  bool ret = false;
  double u, v;
  // tells the window there has been a getclickup demand
  //   PostMessage(window.getHWnd(), vpWM_GETCLICK, 0,0);
  // waits for a click
  if (blocking) {
    WaitForSingleObject(window.semaClick, 0);
    WaitForSingleObject(window.semaClickUp, 0); // to erase previous events
    WaitForSingleObject(window.semaClick, INFINITE);
    ret = true;
  }
  else
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClick, 0));

  u = window.clickX;
  v = window.clickY;
  ip.set_u(u * m_scale);
  ip.set_v(v * m_scale);
  button = window.clickButton;

  return ret;
}

/*!
  Wait for a mouse button click release and get the position of the
  image point were the click release occurs.  The button used to click is
  also set. Same method as getClick(unsigned int&, unsigned int&,
  vpMouseButton::vpMouseButtonType &, bool).

  \param[out] ip : Position of the clicked image point.

  \param[in] button : Button used to click.

  \param[in] blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set to \e true.
  - false if no button was clicked. This can occur if blocking is set to \e false.

  \sa getClick(vpImagePoint &, vpMouseButton::vpMouseButtonType &, bool)

*/
bool vpDisplayWin32::getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  // wait if the window is not initialized
  waitForInit();
  bool ret = false;
  double u, v;
  // tells the window there has been a getclickup demand
  //   PostMessage(window.getHWnd(), vpWM_GETCLICKUP, 0,0);

  // waits for a click release
  if (blocking) {
    WaitForSingleObject(window.semaClickUp, 0);
    WaitForSingleObject(window.semaClick, 0); // to erase previous events
    WaitForSingleObject(window.semaClickUp, INFINITE);
    ret = true;
  }
  else
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClickUp, 0));

  u = window.clickXUp;
  v = window.clickYUp;
  ip.set_u(u * m_scale);
  ip.set_v(v * m_scale);
  button = window.clickButtonUp;

  return ret;
}

/*!
  Get a keyboard event.

  \param[in] blocking : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \return
  - true if a key was pressed. This is always the case if blocking is set to \e true.
  - false if no key was pressed. This can occur if blocking is set to \e false.
*/
bool vpDisplayWin32::getKeyboardEvent(bool blocking)
{
  // wait if the window is not initialized
  waitForInit();

  bool ret = false;
  // waits for a keyboard event
  if (blocking) {
    WaitForSingleObject(window.semaKey, 0); // key down
    WaitForSingleObject(window.semaKey, 0); // key up
    WaitForSingleObject(window.semaKey, INFINITE);
    ret = true;
  }
  else {
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaKey, 0));
  }

  return ret;
}

/*!
  Get a keyboard event.

  \param[in] blocking : Blocking behavior.
  - When set to true, this method waits until a key is pressed and then returns always true.
  - When set to false, returns true only if a key is pressed, otherwise returns false.

  \param[out] key : If possible, an ISO Latin-1 character corresponding to the keyboard key.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayWin32::getKeyboardEvent(std::string &key, bool blocking)
{
  // wait if the window is not initialized
  waitForInit();

  bool ret = false;
  // waits for a keyboard event
  if (blocking) {
    WaitForSingleObject(window.semaKey, 0); // key down
    WaitForSingleObject(window.semaKey, 0); // key up
    WaitForSingleObject(window.semaKey, INFINITE);
    ret = true;
  }
  else {
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaKey, 0));
  }
  //  printf("key: %ud\n", window.key);
  std::stringstream ss;
  ss << window.lpString;
  key = ss.str();

  return ret;
}
/*!
  Get the coordinates of the mouse pointer.

  \param[out] ip  : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.

  \exception vpDisplayException::notInitializedError : If the display was not initialized.
*/
bool vpDisplayWin32::getPointerMotionEvent(vpImagePoint &ip)
{
  // wait if the window is not initialized
  waitForInit();

  bool ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaMove, 0));
  if (ret) {
    double u, v;
    // tells the window there has been a getclick demand
    // PostMessage(window.getHWnd(), vpWM_GETPOINTERMOTIONEVENT, 0,0);

    u = window.coordX;
    v = window.coordY;
    ip.set_u(u * m_scale);
    ip.set_v(v * m_scale);
  }

  return ret;
}

/*!
  Get the coordinates of the mouse pointer.

  \param[out] ip  : The coordinates of the mouse pointer.

  \return true.

  \exception vpDisplayException::notInitializedError : If the display was not initialized.
*/
bool vpDisplayWin32::getPointerPosition(vpImagePoint &ip)
{
  // wait if the window is not initialized
  waitForInit();

  bool ret = true;
  double u, v;
  // tells the window there has been a getclick demand
  // PostMessage(window.getHWnd(), vpWM_GETPOINTERMOTIONEVENT, 0,0);

  u = window.coordX;
  v = window.coordY;
  ip.set_u(u * m_scale);
  ip.set_v(v * m_scale);

  return ret;
}

/*!
  Changes the window's position.

  \param[in] winx : Horizontal position of the upper-left window's corner in the screen.
  \param[in] winy : Vertical position of the upper-left window's corner in the screen.
*/
void vpDisplayWin32::setWindowPosition(int winx, int winy)
{
  // wait if the window is not initialized
  waitForInit();

  // cahange the window position only
  SetWindowPos(window.hWnd, HWND_TOP, winx, winy, 0, 0,
               SWP_ASYNCWINDOWPOS | SWP_NOACTIVATE | SWP_NOZORDER | SWP_NOSIZE);
}

/*!
  Changes the window's titlebar text

  \param[in] windowtitle : Window title.
*/
void vpDisplayWin32::setTitle(const std::string &windowtitle)
{
  // wait if the window is not initialized
  waitForInit();
  SetWindowText(window.hWnd, windowtitle.c_str());
}

/*!
  \brief Set the font used to display text.
  \param[in] fontname : Name of the font.
 */

void vpDisplayWin32::setFont(const std::string &fontname)
{
  // Not yet implemented
  (void)fontname;
}

/*!
  \brief Flush the Win32 buffer.

  It's necessary to use this function to see the results of any drawing.
*/
void vpDisplayWin32::flushDisplay()
{
  // waits if the window is not initialized
  waitForInit();

  // sends a message to the window
  PostMessage(window.getHWnd(), vpWM_DISPLAY, 0, 0);
}

/*!
  \brief Flush the Win32 buffer.

  It's necessary to use this function to see the results of any drawing.
*/
void vpDisplayWin32::flushDisplayROI(const vpImagePoint &iP, unsigned int width, unsigned int height)
{
  // waits if the window is not initialized
  waitForInit();
  /*
  Under windows, flushing an ROI takes more time than
  flushing the whole image.
  Therefore, we update the maximum area even when asked to update a region.
  */
  WORD left = static_cast<WORD>(iP.get_u());
  WORD right = static_cast<WORD>(iP.get_u() + width - 1);

  WORD top = static_cast<WORD>(iP.get_v());
  WORD bottom = static_cast<WORD>(iP.get_v() + height - 1);

  // sends a message to the window
  WPARAM wp = MAKEWPARAM(left, right);
  LPARAM lp = MAKELPARAM(top, bottom);

  PostMessage(window.getHWnd(), vpWM_DISPLAY_ROI, wp, lp);
}

/*!
  Display a point at the image point \e ip location.
  \param[in] ip : Point location.
  \param[in] color : Point color.
  \param[in] thickness : Point thickness.
*/
void vpDisplayWin32::displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  if (thickness == 1) {
    window.renderer->setPixel(ip, color);
  }
  else {
    window.renderer->drawRect(ip, thickness * m_scale, thickness * m_scale, color, true, 1);
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param[in] ip1 : Initial line image point.
  \param[in] ip2 : Final line image points.
  \param[in] color : Line color.
  \param[in] thickness : Line thickness.
*/
void vpDisplayWin32::displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                 unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawLine(ip1, ip2, color, thickness);
}

/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.

  \warning This line is a dashed line only if the thickness is equal to 1.

  \param[in] ip1 : Initial line image point.
  \param[in] ip2 : Final line image points.
  \param[in] color : Line color.
  \param[in] thickness : Line thickness.
*/
void vpDisplayWin32::displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                    unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawLine(ip1, ip2, color, thickness, PS_DASHDOT);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param[in] topLeft : Top-left corner of the rectangle.
  \param[in] width : Rectangle width.
  \param[in] height : Rectangle height.
  \param[in] color : Rectangle color.
  \param[in] fill : When set to true fill the rectangle.
  \param[in] thickness : Thickness of the four lines used to display the rectangle.

  \warning The thickness can not be set if the display uses the d3d library.
*/
void vpDisplayWin32::displayRectangle(const vpImagePoint &topLeft, unsigned int width, unsigned int height,
                                      const vpColor &color, bool fill, unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawRect(topLeft, width, height, color, fill, thickness);
}

/*!
  Display a rectangle.

  \param[in] topLeft : Top-left corner of the rectangle.
  \param[in] bottomRight : Bottom-right corner of the rectangle.
  \param[in] color : Rectangle color.
  \param[in] fill : When set to true fill the rectangle.
  \param[in] thickness : Thickness of the four lines used to display the rectangle.

  \warning The thickness can not be set if the display uses the d3d library.
*/
void vpDisplayWin32::displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight,
                                      const vpColor &color, bool fill, unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  unsigned int width = static_cast<unsigned int>(bottomRight.get_j() - topLeft.get_j());
  unsigned int height = static_cast<unsigned int>(bottomRight.get_i() - topLeft.get_i());
  window.renderer->drawRect(topLeft, width, height, color, fill, thickness);
}

/*!
  Display a rectangle.

  \param[in] rectangle : Rectangle characteristics.
  \param[in] color : Rectangle color.
  \param[in] fill : When set to true fill the rectangle.
  \param[in] thickness : Thickness of the four lines used to display the rectangle.

  \warning The thickness can not be set if the display uses the d3d library.
*/
void vpDisplayWin32::displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill, unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  vpImagePoint topLeft;
  topLeft.set_i(rectangle.getTop());
  topLeft.set_j(rectangle.getLeft());
  window.renderer->drawRect(topLeft, static_cast<unsigned int>(rectangle.getWidth()),
                            static_cast<unsigned int>(rectangle.getHeight()), color, fill, thickness);
}

/*!
  Display a circle.
  \param[in] center : Circle center position.
  \param[in] radius : Circle radius.
  \param[in] color : Circle color.
  \param[in] fill : When set to true fill the circle.
  \param[in] thickness : Thickness of the circle. This parameter is only useful when \e fill is set to false.
*/
void vpDisplayWin32::displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                                   unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawCircle(center, radius, color, fill, thickness);
}

/*!
  Displays a string.
  \param[in] ip : its top left point's coordinates
  \param[in] text : The string to display
  \param[in] color : The text's color
*/
void vpDisplayWin32::displayText(const vpImagePoint &ip, const std::string &text, const vpColor &color)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawText(ip, text.c_str(), color);
}

/*!
  Display a cross at the image point \e ip location.
  \param[in] ip : Cross location.
  \param[in] size : Size (width and height) of the cross.
  \param[in] color : Cross color.
  \param[in] thickness : Thickness of the lines used to display the cross.
*/
void vpDisplayWin32::displayCross(const vpImagePoint &ip, unsigned int size, const vpColor &color,
                                  unsigned int thickness)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawCross(ip, size, color, thickness);
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param[in] ip1 : Initial image point.
  \param[in] ip2 : Final image point.
  \param[in] color : Arrow color.
  \param[in] w : Arrow width.
  \param[in] h : Arrow height.
  \param[in] thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayWin32::displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                  unsigned int w, unsigned int h, unsigned int thickness)

{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->drawArrow(ip1, ip2, color, w, h, thickness);
}

/*!
  Clears the display.
  \param[in] color : Color to fill the display with.
*/
void vpDisplayWin32::clearDisplay(const vpColor &color)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->clear(color);
}

/*!
  Closes the display.
  Destroys the window.
*/
void vpDisplayWin32::closeDisplay()
{
  if (m_displayHasBeenInitialized) {
    waitForInit();
    PostMessage(window.getHWnd(), vpWM_CLOSEDISPLAY, 0, 0);
    // if the destructor is called for a reason different than a
    // problem in the thread creation
    if (iStatus) {
      // waits for the thread to end
      WaitForSingleObject(hThread, INFINITE);
      CloseHandle(hThread);
    }
    m_displayHasBeenInitialized = false;
    window.initialized = false;
  }
}

/*!
  Gets the displayed image (if overlay, if any).
  \param[out] I : Image with overlayed drawings.
*/
void vpDisplayWin32::getImage(vpImage<vpRGBa> &I)
{
  // wait if the window is not initialized
  waitForInit();
  window.renderer->getImage(I);
}

/*!
  Gets screen resolution.
  \param[in] w : Horizontal screen resolution.
  \param[in] h : Vertical screen resolution.
 */
void vpDisplayWin32::getScreenSize(unsigned int &w, unsigned int &h)
{
  w = static_cast<unsigned int>(GetSystemMetrics(SM_CXSCREEN));
  h = static_cast<unsigned int>(GetSystemMetrics(SM_CYSCREEN));
}

/*!
  Gets the screen horizontal resolution.
 */
unsigned int vpDisplayWin32::getScreenWidth()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return width;
}

/*!
  Gets the screen vertical resolution.
 */
unsigned int vpDisplayWin32::getScreenHeight()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return height;
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_gui.a(vpDisplayWin32.cpp.o) has no symbols
void dummy_vpDisplayWin32() { }
#endif
