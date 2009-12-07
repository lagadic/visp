/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
// image
#include <visp/vpImage.h>

//color
#include <visp/vpColor.h>
#include <visp/vpMouseButton.h>
#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpRect.h>
#include <visp/vpImagePoint.h>

/*!
  \file vpDisplay.h
  \brief Generic class for image display, also provide the interface
  with the image.
*/

/*!

  \class vpDisplay

  \ingroup ImageGUI

  \brief Class that defines generic functionnalities for display.

  The example below shows how to use this class.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayOpenCV.h>

int main()
{
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
#ifdef UNIX
  //vpImageIo::readPGM(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
  vpImageIo::readPGM(I, "/tmp/Klimt.pgm");
#elif WIN32
  vpImageIo::readPGM(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");
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
  d->init(I);

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
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Get non blocking keyboard events
  std::cout << "Check keyboard events..." << std::endl; 
  char key[10]; sprintf(key, "\0");
  bool ret;
  for (int i=0; i< 200; i++) {
    bool ret = vpDisplay::getKeyboardEvent(I, key, false);
    if (ret) 
      std::cout << "keyboard event: key: " << "\"" << key << "\"" << std::endl;
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
*/
class VISP_EXPORT vpDisplay
{
 protected :
  //! display has been initialized
  bool displayHasBeenInitialized ;
  //! display position
  int windowXPosition ;
  //! display position
  int windowYPosition ;
  //! display title
  char *title ;
  char *font;
  unsigned int width ;
  unsigned int height ;

  vpDisplay() ;

  /*!
    Display an arrow from image point \e ip1 to image point \e ip2.
    \param ip1,ip2 : Initial and final image points.
    \param color : Arrow color.
    \param w,h : Width and height of the arrow.
    \param thickness : Thickness of the lines used to display the arrow.
  */
  virtual void displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2,
			    vpColor color=vpColor::white,
			    unsigned int w=4, unsigned int h=2,
			    unsigned int thickness=1) =0;
  /*!
    Display a string at the image point \e ip location.
    
    To select the font used to display the string, use setFont().
    
    \param ip : Upper left image point location of the string in the display.
    \param text : String to display in overlay.
    \param color : String color.
    
    \sa setFont()
  */
  virtual void displayCharString(const vpImagePoint &ip, const char *text,
				 vpColor color=vpColor::green) =0;
  /*!
    Display a circle.
    \param center : Circle center position.
    \param radius : Circle radius.
    \param color : Circle color.
    \param fill : When set to true fill the rectangle.
    \param thickness : Thickness of the circle. This parameter is only useful 
    when \e fill is set to false.
  */
  virtual void displayCircle(const vpImagePoint &center, unsigned int radius,
			     vpColor color,
			     bool fill = false,
			     unsigned int thickness=1) =0;
  /*!
    Display a cross at the image point \e ip location.
    \param ip : Cross location.
    \param size : Size (width and height) of the cross.
    \param color : Cross color.
    \param thickness : Thickness of the lines used to display the cross.
  */
  virtual void displayCross(const vpImagePoint &ip, unsigned int size,
			    vpColor color, 
			    unsigned int thickness=1) =0;
  /*!
    Display a dashed line from image point \e ip1 to image point \e ip2.
    \param ip1,ip2 : Initial and final image points.
    \param color : Line color.
    \param thickness : Dashed line thickness.
  */
  virtual void displayDotLine(const vpImagePoint &ip1, 
			      const vpImagePoint &ip2,
			      vpColor color, 
			      unsigned int thickness=1) =0;
  /*!
    Display a line from image point \e ip1 to image point \e ip2.
    \param ip1,ip2 : Initial and final image points.
    \param color : Line color.
    \param thickness : Line thickness.
  */
  virtual void displayLine(const vpImagePoint &ip1, 
			   const vpImagePoint &ip2,
			   vpColor color, 
			   unsigned int thickness=1) =0;

  /*!
    Display a point at the image point \e ip location.
    \param ip : Point location.
    \param color : Point color.
  */
  virtual void displayPoint(const vpImagePoint &ip, vpColor color) =0;

  /*!  
    Display a rectangle with \e topLeft as the top-left corner and \e
    width and \e height the rectangle size.

    \param topLeft : Top-left corner of the rectangle.
    \param width,height : Rectangle size.
    \param color : Rectangle color.
    \param fill : When set to true fill the rectangle.

    \param thickness : Thickness of the four lines used to display the
    rectangle. This parameter is only useful when \e fill is set to
    false.
  */
  virtual void displayRectangle(const vpImagePoint &topLeft,
				unsigned int width, unsigned int height,
				vpColor color, bool fill = false,
				unsigned int thickness=1)=0 ;
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
  virtual void displayRectangle(const vpImagePoint &topLeft,
				const vpImagePoint &bottomRight,
				vpColor color, bool fill = false,
				unsigned int thickness=1 )=0;
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
  virtual void displayRectangle(const vpRect &rectangle,
				vpColor color, bool fill = false,
				unsigned int thickness=1)=0 ;

 public:
  /*!
    Destructor.
  */
  virtual ~vpDisplay() {;} ;

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
  virtual void setFont(const char *font) =0;
  /*!
    Set the window title.
    \param title : Window title.
  */
  virtual void setTitle(const char *title) =0;
  /*!
    Set the window position in the screen.
    
    \param winx, winy : Position of the upper-left window's border in
    the screen.

  */  
  virtual void setWindowPosition(int winx, int winy) = 0 ;

  /*!
    Set the window backgroud to \e color.
    \param color : Background color.
  */  
  virtual void clearDisplay(vpColor color=vpColor::white) =0 ;
  /*!
    Close the window.
  */
  virtual void closeDisplay() =0;

  /*!
    Initialize the display (size, position and title) of a gray level image.
    
    \param I : Image to be displayed (not that image has to be initialized)
    \param x, y : The window is set at position x,y (column index, row index).
    \param title : Window title.
  */
  virtual void init(vpImage<unsigned char> &I,
		    int x=-1, int y=-1,
		    const char *title=NULL) =0 ;
  /*!  
    Initialize the display (size, position and title) of a color
    image in RGBa format.
    
    \param I : Image to be displayed (not that image has to be initialized)
    \param x, y : The window is set at position x,y (column index, row index).
    \param title : Window title.
  */
  virtual void init(vpImage<vpRGBa> &I,
		    int x=-1, int y=-1,
		    const char *title=NULL) =0 ;

  /*!
    Initialize the display size, position and title.
    
    \param width, height : Width and height of the window.
    \param x, y : The window is set at position x,y (column index, row index).
    \param title : Window title.
  */
  virtual void init(unsigned int width, unsigned int height,
		    int x=-1, int y=-1 ,
		    const char *title=NULL) =0;

  /*!
    Display the gray level image \e I (8bits).

    \warning Display has to be initialized.

    \warning Suppress the overlay drawing.

    \param I : Image to display.

    \sa init(), closeDisplay()
  */  
  virtual void displayImage(const vpImage<unsigned char> &I) =0 ;
  /*!
    Display the color image \e I in RGBa format (32bits).

    \warning Display has to be initialized.

    \warning Suppress the overlay drawing.

    \param I : Image to display.

    \sa init(), closeDisplay()
  */
  virtual void displayImage(const vpImage<vpRGBa> &I) =0 ;
  /*!
    Flushes the display.
    It's necessary to use this function to see the results of any drawing.    
  */  
  virtual void flushDisplay() =0;

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
  virtual bool getClick(bool blocking=true) =0;

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
  virtual bool getClick(vpImagePoint &ip,
			bool blocking=true) =0;
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
  virtual bool getClick(vpImagePoint &ip,
			vpMouseButton::vpMouseButtonType& button,
			bool blocking=true) =0 ;
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
  virtual bool getClickUp(vpImagePoint &ip,
			  vpMouseButton::vpMouseButtonType &button,
			  bool blocking=true) =0;

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
  virtual bool getKeyboardEvent(bool blocking=true) =0;
  /*!
    
    Get a keyboard event.
    
    \param blocking [in] : Blocking behavior.
    - When set to true, this method waits until a key is
      pressed and then returns always true.
    - When set to false, returns true only if a key is
      pressed, otherwise returns false.

    \param string [out]: If possible, an ISO Latin-1 character
    corresponding to the keyboard key.

    \return 
    - true if a key was pressed. This is always the case if blocking is set 
      to \e true.
    - false if no key was pressed. This can occur if blocking is set
      to \e false.
  */
  virtual bool getKeyboardEvent(char *string, bool blocking=true) =0;
  /*!
    Get the coordinates of the mouse pointer.
    
    \param ip [out] : The coordinates of the mouse pointer.
  
    \return true if a pointer motion event was received, false otherwise.

    \exception vpDisplayException::notInitializedError : If the display
    was not initialized.
  */
  virtual bool getPointerMotionEvent (vpImagePoint &ip) =0;

  /*!
    Return the display width.
    \sa getHeight()
  */
  inline  unsigned int getWidth() const  { return width ; }
  /*!
    Return the display height.
    \sa getWidth()
  */
  inline  unsigned int getHeight() const { return height ; }


  /*!
    @name Display functionalities on gray level images.
  */
  static void setFont(const vpImage<unsigned char> &I, const char *font);
  static void setTitle(const vpImage<unsigned char> &I, 
		       const char *windowtitle);
  static void setWindowPosition(const vpImage<unsigned char> &I, 
				int winx, int winy);
  static void setBackground(const vpImage<unsigned char> &I, vpColor color);
  static void close(const vpImage<unsigned char> &I) ;
  static void display(const vpImage<unsigned char> &I) ;
  static void displayArrow(const vpImage<unsigned char> &I,
			   const vpImagePoint &ip1, const vpImagePoint &ip2,
			   vpColor color=vpColor::white,
			   unsigned int w=4, unsigned int h=2,
			   unsigned int thickness=1) ;
  static void displayArrow(const vpImage<unsigned char> &I,
			   int i1, int j1, int i2, int j2,
			   vpColor color=vpColor::white,
			   unsigned int w=4, unsigned int h=2,
			   unsigned int thickness=1) ;
  static void displayCharString(const vpImage<unsigned char> &I,
				const vpImagePoint &ip, const char *string,
				vpColor color) ;
  static void displayCharString(const vpImage<unsigned char> &I,
				int i, int j, const char *string,
				vpColor color) ;
  static void displayCircle(const vpImage<unsigned char> &I,
			    const vpImagePoint &center, unsigned int radius,
			    vpColor color,
			    bool fill = false,
			    unsigned int thickness=1);
  static void displayCircle(const vpImage<unsigned char> &I,
			    int i, int j, unsigned int radius,
			    vpColor color,
			    bool fill = false,
			    unsigned int thickness=1);
  static void displayCross(const vpImage<unsigned char> &I,
			   const vpImagePoint &ip, unsigned int size,
			   vpColor color, 
			   unsigned int thickness=1) ;
  static void displayCross(const vpImage<unsigned char> &I,
			   int i, int j, unsigned int size,
			   vpColor color, 
			   unsigned int thickness=1) ;
  static void displayDotLine(const vpImage<unsigned char> &I,
			     const vpImagePoint &ip1, 
			     const vpImagePoint &ip2,
			     vpColor color, 
			     unsigned int thickness=1) ;
  static void displayDotLine(const vpImage<unsigned char> &I,
			     int i1, int j1, int i2, int j2,
			     vpColor color, 
			     unsigned int thickness=1) ;
  static void displayFrame(const vpImage<unsigned char> &I,
			   const vpHomogeneousMatrix &cMo,
			   const vpCameraParameters &cam,
			   double size, vpColor color)  ;
  static void displayLine(const vpImage<unsigned char> &I,
			  const vpImagePoint &ip1, 
			  const vpImagePoint &ip2,
			  vpColor color, 
			  unsigned int thickness=1) ;
  static void displayLine(const vpImage<unsigned char> &I,
			  int i1, int j1, int i2, int j2,
			  vpColor color, 
			  unsigned int thickness=1) ;
  static void displayPoint(const vpImage<unsigned char> &I,
			   const vpImagePoint &ip,
			   vpColor color) ;
  static void displayPoint(const vpImage<unsigned char> &I,
			   int i, int j,
			   vpColor color) ;
  static void displayRectangle(const vpImage<unsigned char> &I,
			       const vpImagePoint &topLeft,
			       unsigned int width, unsigned int height,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<unsigned char> &I,
			       const vpImagePoint &topLeft,
			       const vpImagePoint &bottomRight,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<unsigned char> &I,
			       const vpRect &rectangle,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<unsigned char> &I,
			       const vpImagePoint &center,
			       float angle,
			       unsigned int width, unsigned int height,
			       vpColor color,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<unsigned char> &I,
			       int i, int j,
			       unsigned int width, unsigned int height,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<unsigned char> &I,
			       unsigned int i, unsigned int j, float angle,
			       unsigned int width, unsigned int height,
			       vpColor color, 
			       unsigned int thickness=1);

  static void flush(const vpImage<unsigned char> &I) ;

  static bool getClick(const vpImage<unsigned char> &I, bool blocking=true) ;
  static bool getClick(const vpImage<unsigned char> &I,
		       vpImagePoint &ip, bool blocking=true) ;
  static bool getClick(const vpImage<unsigned char> &I,
		       vpImagePoint &ip,
		       vpMouseButton::vpMouseButtonType &button,
		       bool blocking=true) ;
  static bool getClickUp(const vpImage<unsigned char> &I,
			 vpImagePoint &ip,
			 vpMouseButton::vpMouseButtonType &button,
			 bool blocking=true) ;
  static void getImage(const vpImage<unsigned char> &Is, vpImage<vpRGBa> &Id) ;

  static bool getKeyboardEvent(const vpImage<unsigned char> &I, 
			       bool blocking=true);
  static bool getKeyboardEvent(const vpImage<unsigned char> &I, 
			       char *string, bool blocking=true);
  static bool getPointerMotionEvent (const vpImage<unsigned char> &I, 
				     vpImagePoint &ip);

  /*!
    @name Display functionalities on color images.
  */
  static void setFont(const vpImage<vpRGBa> &I, const char *font);
  static void setTitle(const vpImage<vpRGBa> &I, const char *windowtitle);
  static void setWindowPosition(const vpImage<vpRGBa> &I, int winx, int winy);
  static void setBackground(const vpImage<vpRGBa> &I, vpColor color);

  static void close(const vpImage<vpRGBa> &I) ;

  static void display(const vpImage<vpRGBa> &I) ;
  static void displayArrow(const vpImage<vpRGBa> &I,
			   const vpImagePoint &ip1, const vpImagePoint &ip2,
			   vpColor color=vpColor::white,
			   unsigned int w=4, unsigned int h=2,
			   unsigned int thickness=1) ;
  static void displayArrow(const vpImage<vpRGBa> &I,
			   int i1, int j1, int i2, int j2,
			   vpColor color=vpColor::white,
			   unsigned int w=4, unsigned int h=2,
			   unsigned int thickness=1) ;
  static void displayCharString(const vpImage<vpRGBa> &I,
				const vpImagePoint &ip, const char *string,
				vpColor color) ;
  static void displayCharString(const vpImage<vpRGBa> &I,
				int i, int j, const char *string,
				vpColor color) ;
  static void displayCircle(const vpImage<vpRGBa> &I,
			    const vpImagePoint &center, unsigned int radius,
			    vpColor color,
			    bool fill = false,
			    unsigned int thickness=1);
  static void displayCircle(const vpImage<vpRGBa> &I,
			    int i, int j, unsigned int radius,
			    vpColor color,
			    bool fill = false,
			    unsigned int thickness=1);
  static void displayCross(const vpImage<vpRGBa> &I,
			   const vpImagePoint &ip, unsigned int size,
			   vpColor color, 
			   unsigned int thickness=1) ;
  static void displayCross(const vpImage<vpRGBa> &I,
			   int i, int j, unsigned int size,
			   vpColor color, 
			   unsigned int thickness=1) ;
  static void displayDotLine(const vpImage<vpRGBa> &I,
			     const vpImagePoint &ip1, 
			     const vpImagePoint &ip2,
			     vpColor color, 
			     unsigned int thickness=1) ;
  static void displayDotLine(const vpImage<vpRGBa> &I,
			     int i1, int j1, int i2, int j2,
			     vpColor color, 
			     unsigned int thickness=1) ;
  static void displayFrame(const vpImage<vpRGBa> &I,
			   const vpHomogeneousMatrix &cMo,
			   const vpCameraParameters &cam,
			   double size, vpColor color)  ;
  static void displayLine(const vpImage<vpRGBa> &I,
			  const vpImagePoint &ip1, 
			  const vpImagePoint &ip2,
			  vpColor color, 
			  unsigned int thickness=1) ;
  static void displayLine(const vpImage<vpRGBa> &I,
			  int i1, int j1, int i2, int j2,
			  vpColor color, 
			  unsigned int thickness=1) ;
  static void displayPoint(const vpImage<vpRGBa> &I,
			   const vpImagePoint &ip,
			   vpColor color) ;
  static void displayPoint(const vpImage<vpRGBa> &I,
			   int i, int j,
			   vpColor color) ;
  static void displayRectangle(const vpImage<vpRGBa> &I,
			       const vpImagePoint &topLeft,
			       unsigned int width, unsigned int height,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<vpRGBa> &I,
			       const vpImagePoint &topLeft,
			       const vpImagePoint &bottomRight,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<vpRGBa> &I,
			       const vpRect &rectangle,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<vpRGBa> &I,
			       const vpImagePoint &center,
			       float angle,
			       unsigned int width, unsigned int height,
			       vpColor color,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<vpRGBa> &I,
			       int i, int j,
			       unsigned int width, unsigned int height,
			       vpColor color, bool fill = false,
			       unsigned int thickness=1);
  static void displayRectangle(const vpImage<vpRGBa> &I,
			       unsigned int i, unsigned int j, 
			       float angle,
			       unsigned int width, unsigned int height,
			       vpColor color, 
			       unsigned int thickness=1);

  static void flush(const vpImage<vpRGBa> &I) ;
  static bool getClick(const vpImage<vpRGBa> &I, bool blocking=true) ;
  static bool getClick(const vpImage<vpRGBa> &I,
		       vpImagePoint &ip, bool blocking=true) ;
  static bool getClick(const vpImage<vpRGBa> &I,
		       vpImagePoint &ip,
		       vpMouseButton::vpMouseButtonType &button,
		       bool blocking=true) ;
  static bool getClickUp(const vpImage<vpRGBa> &I,
			 vpImagePoint &ip,
			 vpMouseButton::vpMouseButtonType &button,
			 bool blocking=true) ;
  static void getImage(const vpImage<vpRGBa> &Is, vpImage<vpRGBa> &Id) ;

  static bool getKeyboardEvent(const vpImage<vpRGBa> &I, 
			       bool blocking=true);
  static bool getKeyboardEvent(const vpImage<vpRGBa> &I, 
			       char *string, bool blocking=true);
  static bool getPointerMotionEvent (const vpImage<vpRGBa> &I, vpImagePoint &ip);



#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  static vp_deprecated void displayTitle(const vpImage<unsigned char> &I,
			   const char *windowtitle);

  static vp_deprecated void displayTitle(const vpImage<vpRGBa> &I, const char *windowtitle);

  static vp_deprecated void displayArrow_uv(const vpImage<unsigned char> &I,
			      int u1, int v1, int u2, int v2,
			      vpColor col=vpColor::white,
			      unsigned int L=4,unsigned int l=2) ;
  static vp_deprecated void displayArrow_uv(const vpImage<vpRGBa> &I,
			      int u1, int v1, int u2, int v2,
			      vpColor col=vpColor::white,
			      unsigned int L=4,unsigned int l=2) ;
  static vp_deprecated void displayCharString_uv(const vpImage<unsigned char> &I,
				   int u, int v, const char *s,
				   vpColor c) ;

  static vp_deprecated void displayCharString_uv(const vpImage<vpRGBa> &I,
				   int u, int v, const char *s,
				   vpColor c) ;

  static vp_deprecated void displayCircle_uv(const vpImage<unsigned char> &I,
			       int u, int v, unsigned int r,
			       vpColor c);
  static vp_deprecated void displayCircle_uv(const vpImage<vpRGBa> &I,
			       int u, int v, unsigned int r,
			       vpColor c);
  static vp_deprecated void displayCross_uv(const vpImage<unsigned char> &I,
			      int u, int v, unsigned int size,
			      vpColor col) ;
  static vp_deprecated void displayCross_uv(const vpImage<vpRGBa> &I,
			      int u, int v, unsigned int size,
			      vpColor col) ;
  static vp_deprecated void displayCrossLarge(const vpImage<unsigned char> &I,
				int i, int j, unsigned int size,
				vpColor col) ;
  static vp_deprecated void displayCrossLarge(const vpImage<vpRGBa> &I,
				int i, int j, unsigned int size,
				vpColor col) ;
  static vp_deprecated void displayCrossLarge_uv(const vpImage<unsigned char> &I,
				   int u, int v, unsigned int size,
				   vpColor col);
  static vp_deprecated void displayCrossLarge_uv(const vpImage<vpRGBa> &I,
				   int u, int v, unsigned int size,
				   vpColor col);
  static vp_deprecated void displayDotLine_uv(const vpImage<unsigned char> &I,
				int u1, int v1, int u2, int v2,
				vpColor col, unsigned int e=1) ;
  static vp_deprecated void displayDotLine_uv(const vpImage<vpRGBa> &I,
				int u1, int v1, int u2, int v2,
				vpColor col, unsigned int e=1) ;
  static vp_deprecated void displayLine_uv(const vpImage<unsigned char> &I,
			     int u1, int v1, int u2, int v2,
			     vpColor col, unsigned int e=1) ;

  static vp_deprecated void displayLine_uv(const vpImage<vpRGBa> &I,
			     int u1, int v1, int u2, int v2,
			     vpColor col, unsigned int e=1) ;
  static vp_deprecated void displayPoint_uv(const vpImage<unsigned char> &I,
			      int u, int v,
			      vpColor col) ;
  static vp_deprecated void displayPoint_uv(const vpImage<vpRGBa> &I,
			      int u, int v,
			      vpColor col);
  static vp_deprecated void displayRectangle_uv(const vpImage<unsigned char> &I,
				  int u, int v,
				  unsigned int width, unsigned int height,
				  vpColor col, bool fill = false,
				  unsigned int e=1);			   
  static void displayRectangle_uv(const vpImage<unsigned char> &I,
			    unsigned int u, unsigned int v, float angle,
			    unsigned int width, unsigned int height,
			    vpColor col, unsigned int e=1);
		
  static vp_deprecated void displayRectangle_uv(const vpImage<vpRGBa> &I,
			    unsigned int u, unsigned int v, float angle,
			    unsigned int width, unsigned int height,
			    vpColor col, unsigned int e=1);

  static vp_deprecated bool getClick(const vpImage<unsigned char> &I,
		       unsigned int& i, unsigned int& j, bool blocking=true) ;
  static vp_deprecated bool getClick(const vpImage<vpRGBa> &I,
			unsigned int& i, unsigned int& j, bool blocking=true) ;
  static vp_deprecated bool getClick_uv(const vpImage<unsigned char> &I,
			  unsigned int& u, unsigned int& v,
			  bool blocking=true);
  static vp_deprecated bool getClick_uv(const vpImage<vpRGBa> &I,
			  unsigned int& u, unsigned int& v,
			  bool blocking=true) ;
  static vp_deprecated bool getClick(const vpImage<unsigned char> &I,
		       unsigned int& i, unsigned int& j,
		       vpMouseButton::vpMouseButtonType &button,
		       bool blocking=true) ;
  static vp_deprecated bool getClick(const vpImage<vpRGBa> &I,
		       unsigned int& i, unsigned int& j,
		       vpMouseButton::vpMouseButtonType &button,
		       bool blocking=true) ;
  static vp_deprecated bool getClick_uv(const vpImage<unsigned char> &I,
			  unsigned int& u, unsigned int& v,
			  vpMouseButton::vpMouseButtonType &button,
			  bool blocking=true) ;
  static vp_deprecated bool getClick_uv(const vpImage<vpRGBa> &I,
			  unsigned int& u, unsigned int& v,
			  vpMouseButton::vpMouseButtonType& button,
			  bool blocking=true) ;
  static vp_deprecated bool getClickUp(const vpImage<unsigned char> &I,
			 unsigned int& i, unsigned int& j,
			 vpMouseButton::vpMouseButtonType &button,
			 bool blocking=true) ;
  static vp_deprecated bool getClickUp(const vpImage<vpRGBa> &I,
			 unsigned int& i, unsigned int& j,
			 vpMouseButton::vpMouseButtonType &button,
			 bool blocking=true) ;
  static vp_deprecated bool getClickUp_uv(const vpImage<unsigned char> &I,
			    unsigned int& u, unsigned int& v,
			    vpMouseButton::vpMouseButtonType &button,
			    bool blocking=true);
  static vp_deprecated bool getClickUp_uv(const vpImage<vpRGBa> &I,
			    unsigned int& u, unsigned int& v,
			    vpMouseButton::vpMouseButtonType& button,
			    bool blocking=true);

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS


 private:
  //! get the window pixmap and put it in vpRGBa image
  virtual void getImage(vpImage<vpRGBa> &I) = 0;

} ;

#endif
