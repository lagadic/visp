/****************************************************************************
 *
 * $Id: vpDisplay.h,v 1.30 2008-12-03 10:25:11 nmelchio Exp $
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
  vpImageIo::readPGM(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
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

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for a click in the display window
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

 protected:

  vpDisplay() ;

 public:
  virtual ~vpDisplay() {;} ;

  virtual void clearDisplay(vpColor::vpColorType c=vpColor::white)=0 ;
  virtual void closeDisplay() =0;

  // display 8bits image
  virtual void displayImage(const vpImage<vpRGBa> &I)=0 ;
  // display 32 bits image
  virtual void displayImage(const vpImage<unsigned char> &I)=0 ;
  virtual void flushDisplay() =0;
  virtual void setTitle(const char *string) =0;
  virtual void setFont(const char *string) =0;
  // get information
  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }

  //! initialization
  virtual void init(vpImage<unsigned char> &I,
		    int winx=-1, int winy=-1,
		    const char *title=NULL) =0 ;
  //! initialization
  virtual void init(vpImage<vpRGBa> &I,
		    int winx=-1, int winy=-1,
		    const char *title=NULL) =0 ;

  //! initialization
  virtual void init(unsigned int width, unsigned int height,
		    int winx=-1, int winy=-1 ,
		    const char *title=NULL) =0;

  virtual void setWindowPosition(int winx, int winy) = 0 ;

 private:
  //! get the window pixmap and put it in vpRGBa image
  virtual void getImage(vpImage<vpRGBa> &I) = 0;



 protected:
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  virtual void displayArrow(int i1, int j1, int i2, int j2,
			    vpColor::vpColorType col=vpColor::white,
			    unsigned int L=4,unsigned int l=2) =0;
  virtual void displayCharString(int i, int j,const char *s,
				 vpColor::vpColorType c=vpColor::green)=0 ;
  //! Display a circle at coordinates (i,j) in the display window.
  virtual void displayCircle(int i, int j, unsigned int r,
			     vpColor::vpColorType c)=0;
  //! Display a cross at coordinates (i,j) in the display window
  virtual void displayCross(int i, int j, unsigned int size,
			    vpColor::vpColorType col) =0;
  //! Display a large cross at coordinates (i,j) in the display window
  virtual void displayCrossLarge(int i, int j, unsigned int size,
				 vpColor::vpColorType col) =0;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  virtual void displayDotLine(int i1, int j1,
			      int i2, int j2,
			      vpColor::vpColorType col, unsigned int e=1) =0;
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  virtual void displayLine(int i1, int j1,
			   int i2, int j2,
			   vpColor::vpColorType col, unsigned int e=1) =0;

  //! Display a point at coordinates (i,j) in the display window
  virtual void displayPoint(int i, int j,
			    vpColor::vpColorType col) =0;

  virtual void displayRectangle(int i, int j,
				unsigned int width, unsigned int height,
				vpColor::vpColorType col, bool fill = false,
				unsigned int e=1)=0 ;
  virtual void displayRectangle(const vpRect &rect,
				vpColor::vpColorType col, bool fill = false,
				unsigned int e=1)=0 ;

 public:
  //! Close a display
  static void close(const vpImage<unsigned char> &I) ;

  //! Close a display
  static void close(const vpImage<vpRGBa> &I) ;

  //! Display a 8bits image in the display window
  static void display(const vpImage<unsigned char> &I) ;

  //! Display a 32bits image in the display window
  static void display(const vpImage<vpRGBa> &I) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow(const vpImage<unsigned char> &I,
			   int i1, int j1, int i2, int j2,
			   vpColor::vpColorType col=vpColor::white,
			   unsigned int L=4,unsigned int l=2) ;
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow(const vpImage<vpRGBa> &I,
			   int i1, int j1, int i2, int j2,
			   vpColor::vpColorType col=vpColor::white,
			   unsigned int L=4,unsigned int l=2) ;
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow_uv(const vpImage<unsigned char> &I,
			      int u1, int v1, int u2, int v2,
			      vpColor::vpColorType col=vpColor::white,
			      unsigned int L=4,unsigned int l=2) ;
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow_uv(const vpImage<vpRGBa> &I,
			      int u1, int v1, int u2, int v2,
			      vpColor::vpColorType col=vpColor::white,
			      unsigned int L=4,unsigned int l=2) ;
  //! Display a string
  static void displayCharString(const vpImage<unsigned char> &I,
				int i, int j,const char *s,
				vpColor::vpColorType c) ;

  //! Display a string
  static void displayCharString(const vpImage<vpRGBa> &I,
				int i, int j, const char *s,
				vpColor::vpColorType c) ;
  //! Display a string
  static void displayCharString_uv(const vpImage<unsigned char> &I,
				   int u, int v, const char *s,
				   vpColor::vpColorType c) ;

  //! Display a string
  static void displayCharString_uv(const vpImage<vpRGBa> &I,
				   int u, int v, const char *s,
				   vpColor::vpColorType c) ;

  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle(const vpImage<unsigned char> &I,
			    int i, int j, unsigned int r,
			    vpColor::vpColorType c);
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle(const vpImage<vpRGBa> &I,
			    int i, int j, unsigned int r,
			    vpColor::vpColorType c);
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle_uv(const vpImage<unsigned char> &I,
			       int u, int v, unsigned int r,
			       vpColor::vpColorType c);
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle_uv(const vpImage<vpRGBa> &I,
			       int u, int v, unsigned int r,
			       vpColor::vpColorType c);
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross(const vpImage<unsigned char> &I,
			   int i, int j, unsigned int size,
			   vpColor::vpColorType col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross(const vpImage<vpRGBa> &I,
			   int i, int j, unsigned int size,
			   vpColor::vpColorType col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross_uv(const vpImage<unsigned char> &I,
			      int u, int v, unsigned int size,
			      vpColor::vpColorType col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross_uv(const vpImage<vpRGBa> &I,
			      int u, int v, unsigned int size,
			      vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge(const vpImage<unsigned char> &I,
				int i, int j, unsigned int size,
				vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge(const vpImage<vpRGBa> &I,
				int i, int j, unsigned int size,
				vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge_uv(const vpImage<unsigned char> &I,
				   int u, int v, unsigned int size,
				   vpColor::vpColorType col);
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge_uv(const vpImage<vpRGBa> &I,
				   int u, int v, unsigned int size,
				   vpColor::vpColorType col);
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine(const vpImage<unsigned char> &I,
			     int i1, int j1, int i2, int j2,
			     vpColor::vpColorType col, unsigned int e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine(const vpImage<vpRGBa> &I,
			     int i1, int j1, int i2, int j2,
			     vpColor::vpColorType col, unsigned int e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine_uv(const vpImage<unsigned char> &I,
				int u1, int v1, int u2, int v2,
				vpColor::vpColorType col, unsigned int e=1) ;


  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine_uv(const vpImage<vpRGBa> &I,
				int u1, int v1, int u2, int v2,
				vpColor::vpColorType col, unsigned int e=1) ;

  static void displayFrame(const vpImage<unsigned char> &I,
			   const vpHomogeneousMatrix &cMo,
			   const vpCameraParameters &cam,
			   double size, vpColor::vpColorType col)  ;
 static void displayFrame(const vpImage<vpRGBa> &I,
			   const vpHomogeneousMatrix &cMo,
			   const vpCameraParameters &cam,
			   double size, vpColor::vpColorType col)  ;
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine(const vpImage<unsigned char> &I,
			  int i1, int j1, int i2, int j2,
			  vpColor::vpColorType col, unsigned int e=1) ;
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine(const vpImage<vpRGBa> &I,
			  int i1, int j1, int i2, int j2,
			  vpColor::vpColorType col, unsigned int e=1) ;
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine_uv(const vpImage<unsigned char> &I,
			     int u1, int v1, int u2, int v2,
			     vpColor::vpColorType col, unsigned int e=1) ;

  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine_uv(const vpImage<vpRGBa> &I,
			     int u1, int v1, int u2, int v2,
			     vpColor::vpColorType col, unsigned int e=1) ;
  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint(const vpImage<unsigned char> &I,
			   int i, int j,
			   vpColor::vpColorType col) ;
  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint(const vpImage<vpRGBa> &I,
			   int i, int j,
			   vpColor::vpColorType col) ;
  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint_uv(const vpImage<unsigned char> &I,
			      int u, int v,
			      vpColor::vpColorType col) ;
  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint_uv(const vpImage<vpRGBa> &I,
			      int u, int v,
			      vpColor::vpColorType col);


  //! Display a rectangle having (i,j) as top/left corner coordinates
  //! and \e width and \e height.
  static void displayRectangle(const vpImage<unsigned char> &I,
			       int i, int j,
			       unsigned int width, unsigned int height,
			       vpColor::vpColorType col, bool fill = false,
			       unsigned int e=1);
  //! Display a rectangle.
  static void displayRectangle(const vpImage<unsigned char> &I,
			       const vpRect &rect,
			       vpColor::vpColorType col, bool fill = false,
			       unsigned int e=1);

  //! Display a rectangle having (u,v) as top/left corner coordinates,
  //! \e width and \e height.
  static void displayRectangle_uv(const vpImage<unsigned char> &I,
				  int u, int v,
				  unsigned int width, unsigned int height,
				  vpColor::vpColorType col, bool fill = false,
				  unsigned int e=1);
				  
  //! Display a rectangle defined by center, orientation and size.
  static void displayRectangle(const vpImage<unsigned char> &I,
			    unsigned int i, unsigned int j, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col, unsigned int e=1);

  //! Display a rectangle defined by center, orientation and size.
  static void displayRectangle(const vpImage<vpRGBa> &I,
			    unsigned int i, unsigned int j, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col, unsigned int e=1);
			   
  //! Display a rectangle defined by center, orientation and size.
  static void displayRectangle_uv(const vpImage<unsigned char> &I,
			    unsigned int u, unsigned int v, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col, unsigned int e=1);
		
  //! Display a rectangle defined by center, orientation and size.
  static void displayRectangle_uv(const vpImage<vpRGBa> &I,
			    unsigned int u, unsigned int v, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col, unsigned int e=1);

  static void setTitle(const vpImage<unsigned char> &I, 
		       const char *windowtitle);

  static void setTitle(const vpImage<vpRGBa> &I, const char *windowtitle);

  static void setFont(const vpImage<unsigned char> &I, const char *font);

  static void setFont(const vpImage<vpRGBa> &I, const char *font);

  static void setWindowPosition(const vpImage<unsigned char> &I, 
				int winx, int winy);

  static void setWindowPosition(const vpImage<vpRGBa> &I, int winx, int winy);

  //! flushes the output buffer
  static void flush(const vpImage<unsigned char> &I) ;
  //! flushes the output buffer
  static void flush(const vpImage<vpRGBa> &I) ;


 public:
  /* Simple interface with the mouse event */
  //!return true way a button is pressed
  virtual bool getClick(unsigned int& i, unsigned int& j,
			bool blocking=true) =0;
  //!  return true way button is pressed
  virtual bool getClick(unsigned int& i, unsigned int& j,
			vpMouseButton::vpMouseButtonType& button,
			bool blocking=true)=0 ;
  //! return true way  button is released
  virtual bool getClickUp(unsigned int& i, unsigned int& j,
			  vpMouseButton::vpMouseButtonType &button,
			  bool blocking=true)= 0;
  //! wait for a click
  virtual bool getClick(bool blocking=true) =0;

 public:
  //! wait for a click
  static bool getClick(const vpImage<unsigned char> &I, bool blocking=true) ;
  //! wait for a click
  static bool getClick(const vpImage<vpRGBa> &I, bool blocking=true) ;
  //!return true way a button is pressed
  static bool getClick(const vpImage<unsigned char> &I,
		       unsigned int& i, unsigned int& j, bool blocking=true) ;
  //!return true when a button is pressed
  static bool getClick(const vpImage<vpRGBa> &I,
			unsigned int& i, unsigned int& j, bool blocking=true) ;
  //!return true when a button is pressed
  static bool getClick_uv(const vpImage<unsigned char> &I,
			  unsigned int& u, unsigned int& v,
			  bool blocking=true);
  //!return true when a button is pressed
  static bool getClick_uv(const vpImage<vpRGBa> &I,
			  unsigned int& u, unsigned int& v,
			  bool blocking=true) ;
  //!  return true way button is pressed
  static bool getClick(const vpImage<unsigned char> &I,
		       unsigned int& i, unsigned int& j,
		       vpMouseButton::vpMouseButtonType &button,
		       bool blocking=true) ;
  //!  return true when button is pressed
  static bool getClick(const vpImage<vpRGBa> &I,
		       unsigned int& i, unsigned int& j,
		       vpMouseButton::vpMouseButtonType &button,
		       bool blocking=true) ;
  //!  return true when button is pressed
  static bool getClick_uv(const vpImage<unsigned char> &I,
			  unsigned int& u, unsigned int& v,
			  vpMouseButton::vpMouseButtonType &button,
			  bool blocking=true) ;
  //!  return true when button is pressed
  static bool getClick_uv(const vpImage<vpRGBa> &I,
			  unsigned int& u, unsigned int& v,
			  vpMouseButton::vpMouseButtonType& button,
			  bool blocking=true) ;
  //! return true when  button is released
  static bool getClickUp(const vpImage<unsigned char> &I,
			 unsigned int& i, unsigned int& j,
			 vpMouseButton::vpMouseButtonType &button,
			 bool blocking=true) ;


  //! return true when button is released
  static bool getClickUp(const vpImage<vpRGBa> &I,
			 unsigned int& i, unsigned int& j,
			 vpMouseButton::vpMouseButtonType &button,
			 bool blocking=true) ;
  //! return true when button is released
  static bool getClickUp_uv(const vpImage<unsigned char> &I,
			    unsigned int& u, unsigned int& v,
			    vpMouseButton::vpMouseButtonType &button,
			    bool blocking=true);


  //! return true when button is released
  static bool getClickUp_uv(const vpImage<vpRGBa> &I,
			    unsigned int& u, unsigned int& v,
			    vpMouseButton::vpMouseButtonType& button,
			    bool blocking=true);

  //! get the window pixmap and put it in vpRGBa image
  static void getImage(const vpImage<unsigned char> &Is, vpImage<vpRGBa> &Id) ;

  //! get the window pixmap and put it in vpRGBa image
  static void getImage(const vpImage<vpRGBa> &Is, vpImage<vpRGBa> &Id) ;

  /*!
    @name Deprecated functions
  */
  /*! \deprecated Use setTitle() instead. */
  virtual void flushTitle(const char *string) =0;
  static void displayTitle(const vpImage<unsigned char> &I,
			   const char *windowtitle);

  static void displayTitle(const vpImage<vpRGBa> &I, const char *windowtitle);

} ;

#endif
