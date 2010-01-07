/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Christophe Collewet
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpDisplayOpenCV_h
#define vpDisplayOpenCV_h

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_OPENCV) )

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDisplay.h>

#include <cv.h>
#include <highgui.h>
#include <cxcore.h>

/*!
  \file vpDisplayOpenCV.h
  \brief Define the OpenCV console to display images.
*/


/*!

  \class vpDisplayOpenCV

  \ingroup ImageGUI

  \brief The vpDisplayOpenCV allows to display image using the opencv library.

  The example below shows how to display an image with this video device.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImagePoint.h>

int main() 
{
#if defined(VISP_HAVE_OPENCV)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
  vpImageIo::readPGM(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");

  vpDisplayOpenCV d; 

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My OpenCV display");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(10);
  topLeftCorner.set_j(50);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Get non blocking keyboard events
  std::cout << "Check keyboard events..." << std::endl; 
  char key[10];
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
#endif
}
  \endcode
*/

class VISP_EXPORT vpDisplayOpenCV: public vpDisplay
{
private:
  //! true if OpenCV display is ready to use
  bool OpenCVinitialized ;
  IplImage* background;
  int window;
  int windowXPosition ; int  windowYPosition ;
  static int count; 
  CvScalar *col ;
  CvScalar cvcolor;
  CvFont *font;
  int fontHeight;  
  int ncol, nrow ;
  int x_lbuttondown ;
  int y_lbuttondown ;
  bool lbuttondown;
  int x_mbuttondown ;
  int y_mbuttondown ;
  bool mbuttondown;
  int x_rbuttondown ;
  int y_rbuttondown ;
  bool rbuttondown;
  int x_lbuttonup ;
  int y_lbuttonup ;
  bool lbuttonup;
  int x_mbuttonup ;
  int y_mbuttonup ;
  bool mbuttonup;
  int x_rbuttonup ;
  int y_rbuttonup ;
  bool rbuttonup;
  
public:
  vpDisplayOpenCV() ;
  vpDisplayOpenCV(int winx, int winy, const char *title=NULL) ;
  vpDisplayOpenCV(vpImage<unsigned char> &I, int winx=-1, int winy=-1,
	       const char *title=NULL) ;
  vpDisplayOpenCV(vpImage<vpRGBa> &I, int winx=-1, int winy=-1,
	       const char *title=NULL) ;

  virtual ~vpDisplayOpenCV() ;

  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    const char *title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	    int winx=-1, int winy=-1,
	    const char *title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    const char *title=NULL) ;
  void getImage(vpImage<vpRGBa> &I) ;

protected:
  void setFont( const char *font );
  void setTitle(const char *title) ;
  void setWindowPosition(int winx, int winy);

  void clearDisplay(vpColor color=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(const vpImagePoint &ip1, 
		    const vpImagePoint &ip2,
		    vpColor color=vpColor::white,
		    unsigned int w=4,unsigned int h=2,
		    unsigned int thickness=1) ;

  void displayCharString(const vpImagePoint &ip, const char *text,
			 vpColor color=vpColor::green) ;

  void displayCircle(const vpImagePoint &center, unsigned int radius,
		     vpColor color,
		     bool fill = false,
		     unsigned int thickness=1);
  void displayCross(const vpImagePoint &ip, unsigned int size,
		    vpColor color, unsigned int thickness=1) ;
  void displayDotLine(const vpImagePoint &ip1, 
		      const vpImagePoint &ip2,
		      vpColor color, unsigned int thickness=1) ;

  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void displayLine(const vpImagePoint &ip1, 
		   const vpImagePoint &ip2,
		   vpColor color, unsigned int thickness=1) ;
  void displayPoint(const vpImagePoint &ip, vpColor color) ;

  void displayRectangle(const vpImagePoint &topLeft,
			unsigned int width, unsigned int height,
			vpColor color, bool fill = false,
			unsigned int thickness=1) ;
  void displayRectangle(const vpImagePoint &topLeft,
			const vpImagePoint &bottomRight,
			vpColor color, bool fill = false,
			unsigned int thickness=1) ;
  void displayRectangle(const vpRect &rectangle,
			vpColor color, bool fill = false,
			unsigned int thickness=1) ;

  void flushDisplay() ;

  bool getClick(bool blocking=true) ;
  bool getClick(vpImagePoint &ip, bool blocking=true) ;
  bool getClick(vpImagePoint &ip,
		 vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClickUp(vpImagePoint &ip,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true) ;

  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }

  bool getKeyboardEvent(bool blocking=true);
  bool getKeyboardEvent(char *string, bool blocking=true);
  bool getPointerMotionEvent (vpImagePoint &ip);

  static void on_mouse( int event, int x, int y, int flags, void* param );
} ;

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
