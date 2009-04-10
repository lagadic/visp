/****************************************************************************
 *
 * $Id: vpDisplayOpenCV.h,v 1.8 2008-12-03 10:25:11 nmelchio Exp $
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
  \brief  Define the OpenCV console to display images
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

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for a click in the display window
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
  vpDisplayOpenCV(vpImage<unsigned char> &I, int winx=-1, int winy=-1,
	       const char *title=NULL) ;
  vpDisplayOpenCV(vpImage<vpRGBa> &I, int winx=-1, int winy=-1,
	       const char *title=NULL) ;

  vpDisplayOpenCV() ;
  virtual ~vpDisplayOpenCV() ;

  void getImage(vpImage<vpRGBa> &I) ;
  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    const char *title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	    int winx=-1, int winy=-1,
	    const char *title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    const char *title=NULL) ;
protected:
  void clearDisplay(vpColor::vpColorType c=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(int i1, int j1, int i2, int j2,
		    vpColor::vpColorType col=vpColor::white,
		    unsigned int L=4, unsigned int l=2) ;
  void displayCharString(int i, int j,const char *s,
			 vpColor::vpColorType c=vpColor::green) ;

  void displayCircle(int i, int j, unsigned int r,
		     vpColor::vpColorType c);
  void displayCross(int i, int j, unsigned int size,
		    vpColor::vpColorType col) ;
  void displayCrossLarge(int i, int j, unsigned int size,
			 vpColor::vpColorType col) ;
  void displayDotLine(int i1, int j1, int i2, int j2,
		      vpColor::vpColorType col, unsigned int e=1) ;

  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void displayLine(int i1, int j1, int i2, int j2,
		   vpColor::vpColorType col, unsigned int e=1) ;

  void displayPoint(int i, int j, vpColor::vpColorType col) ;
  void displayRectangle(int i, int j,
			unsigned int width, unsigned int height,
			vpColor::vpColorType col, bool fill = false,
			unsigned int e=1);
  void displayRectangle(const vpRect &rect,
			vpColor::vpColorType col, bool fill = false,
			unsigned int e=1);
  void flushDisplay() ;
  void setTitle(const char *windowtitle) ;
  void setFont( const char *fontname );
  bool getClick(unsigned int& i, unsigned int& j, bool blocking=true) ;
  bool getClick(unsigned int& i, unsigned int& j,
		 vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClick(bool blocking=true) ;
  bool getClickUp(unsigned int& i, unsigned int& j,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true) ;

  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }

  void setWindowPosition(int winx, int winy);
  static void on_mouse( int event, int x, int y, int flags, void* param );
  /*!
    @name Deprecated functions
  */
  void flushTitle(const char *string) ;
} ;

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
