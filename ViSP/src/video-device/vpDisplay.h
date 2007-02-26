/****************************************************************************
 *
 * $Id: vpDisplay.h,v 1.10 2007-02-26 17:26:58 fspindle Exp $
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
// image
#include <visp/vpImage.h>

//color
#include <visp/vpColor.h>
#include <visp/vpMouseButton.h>
#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

/*!
  \file vpDisplay.h
  \brief Generic class for image display, also provide the interface with the image
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
protected:
  unsigned width ;
  unsigned height ;

 protected:

  vpDisplay() ;

 public:
  // get information
  inline  unsigned getWidth() const  { return width ; }
  inline  unsigned getHeight() const { return height ; }

 protected:
  //! initialization
  virtual void init(vpImage<unsigned char> &I,
		    int winx=-1, int winy=-1,
		    char *title=NULL) =0 ;
  //! initialization
  virtual void init(vpImage<vpRGBa> &I,
		    int winx=-1, int winy=-1,
		    char *title=NULL) =0 ;

  //! initialization
  virtual void init(unsigned width, unsigned height,
		    int winx=-1, int winy=-1 ,
		    char *title=NULL) =0;

  virtual void setWindowPosition(int _winx, int _winy) = 0 ;



  virtual void closeDisplay() =0;
  virtual void flushDisplay() =0;
  virtual void flushTitle(const char *string) =0;

  virtual void clearDisplay(vpColor::vpColorType c=vpColor::white)=0 ;

  // display 8bits image
  virtual void displayImage(const vpImage<vpRGBa> &I)=0 ;
  // display 32 bits image
  virtual void displayImage(const vpImage<unsigned char> &I)=0 ;

 private:
  //! get the window pixmap and put it in vpRGBa image
  virtual void getImage(vpImage<vpRGBa> &I) = 0;

 public:
  virtual ~vpDisplay() {;} ;


 protected:
  //! Display a point at coordinates (i,j) in the display window
  virtual void displayPoint(unsigned i, unsigned j,
			    vpColor::vpColorType col) =0;
  //! Display a cross at coordinates (i,j) in the display window
  virtual void displayCross(unsigned i, unsigned j, unsigned size, 
			    vpColor::vpColorType col) =0;
  //! Display a large cross at coordinates (i,j) in the display window
  virtual void displayCrossLarge(unsigned i, unsigned j, unsigned size,
				 vpColor::vpColorType col) =0;
  //! Display a circle at coordinates (i,j) in the display window.
  virtual void displayCircle(unsigned i, unsigned j, unsigned r,
			     vpColor::vpColorType c)=0;
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  virtual void displayLine(unsigned i1, unsigned j1, unsigned i2, unsigned j2,
			   vpColor::vpColorType col, unsigned e=1) =0;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  virtual void displayDotLine(unsigned i1, unsigned j1, 
			      unsigned i2, unsigned j2, 
			      vpColor::vpColorType col, unsigned e=1) =0;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  virtual void displayArrow(unsigned i1,unsigned j1, 
			    unsigned i2, unsigned j2, 
			    vpColor::vpColorType col=vpColor::white,
			    unsigned L=4,unsigned l=2) =0;

  virtual void displayRectangle(unsigned i, unsigned j, 
				unsigned width, unsigned height, 
				vpColor::vpColorType col)=0 ;
  virtual void displayCharString(unsigned i,unsigned j,char *s, 
				 vpColor::vpColorType c=vpColor::green)=0 ;

 public:
  static void displayTitle(const vpImage<unsigned char> &I, 
			   const char *windowtitle);

  //! Display a 8bits image in the display window
  static void display(const vpImage<unsigned char> &I) ;

  //! get the window pixmap and put it in vpRGBa image
  static void getImage(const vpImage<unsigned char> &Is, vpImage<vpRGBa> &Id) ;


  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint(const vpImage<unsigned char> &I,
			   unsigned i,unsigned j,vpColor::vpColorType col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross(const vpImage<unsigned char> &I,
			   unsigned i,unsigned j,
			   unsigned size,vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge(const vpImage<unsigned char> &I,
				unsigned i,unsigned j,
				unsigned size,vpColor::vpColorType col) ;
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle(const vpImage<unsigned char> &I,
			    unsigned i, unsigned j, unsigned r, 
			    vpColor::vpColorType c);
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine(const vpImage<unsigned char> &I,
			  unsigned i1, unsigned j1, unsigned i2, unsigned j2,
			  vpColor::vpColorType col, unsigned e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine(const vpImage<unsigned char> &I,
			     unsigned i1, unsigned j1, 
			     unsigned i2, unsigned j2,
			     vpColor::vpColorType col, unsigned e=1) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow(const vpImage<unsigned char> &I,
			   unsigned i1,unsigned j1, unsigned i2, unsigned j2,
			   vpColor::vpColorType col=vpColor::white,
			   unsigned L=4,unsigned l=2) ;
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayRectangle(const vpImage<unsigned char> &I,
			       unsigned i, unsigned j,
			       unsigned width, unsigned height, 
			       vpColor::vpColorType col);
  //! Display a string
  static void displayCharString(const vpImage<unsigned char> &I,
				unsigned i,unsigned j,char *s, 
				vpColor::vpColorType c) ;

  static void displayFrame(const vpImage<unsigned char> &I,
			   const vpHomogeneousMatrix &cMo,
			   const vpCameraParameters &cam,
			   double size, vpColor::vpColorType col)  ;

  //! flushes the output buffer
  static void flush(const vpImage<unsigned char> &I) ;

  //! Close a display
  static void close(const vpImage<unsigned char> &I) ;

 public:


  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint_uv(const vpImage<unsigned char> &I,
			      unsigned u,unsigned v,
			      vpColor::vpColorType col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross_uv(const vpImage<unsigned char> &I,
			      unsigned u,unsigned v,
			      unsigned size,vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge_uv(const vpImage<unsigned char> &I,
				   unsigned u,unsigned v,
				   unsigned size,vpColor::vpColorType col) ;
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle_uv(const vpImage<unsigned char> &I,
			       unsigned u, unsigned v, unsigned r, 
			       vpColor::vpColorType c);
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine_uv(const vpImage<unsigned char> &I,
			     unsigned u1, unsigned v1, 
			     unsigned u2, unsigned v2,
			     vpColor::vpColorType col, unsigned e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine_uv(const vpImage<unsigned char> &I,
				unsigned u1, unsigned v1,
				unsigned u2, unsigned v2,
				vpColor::vpColorType col, unsigned e=1) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow_uv(const vpImage<unsigned char> &I,
			      unsigned u1,unsigned v1,
			      unsigned u2, unsigned v2,
			      vpColor::vpColorType col=vpColor::white,
			      unsigned L=4,unsigned l=2) ;
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayRectangle_uv(const vpImage<unsigned char> &I, 
				  unsigned u, unsigned v,
				  unsigned width, unsigned height,
				  vpColor::vpColorType col);
  //! Display a string
  static void displayCharString_uv(const vpImage<unsigned char> &I,
				   unsigned u,unsigned v,char *s, 
				   vpColor::vpColorType c) ;

 public:

  static void displayTitle(const vpImage<vpRGBa> &I, const char *windowtitle);

  //! Display a 32bits image in the display window
  static void display(const vpImage<vpRGBa> &I) ;

  //! get the window pixmap and put it in vpRGBa image
  static void getImage(const vpImage<vpRGBa> &Is, vpImage<vpRGBa> &Id) ;


  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint(const vpImage<vpRGBa> &I,
			   unsigned i,unsigned j,vpColor::vpColorType col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross(const vpImage<vpRGBa> &I,
			   unsigned i,unsigned j,
			   unsigned size,vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge(const vpImage<vpRGBa> &I,
				unsigned i,unsigned j,
				unsigned size,vpColor::vpColorType col) ;
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle(const vpImage<vpRGBa> &I,
			    unsigned i, unsigned j, unsigned r, 
			    vpColor::vpColorType c);
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine(const vpImage<vpRGBa> &I,
			  unsigned i1, unsigned j1, unsigned i2, unsigned j2,
			  vpColor::vpColorType col, unsigned e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine(const vpImage<vpRGBa> &I,
			     unsigned i1, unsigned j1, 
			     unsigned i2, unsigned j2,
			     vpColor::vpColorType col, unsigned e=1) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow(const vpImage<vpRGBa> &I,
			   unsigned i1,unsigned j1, unsigned i2, unsigned j2,
			   vpColor::vpColorType col=vpColor::white,
			   unsigned L=4,unsigned l=2) ;
  //! Display a string
  static void displayCharString(const vpImage<vpRGBa> &I,
				unsigned i,unsigned j,char *s, 
				vpColor::vpColorType c) ;
  //! flushes the output buffer
  static void flush(const vpImage<vpRGBa> &I) ;

  //! Close a display
  static void close(const vpImage<vpRGBa> &I) ;

 public:

  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint_uv(const vpImage<vpRGBa> &I,
			      unsigned u,unsigned v, vpColor::vpColorType col);
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross_uv(const vpImage<vpRGBa> &I,
			      unsigned u,unsigned v,
			      unsigned size,vpColor::vpColorType col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge_uv(const vpImage<vpRGBa> &I,
				   unsigned u,unsigned v,
				   unsigned size,vpColor::vpColorType col) ;
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle_uv(const vpImage<vpRGBa> &I,
			       unsigned u, unsigned v, unsigned r, 
			       vpColor::vpColorType c);
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine_uv(const vpImage<vpRGBa> &I,
			     unsigned u1, unsigned v1, 
			     unsigned u2, unsigned v2,
			     vpColor::vpColorType col, unsigned e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine_uv(const vpImage<vpRGBa> &I,
				unsigned u1, unsigned v1, 
				unsigned u2, unsigned v2,
				vpColor::vpColorType col, unsigned e=1) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow_uv(const vpImage<vpRGBa> &I,
			      unsigned u1,unsigned v1,
			      unsigned u2, unsigned v2,
			      vpColor::vpColorType col=vpColor::white, 
			      unsigned L=4,unsigned l=2) ;
  //! Display a string
  static void displayCharString_uv(const vpImage<vpRGBa> &I,
				   unsigned u,unsigned v,char *s, 
				   vpColor::vpColorType c) ;

 public:
  /* Simple interface with the mouse event */
  //!return true way a button is pressed
  virtual bool  getClick(unsigned& i, unsigned& j) =0;
  //!  return true way button is pressed
  virtual bool  getClick(unsigned& i, unsigned& j, 
			 vpMouseButton::vpMouseButtonType& button)=0 ;
  //! return true way  button is released
  virtual bool  getClickUp(unsigned& i, unsigned& j, 
			   vpMouseButton::vpMouseButtonType &button)= 0;
  //! wait for a click
  virtual void  getClick() =0;

 public:
  //!return true way a button is pressed
  static bool  getClick(const vpImage<unsigned char> &I,
			unsigned& i, unsigned& j) ;
  //!  return true way button is pressed
  static  bool  getClick(const vpImage<unsigned char> &I,
			 unsigned& i, unsigned& j, 
			 vpMouseButton::vpMouseButtonType &button) ;
  //! wait for a click
  static void  getClick(const vpImage<unsigned char> &I) ;
  //! return true when  button is released
  static  bool  getClickUp(const vpImage<unsigned char> &I,
			   unsigned& i, unsigned& j, 
			   vpMouseButton::vpMouseButtonType &button) ;


  //!return true when a button is pressed
  static bool  getClick(const vpImage<vpRGBa> &I,
			unsigned& i, unsigned& j) ;
  //!  return true when button is pressed
  static  bool  getClick(const vpImage<vpRGBa> &I,
			 unsigned& i, unsigned& j, 
			 vpMouseButton::vpMouseButtonType &button) ;
  //! wait for a click
  static void  getClick(const vpImage<vpRGBa> &I) ;
  //! return true when button is released
  static  bool  getClickUp(const vpImage<vpRGBa> &I,
			   unsigned& i, unsigned& j, 
			   vpMouseButton::vpMouseButtonType &button) ;
 public:
  //!return true when a button is pressed
  static bool  getClick_uv(const vpImage<unsigned char> &I,
			   unsigned& u, unsigned& v) ;
  //!  return true when button is pressed
  static  bool  getClick_uv(const vpImage<unsigned char> &I,
			    unsigned& u, unsigned& v, 
			    vpMouseButton::vpMouseButtonType &button) ;
  //! return true when button is released
  static  bool  getClickUp_uv(const vpImage<unsigned char> &I,
			      unsigned& u, unsigned& v, 
			      vpMouseButton::vpMouseButtonType &button);


  //!return true when a button is pressed
  static bool  getClick_uv(const vpImage<vpRGBa> &I,
			   unsigned& u, unsigned& v) ;
  //!  return true when button is pressed
  static  bool  getClick_uv(const vpImage<vpRGBa> &I,
			    unsigned& u, unsigned& v, 
			    vpMouseButton::vpMouseButtonType& button) ;
  //! return true when button is released
  static  bool  getClickUp_uv(const vpImage<vpRGBa> &I,
			      unsigned& u, unsigned& v, 
			      vpMouseButton::vpMouseButtonType& button);

 public:


} ;

#endif
