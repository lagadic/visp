
/*                                                                -*-c++-*-
 Copyright (C) 1998  IRISA-INRIA Rennes Vista Project

    Contact:
       Eric Marchand
       IRISA-INRIA Rennes
       35042 Rennes Cedex
       France

    email: marchand@irisa.fr
    www  : http://www.irisa.fr/vista

    Auteur :
      Fabien Spindler
      Eric Marchand

    Creation : 1 octobre 1998

*/

#ifndef vpDisplayGTK_h
#define vpDisplayGTK_h

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_GTK) )

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>

#include <gtk/gtk.h>
#include <gdk/gdkrgb.h>

#define THIN		1
#define LARGE		3

/*!
  \file vpDisplayGTK.h
  \brief  Define the GTK console to display images
*/


/*!

  \class vpDisplayGTK

  \brief The  vpDisplayGTK allows to display image using the GTK+ library
  version 1.2.

  \author Christophe Collewet (Christophe.Collewet@irisa.fr),
  imported in ViSP by Eric Marchand (Eric.Marchand@irisa.fr)
  Irisa / Inria Rennes

  The GTK+ 1.2 library has to be available on the system


  \date December 2005
*/

class vpDisplayGTK: public vpDisplay
{
private:
  //! true if GTK display is ready to use
  bool GTKinitialized ;

  GdkWindow *window;
  GdkPixmap *background;
  GdkGC *gc;
  GdkColor blue,red,yellow,green,cyan,magenta,goldenrod,coral,orange,white;
  GdkFont *Police1,*Police2;
  guchar  *vectgtk;
  int windowXPosition ; int  windowYPosition ;
  GdkColor **col ;
  int ncol, nrow ;

protected:
  void setWindowPosition(int _winx, int _winy) { ; }
  inline  int getRows() { return nrows ; }
  inline  int getCols() { return ncols ; }

public:


  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    char *_title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	   int winx=-1, int winy=-1,
	   char *_title=NULL)  ;

  void init(int cols, int rows,
	   int winx=-1, int winy=-1 ,
	   char *_title=NULL) ;
  // only the constructor/destructor are public
public:
  vpDisplayGTK(vpImage<unsigned char> &I, int _winx=-1, int _winy=-1, char *title=NULL) ;
  vpDisplayGTK(vpImage<vpRGBa> &I, int _winx=-1, int _winy=-1, char *title=NULL) ;

  vpDisplayGTK(int _winx, int _winy, char *title=NULL) ;

  vpDisplayGTK() ;
  ~vpDisplayGTK() ;

protected:
  void displayImage(vpImage<vpRGBa> &I) ;
  void displayImage(vpImage<unsigned char> &I) ;
  void displayImage(unsigned char *I) ;

  void closeDisplay() ;
  void flushDisplay() ;
  void flushTitle(const char *string) ;

  void clearDisplay(int c=vpColor::white) ;

  void displayPoint(int x,int y,int col) ;
  void displayCross(int x,int y, int size,int col) ;
  void displayCrossLarge(int x,int y, int size,int col) ;
  void displayCircle(int i, int j, int r, int c);
  void displayLine(int x1, int y1, int x2, int y2, int col, int e=1) ;
  void displayDotLine(int x1, int y1, int x2, int y2, int col, int e=1) ;


  void displayArrow(int i1,int j1, int i2, int j2,
		    int col=1, int L=4,int l=2) ;

  void displayRectangle(int i, int j, int width, int height, int col);
  void displayCharString(int i,int j,char *s, int c=vpColor::green) ;

  bool  getClick(int& i, int& j) ;
  bool  getClick(int& i, int& j, int& button) ;
  void  getClick() ;
  bool  getClickUp(int& i, int& j, int& button) ;

public:

  int  getScreenDepth();
  void getScreenSize(int *xsize, int *ysize);



  void   getImage(vpImage<vpRGBa> &I) ;
} ;

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
