

#ifndef vpDisplay_h
#define vpDisplay_h

// image
#include <visp/vpImage.h>

//color
#include <visp/vpColor.h>
#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

/*!
  \file vpDisplay.h
  \brief Generic class for image display, also provide the interface with the image
*/


class vpDisplay
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
  int ncols ;
  int nrows ;

protected:

  vpDisplay() ;

  // get information
  inline  int getRows() const { return nrows ; }
  inline  int getCols() const { return ncols ; }

  //! initialization
  virtual void init(vpImage<unsigned char> &I,
		   int winx=-1, int winy=-1,
		   char *title=NULL) =0 ;
  //! initialization
  virtual void init(vpImage<vpRGBa> &I,
		   int winx=-1, int winy=-1,
		   char *title=NULL) =0 ;

  //! initialization
  virtual void init(int cols, int rows,
		   int winx=-1, int winy=-1 ,
		   char *title=NULL) =0;

  virtual   void setWindowPosition(int _winx, int _winy) = 0 ;



  virtual void closeDisplay() =0;
  virtual void flushDisplay() =0;
  virtual void flushTitle(const char *string) =0;

  virtual void clearDisplay(int c=vpColor::white)=0 ;

  // display 8bits image
  virtual void displayImage(vpImage<vpRGBa> &I)=0 ;
  // display 32 bits image
  virtual void displayImage(vpImage<unsigned char> &I)=0 ;

private:
  //! get the window pixmap and put it in vpRGBa image
  virtual void   getImage(vpImage<vpRGBa> &I) = 0;

public:
  virtual ~vpDisplay() {;} ;


protected:
  //! Display a point at coordinates (i,j) in the display window
  virtual void displayPoint(int i,int j,int col) =0;
  //! Display a cross at coordinates (i,j) in the display window
  virtual void displayCross(int i,int j, int size,int col) =0;
  //! Display a large cross at coordinates (i,j) in the display window
  virtual void displayCrossLarge(int i,int j, int size,int col) =0;
  //! Display a circle at coordinates (i,j) in the display window.
  virtual void displayCircle(int i, int j, int r, int c)=0;
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  virtual void displayLine(int i1, int j1, int i2, int j2, int col, int e=1) =0;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  virtual void displayDotLine(int i1, int j1, int i2, int j2, int col, int e=1) =0;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  virtual void displayArrow(int i1,int j1, int i2, int j2, int col=1, int L=4,int l=2) =0;

  virtual void displayRectangle(int i, int j, int width, int height, int col)=0 ;
  virtual void displayCharString(int i,int j,char *s, int c=vpColor::green)=0 ;

public:

  //! Display a 8bits image in the display window
  static void display(vpImage<unsigned char> &I) ;

  //! get the window pixmap and put it in vpRGBa image
  static void getImage(vpImage<unsigned char> &Is, vpImage<vpRGBa> &Id) ;


  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint(vpImage<unsigned char> &I,
			   int i,int j,int col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross(vpImage<unsigned char> &I,
			   int i,int j,
			   int size,int col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge(vpImage<unsigned char> &I,
				int i,int j,
				int size,int col) ;
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle(vpImage<unsigned char> &I,
			    int i, int j, int r, int c);
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine(vpImage<unsigned char> &I,
			  int i1, int j1, int i2, int j2,
			  int col, int e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine(vpImage<unsigned char> &I,
			     int i1, int j1, int i2, int j2,
			     int col, int e=1) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow(vpImage<unsigned char> &I,
			   int i1,int j1, int i2, int j2,
			   int col=1, int L=4,int l=2) ;
  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayRectangle(vpImage<unsigned char> &I, int i, int j,
			       int width, int height, int col);
  //! Display a string
  static void displayCharString(vpImage<unsigned char> &I,
				int i,int j,char *s, int c) ;

  static void displayFrame(vpImage<unsigned char> &I,
			   vpHomogeneousMatrix &cMo,
			   vpCameraParameters &cam,
			   double size, int col)  ;

  //! flushes the output buffer
  static void flush(vpImage<unsigned char> &I) ;

  //! Close a display
  static void close(vpImage<unsigned char> &I) ;
public:

  //! Display a 32bits image in the display window
  static void display(vpImage<vpRGBa> &I) ;

  //! get the window pixmap and put it in vpRGBa image
  static void getImage(vpImage<vpRGBa> &Is, vpImage<vpRGBa> &Id) ;


  //! Display a point at coordinates (i,j) in the display window
  static void displayPoint(vpImage<vpRGBa> &I,
			   int i,int j,int col) ;
  //! Display a cross at coordinates (i,j) in the display window
  static void displayCross(vpImage<vpRGBa> &I,
			   int i,int j,
			   int size,int col) ;
  //! Display a large cross at coordinates (i,j) in the display window
  static void displayCrossLarge(vpImage<vpRGBa> &I,
				int i,int j,
				int size,int col) ;
  //! Display a circle at coordinates (i,j) in the display window.
  static void displayCircle(vpImage<vpRGBa> &I,
			    int i, int j, int r, int c);
  //! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
  static void displayLine(vpImage<vpRGBa> &I,
			  int i1, int j1, int i2, int j2,
			  int col, int e=1) ;
  //! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  //! window.
  static void displayDotLine(vpImage<vpRGBa> &I,
			     int i1, int j1, int i2, int j2,
			     int col, int e=1) ;

  //! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  //! window
  static void displayArrow(vpImage<vpRGBa> &I,
			   int i1,int j1, int i2, int j2,
			   int col=1, int L=4,int l=2) ;
  //! Display a string
  static void displayCharString(vpImage<vpRGBa> &I,
				int i,int j,char *s, int c) ;
  //! flushes the output buffer
  static void flush(vpImage<vpRGBa> &I) ;


  /* Simple interface with the mouse event */
public:
  enum buttonEnum
    {
      button1,
      button2,
      button3
    } ;
public:
  //!return true way a button is pressed
  virtual bool  getClick(int& i, int& j) =0;
  //!  return true way button is pressed
  virtual bool  getClick(int& i, int& j, int& button)=0 ;
  //! return true way  button is released
  virtual bool  getClickUp(int& i, int& j, int& button) = 0;
  //! wait for a click
  virtual void  getClick() =0;

public:
  //!return true way a button is pressed
  static bool  getClick(vpImage<unsigned char> &I,
			 int& i, int& j) ;
  //!  return true way button is pressed
  static  bool  getClick(vpImage<unsigned char> &I,
			 int& i, int& j, int& button) ;
  //! wait for a click
  static void  getClick(vpImage<unsigned char> &I) ;
  //! return true way  button is released
  static  bool  getClickUp(vpImage<unsigned char> &I,
			   int& i, int& j, int& button) ;


  //!return true way a button is pressed
  static bool  getClick(vpImage<vpRGBa> &I,
			 int& i, int& j) ;
  //!  return true way button is pressed
  static  bool  getClick(vpImage<vpRGBa> &I,
			 int& i, int& j, int& button) ;
  //! wait for a click
  static void  getClick(vpImage<vpRGBa> &I) ;
  //! return true way  button is released
  static  bool  getClickUp(vpImage<vpRGBa> &I,
			   int& i, int& j, int& button) ;

public:


} ;

#endif
