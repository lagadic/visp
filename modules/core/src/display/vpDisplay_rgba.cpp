#include <visp3/core/vpDisplay.h>

#include "vpDisplay_impl.h"

//************************************************************************
// Modifications done in this file should be reported in all vpDisplay_*.cpp files
// that implement other types (unsigned char, vpRGB, vpRGBa)
//************************************************************************

/*!
  Close the display attached to I.
*/
void vpDisplay::close(vpImage<vpRGBa> &I)
{
  vp_display_close(I);
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void
vpDisplay::displayArrow(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                        const vpColor &color, unsigned int w, unsigned int h, unsigned int thickness )
{
  vp_display_display_arrow(I, ip1, ip2, color, w, h, thickness);
}

/*!
  Display an arrow from image point (i1,j1) to  image point (i2,j2).

  \param I : The image associated to the display.
  \param i1,j1 : Initial image point.
  \param i2,j2 : Final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void
vpDisplay::displayArrow(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2,
                        const vpColor &color, unsigned int w, unsigned int h, unsigned int thickness)
{
  vp_display_display_arrow(I, i1, j1, i2, j2, color, w, h, thickness);
}

/*!
  Display the projection of an object camera represented by a cone in
  the image.

  \param I : The image associated to the display.
  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.
  \param cam : Camera intrinsic parameters.
  \param size : Size of the object camera.
  \param color : Color used to display the camera in the image.
  \param thickness : Thickness of the graphics drawing.
*/
void
vpDisplay::displayCamera(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                         const vpCameraParameters &cam, double size, const vpColor &color, unsigned int thickness)
{
  vp_display_display_camera(I, cMo, cam, size, color, thickness);
}

/*!
  Display a string at the image point \e ip location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.

  \sa setFont(), displayText()
*/
void
vpDisplay::displayCharString(const vpImage<vpRGBa> &I, const vpImagePoint &ip,
                             const char *string, const vpColor &color )
{
  vp_display_display_char_string(I, ip, string, color);
}

/*!
  Display a string at the image point (i,j) location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.

  \sa setFont(), displayText()
*/
void
vpDisplay::displayCharString(const vpImage<vpRGBa> &I, int i, int j,
                             const char *string, const vpColor &color)
{
  vp_display_display_char_string(I, i, j, string, color);
}

/*!
  Display a circle.
  \param I : The image associated to the display.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void
vpDisplay::displayCircle(const vpImage<vpRGBa> &I, const vpImagePoint &center, unsigned int radius,
                         const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_circle(I, center, radius, color, fill, thickness);
}

/*!
  Display a circle.
  \param I : The image associated to the display.
  \param i,j : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void
vpDisplay::displayCircle(const vpImage<vpRGBa> &I, int i, int j,  unsigned int radius,
                         const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_circle(I, i, j, radius, color, fill, thickness);
}

/*!
  Display a cross at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross(const vpImage<vpRGBa> &I, const vpImagePoint &ip, unsigned int size,
                             const vpColor &color, unsigned int thickness)
{
  vp_display_display_cross(I, ip, size, color, thickness);
}

/*!
  Display a cross at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross(const vpImage<vpRGBa> &I, int i, int j, unsigned int size,
                             const vpColor &color, unsigned int thickness)
{
  vp_display_display_cross(I, i, j, size, color, thickness);
}

/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, const vpImagePoint &ip1,
                               const vpImagePoint &ip2, const vpColor &color, unsigned int thickness )
{
  vp_display_display_dot_line(I, ip1, ip2, color, thickness);
}

/*!
  Display a dashed line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2,
                               const vpColor &color, unsigned int thickness )
{
  vp_display_display_dot_line(I, i1, j1, i2, j2, color, thickness);
}

/*!
  Display the dashed lines formed by the list of image points
  \param I : The image associated to the display.
  \param ips : List of image points.
  \param closeTheShape : If true, display a dashed line from the first and last image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &ips,
                               const bool closeTheShape, const vpColor &color, unsigned int thickness )
{
  if (ips.size() <= 1)
    return;

  for (size_t i = 0; i < ips.size()-1; i++)
    vp_display_display_dot_line(I, ips[i], ips[i+1], color, thickness);

  if (closeTheShape)
    vp_display_display_dot_line(I, ips.front(), ips.back(), color, thickness);
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e use_centered_moments these parameters
  are:
  - the centered moments expressed in pixels: \f$\mu_{20}, \mu_{11}, \mu_{02}\f$;
  - the major and minor axis lenght in pixels and the excentricity of the ellipse in radians: \f$a, b, e\f$.
  \param use_centered_moments : When false, the parameters coef1, coef2, coef3
  are the parameters \f$a, b, e\f$. When true, the parameters coef1, coef2, coef3 are rather the centered moments
  \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ expressed in pixels. In that case, we compute the parameters \e a, \e b and \e e
  from the centered moments.
  \param color : Drawings color.
  \param thickness : Drawings thickness.

  All the points \f$(u_\theta,v_\theta)\f$ on the ellipse are drawn thanks to its parametric representation:

  \f[ \left(\begin{array}{c}
  u_\theta \\
  v_\theta
  \end{array} \right) = \left(\begin{array}{c}
  u_c \\
  v_c
  \end{array} \right) + \left(\begin{array}{cc}
  \cos(e) & -\sin(e) \\
  \sin(e) & \cos(e)
  \end{array} \right) \left(\begin{array}{c}
  a \cos(\theta) \\
  b \sin(\theta)
  \end{array} \right) \f]

  with \f$0 \leq \theta \leq 2\pi\f$.

  The following example shows how to use for example this function to display the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I);

    vpDisplay::displayEllipse(I, ellipse.getCenter(), ellipse.get_mu20(), ellipse.get_mu11(), ellipse.get_mu02(),
                              true, vpColor::orange, 1);
    vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center,
                               const double &coef1, const double &coef2, const double &coef3,
                               bool use_centered_moments, const vpColor &color, unsigned int thickness)
{
  vpDisplay::displayEllipse(I, center, coef1, coef2, coef3, 0., vpMath::rad(360), use_centered_moments, color, thickness);
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e use_centered_moments these parameters
  are:
  - the centered moments expressed in pixels: \f$\mu_{20}, \mu_{11}, \mu_{02}\f$;
  - the major and minor axis lenght in pixels and the excentricity of the ellipse in radians: \f$a, b, e\f$.
  \param theta1, theta2 : Angles \f$(\theta_1, \theta_2)\f$ in radians used to select a portion of the ellipse.
  If theta1=0 and theta2=vpMath::rad(360) all the ellipse is displayed.
  \param use_centered_moments : When false, the parameters coef1, coef2, coef3
  are the parameters \f$a, b, e\f$. When true, the parameters coef1, coef2, coef3 are rather the centered moments
  \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ expressed in pixels. In that case, we compute the parameters \e a, \e b and \e e
  from the centered moments.
  \param color : Drawings color.
  \param thickness : Drawings thickness.

  All the points \f$(u_\theta,v_\theta)\f$ on the ellipse are drawn thanks to its parametric representation:

  \f[ \left(\begin{array}{c}
  u_\theta \\
  v_\theta
  \end{array} \right) = \left(\begin{array}{c}
  u_c \\
  v_c
  \end{array} \right) + \left(\begin{array}{cc}
  \cos(e) & -\sin(e) \\
  \sin(e) & \cos(e)
  \end{array} \right) \left(\begin{array}{c}
  a \cos(\theta) \\
  b \sin(\theta)
  \end{array} \right) \f]

  with \f$\theta_1 \leq \theta \leq \theta_2\f$.

  The following example shows how to use for example this function to display the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I);

    vpDisplay::displayEllipse(I, ellipse.getCenter(), ellipse.get_mu20(),
                              ellipse.get_mu11(), ellipse.get_mu02(),
                              ellipse.getSmallestAngle(), ellipse.getHighestAngle(),
                              true, vpColor::orange, 1);
    vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center,
                               const double &coef1, const double &coef2, const double &coef3,
                               const double &theta1, const double &theta2, bool use_centered_moments,
                               const vpColor &color, unsigned int thickness)
{
  vp_display_display_ellipse(I, center, coef1, coef2, coef3, theta1, theta2, use_centered_moments, color, thickness);
}

/*!
  Display the projection of an object frame represented by 3 arrows in
  the image.

  \param I : The image associated to the display.
  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.
  \param cam : Camera intrinsic parameters.
  \param size : Size of the object frame.
  \param color : Color used to display the frame in the image.
  \param thickness : the thickness of the line.
  \param offset : Offset in pixels applied to the frame origin location in the image.
*/
void
vpDisplay::displayFrame(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                        const vpCameraParameters &cam, double size, const vpColor &color,
                        unsigned int thickness, const vpImagePoint &offset)
{
  vp_display_display_frame(I, cMo, cam, size, color, thickness, offset);
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                            const vpColor &color,  unsigned int thickness )
{
  vp_display_display_line(I, ip1, ip2, color, thickness);
}

/*!
  Display a line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2,
                            const vpColor &color, unsigned int thickness )
{
  vp_display_display_line(I, i1, j1, i2, j2, color, thickness);
}

/*!
  Display the lines formed by the list of image points.
  \param I : The image associated to the display.
  \param ips : List of image points.
  \param closeTheShape : If true, draw a line from the first and last image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &ips,
                            const bool closeTheShape, const vpColor &color, unsigned int thickness )
{
  if (ips.size() <= 1)
    return;

  for (size_t i = 0; i < ips.size()-1; i++)
    vp_display_display_line(I, ips[i], ips[i+1], color, thickness);

  if (closeTheShape)
    vp_display_display_line(I, ips.front(), ips.back(), color, thickness);
}

/*!
  Display a point at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint(const vpImage<vpRGBa> &I, const vpImagePoint &ip,
                             const vpColor &color, unsigned int thickness )
{
  vp_display_display_point(I, ip, color, thickness);
}

/*!
  Display a point at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint(const vpImage<vpRGBa> &I, int i, int j,
                             const vpColor &color, unsigned int thickness )
{
  vp_display_display_point(I, i, j, color, thickness);

}

/*!
  Display a polygon defined by a vector of image points.
  \param I : The image associated to the display.
  \param vip : Vector of image point that define the vertexes of the polygon.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void
vpDisplay::displayPolygon(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &vip,
                          const vpColor &color, unsigned int thickness)
{
  vp_display_display_polygon(I, vip, color, thickness);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &topLeft,
                            unsigned int width, unsigned int height,
                            const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_rectangle(I, topLeft, width, height, color, fill, thickness);
}

/*!
  Display a rectangle with (i,j) as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param i,j : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I,
                            int i, int j, unsigned int width, unsigned int height,
                            const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_rectangle(I, i, j, width, height, color, fill, thickness);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpRect &rectangle,
                            const vpColor &color, bool fill, unsigned int thickness )
{
  vp_display_display_rectangle(I, rectangle, color, fill, thickness);
}

/*!
  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param center : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle.
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &center,
                            float angle, unsigned int width, unsigned int height,
                            const vpColor &color, unsigned int thickness)
{
  vp_display_display_rectangle(I, center, angle, width, height, color, thickness);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I,
                            const vpImagePoint &topLeft, const vpImagePoint &bottomRight,
                            const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_rectangle(I, topLeft, bottomRight, color, fill, thickness);
}

/*!
  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param i,j : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle.
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I,
                            unsigned int i, unsigned int j, float angle, unsigned int width, unsigned int height,
                            const vpColor &color, unsigned int thickness)
{
  vp_display_display_rectangle(I, i, j, angle, width, height, color, thickness);
}

/*!
  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void
vpDisplay::displayText(const vpImage<vpRGBa> &I, const vpImagePoint &ip,
                       const std::string &s, const vpColor &color )
{
  vp_display_display_text(I, ip, s, color);
}

/*!
  Display a string at the image point (i,j) location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void
vpDisplay::displayText(const vpImage<vpRGBa> &I, int i, int j,
                       const std::string &s, const vpColor &color)
{
  vp_display_display_text(I, i, j, s, color);
}

/*!
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.

  \code
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>

int main() {
  vpImage<vpRGBa> I(240, 380);
  vpDisplayGDI d;
  d.init(I);
  vpDisplay::display(I); // display the image
  vpImagePoint center;
  unsigned int radius = 100;
  vpDisplay::displayCircle(I, center, radius, vpColor::red);

  vpDisplay::flush(I); // Mendatory to display the requested features.
}
  \endcode

  \sa flushROI()
*/
void vpDisplay::flush(const vpImage<vpRGBa> &I)
{
  vp_display_flush(I);
}

/*!
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.

  \sa flush()
*/
void vpDisplay::flushROI(const vpImage<vpRGBa> &I, const vpRect &roi)
{
  vp_display_flush_roi(I, roi);
}

/*!
  Display image \e I.

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), close()
*/
void
vpDisplay::display(const vpImage<vpRGBa> &I)
{
  vp_display_display(I);
}

/*!
  Update the display with the content of the image that is in the region of interest.
  \param I : Image.
  \param roi : Region of interest.
 */
void
vpDisplay::displayROI(const vpImage<vpRGBa> &I, const vpRect &roi)
{
  vp_display_display_roi(I, roi);
}

/*!
  Wait for a click from one of the mouse button.

  \param I [in] : The displayed image.

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
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, bool blocking)
{
  return vp_display_get_click(I, blocking);
}

/*!
  Wait for a click from one of the mouse button and get the position
  of the clicked image point.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
*/
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, vpImagePoint &ip, bool blocking)
{
  return vp_display_get_click(I, ip, blocking);
}

/*!
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

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
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, vpImagePoint &ip,
                         vpMouseButton::vpMouseButtonType& button, bool blocking)
{
  return vp_display_get_click(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise.
*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClick(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The clicked button.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise. If a
  button is released, the location of the mouse pointer is updated in
  \e ip.
*/
bool
vpDisplay::getClickUp(const vpImage<vpRGBa> &I, vpImagePoint &ip,
                      vpMouseButton::vpMouseButtonType& button, bool blocking )
{
  return vp_display_get_click_up(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The clicked button.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise.
*/
bool  vpDisplay::getClickUp(const vpImage<vpRGBa> &I,
                            vpMouseButton::vpMouseButtonType& button, bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClickUp(I, ip, button, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black image

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

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    bool event = vpDisplay::getKeyboardEvent(I, false);
    if (event) {
      std::cout << "A keyboard event was detected" << std::endl;
      cpt_event ++;
    }

    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, bool blocking)
{
  return vp_display_get_keyboard_event(I, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black image

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

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  std::string key;
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    bool event = vpDisplay::getKeyboardEvent(I, key, false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event ++;
    }

    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, std::string &key, bool blocking)
{
  return vp_display_get_keyboard_event(I, key, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black image

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

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  char key[10];
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    bool event = vpDisplay::getKeyboardEvent(I, &key[Ø], false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event ++;
    }

    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, char *key, bool blocking)
{
  return vp_display_get_keyboard_event(I, key, blocking);
}

/*!
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.
*/
bool
vpDisplay::getPointerMotionEvent(const vpImage<vpRGBa> &I, vpImagePoint &ip)
{
  return vp_display_get_pointer_motion_event(I, ip);
}

/*!
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true.
*/
bool
vpDisplay::getPointerPosition(const vpImage<vpRGBa> &I, vpImagePoint &ip)
{
  return vp_display_get_pointer_position(I, ip);
}

/*!
  Set the window background.

  \param I : Image associated to the display window.
  \param color: Background color.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void
vpDisplay::setBackground(const vpImage<vpRGBa> &I, const vpColor &color)
{
  vp_display_set_background(I, color);
}

/*!
  Set the font of a text printed in the display overlay. To print a
  text you may use displayCharString().

  \param I : Image associated to the display window.
  \param fontname : The expected font name.

  \note Under UNIX, the available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \sa displayText()
*/
void
vpDisplay::setFont(const vpImage<vpRGBa> &I, const std::string &fontname)
{
  vp_display_set_font(I, fontname);
}

/*!
  Set the windows title.
  \note This functionality is not implemented when vpDisplayOpenCV is used.

  \param I : Image associated to the display window.
  \param windowtitle : Window title.
*/
void
vpDisplay::setTitle(const vpImage<vpRGBa> &I, const std::string &windowtitle)
{
  vp_display_set_title(I, windowtitle);
}

/*!
  Set the window position in the screen.

  \param I : Image associated to the display window.
  \param winx, winy : Position of the upper-left window's border in the screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void
vpDisplay::setWindowPosition(const vpImage<vpRGBa> &I, int winx, int winy )
{
  vp_display_set_window_position(I, winx, winy);
}

/*!
  Return the value of the down scale factor applied to the image in order to reduce
  the size of the window used to display the image.

  \param I : Image associated to the display window.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
unsigned int
vpDisplay::getDownScalingFactor(const vpImage<vpRGBa> &I)
{
  return vp_display_get_down_scaling_factor(I);
}
