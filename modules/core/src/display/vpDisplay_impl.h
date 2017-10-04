#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMeterPixelConversion.h>

template <class Type> void
vp_display_close(vpImage<Type> &I)
{
  if ( I.display != NULL )
  {
    ( I.display )->closeDisplay();
    I.display = NULL;
  }

}

template <class Type> void
vp_display_display(const vpImage<Type> &I)
{
  if ( I.display != NULL )
  {
    ( I.display )->displayImage ( I );
  }

}

template <class Type> void
vp_display_display_arrow(const vpImage<Type> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                         const vpColor &color, unsigned int w,unsigned int h, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayArrow ( ip1, ip2, color, w, h, thickness );
  }
}

template <class Type> void
vp_display_display_arrow(const vpImage<Type> &I, int i1, int j1, int i2, int j2,
                         const vpColor &color, unsigned int w, unsigned int h, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    vpImagePoint ip1, ip2;
    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    ( I.display )->displayArrow ( ip1, ip2, color, w, h, thickness );
  }
}

template <class Type> void
vp_display_display_camera(const vpImage<Type> &I, const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam, double size, const vpColor &color, unsigned int thickness)
{
  // used by display
  double halfSize = size/2.0;
  vpPoint pt[5];
  pt[0].setWorldCoordinates ( -halfSize,-halfSize,0.0 );
  pt[1].setWorldCoordinates ( halfSize,-halfSize,0.0 );
  pt[2].setWorldCoordinates ( halfSize,halfSize,0.0 );
  pt[3].setWorldCoordinates ( -halfSize,halfSize,0.0 );
  pt[4].setWorldCoordinates ( 0.0,0.0,-size );

  for (int i = 0; i < 5; i++)
    pt[i].track ( cMo );

  vpImagePoint ip, ip_1, ip0;
  vpMeterPixelConversion::convertPoint ( cam, pt[4].p[0], pt[4].p[1], ip0);

  for (int i = 0; i < 4; i++)
  {
    vpMeterPixelConversion::convertPoint ( cam, pt[i].p[0], pt[i].p[1], ip_1);
    vpMeterPixelConversion::convertPoint ( cam, pt[(i+1)%4].p[0], pt[(i+1)%4].p[1], ip);
    vpDisplay::displayLine ( I, ip_1, ip, color, thickness);
    vpDisplay::displayLine ( I, ip0, ip_1, color, thickness);
  }
}



template <class Type> void
vp_display_display_char_string(const vpImage<Type> &I, const vpImagePoint &ip,
                               const char *string, const vpColor &color )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayCharString ( ip, string, color );
  }
}

template <class Type> void
vp_display_display_char_string(const vpImage<Type> &I, int i, int j,
                               const char *string, const vpColor &color)
{
  if ( I.display != NULL )
  {
    vpImagePoint ip;
    ip.set_i( i );
    ip.set_j( j );

    ( I.display )->displayCharString ( ip, string, color );
  }
}

template <class Type> void
vp_display_display_circle(const vpImage<Type> &I, const vpImagePoint &center, unsigned int radius,
                          const vpColor &color, bool fill, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    ( I.display )->displayCircle ( center, radius, color, fill, thickness );
  }
}

template <class Type> void
vp_display_display_circle(const vpImage<Type> &I, int i, int j,  unsigned int radius,
                          const vpColor &color, bool fill, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    vpImagePoint ip;
    ip.set_i( i );
    ip.set_j( j );

    ( I.display )->displayCircle ( ip, radius, color, fill, thickness );
  }
}

template <class Type> void
vp_display_display_cross(const vpImage<Type> &I, const vpImagePoint &ip, unsigned int size,
                         const vpColor &color, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    ( I.display )->displayCross ( ip, size, color, thickness );
  }
}

template <class Type> void
vp_display_display_cross(const vpImage<Type> &I, int i, int j, unsigned int size,
                         const vpColor &color, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    vpImagePoint ip;
    ip.set_i( i );
    ip.set_j( j );

    ( I.display )->displayCross ( ip, size, color, thickness );
  }
}

template <class Type> void
vp_display_display_dot_line(const vpImage<Type> &I, const vpImagePoint &ip1,
                            const vpImagePoint &ip2, const vpColor &color, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayDotLine ( ip1, ip2, color, thickness );
  }
}

template <class Type> void
vp_display_display_dot_line(const vpImage<Type> &I, int i1, int j1, int i2, int j2,
                            const vpColor &color, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    vpImagePoint ip1, ip2;
    ip1.set_i( i1 );
    ip1.set_j( j1 );
    ip2.set_i( i2 );
    ip2.set_j( j2 );
    ( I.display )->displayDotLine ( ip1, ip2, color, thickness );
  }
}

template <class Type> void
vp_display_display_ellipse(const vpImage<Type> &I, const vpImagePoint &center,
                           const double &coef1, const double &coef2, const double &coef3,
                           const double &theta1, const double &theta2, bool use_centered_moments,
                           const vpColor &color, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    double j1, i1;
    vpImagePoint iP11;
    double j2, i2;
    vpImagePoint iP22;
    j1 = j2 = i1 = i2 = 0;
    double a=0., b=0., e=0.;

    double mu20_p = coef1;
    double mu11_p = coef2;
    double mu02_p = coef3;

    if (use_centered_moments) {
      if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

        double val_p = sqrt(vpMath::sqr(mu20_p-mu02_p) + 4*vpMath::sqr(mu11_p));
        a = sqrt((mu20_p + mu02_p + val_p)/2);
        b = sqrt((mu20_p + mu02_p - val_p)/2);

        e = (mu02_p - mu20_p + val_p)/(2*mu11_p);
        e = atan(e);
      }
      else {
        a = sqrt(mu20_p);
        b = sqrt(mu02_p);
        e = 0.;
      }
    }
    else {
      a = coef1;
      b = coef2;
      e = coef3;
    }

    // Approximation of the circumference of an ellipse:
    // [Ramanujan, S., "Modular Equations and Approximations to ,"
    // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
    double t = (a-b)/(a+b);
    double circumference = M_PI*(a+b)*(1 + 3*vpMath::sqr(t)/(10 + sqrt(4 - 3*vpMath::sqr(t))));

    int nbpoints = (int)(floor(circumference/5));
    if (nbpoints < 10)
      nbpoints = 10;
    double incr = 2*M_PI / nbpoints ; // angle increment

    double smallalpha = theta1;
    double highalpha  = theta2;
    double ce = cos(e);
    double se = sin(e);

    double k = smallalpha;
    j1 = a *cos(k) ; // equation of an ellipse
    i1 = b *sin(k) ; // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse ;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    iP11.set_j ( center.get_j() + ce *j1 - se *i1 );
    iP11.set_i ( center.get_i() + se *j1 + ce *i1 );

    while (k+incr<highalpha+incr)
    {
      j2 = a *cos(k+incr) ; // equation of an ellipse
      i2 = b *sin(k+incr) ; // equation of an ellipse

      // to get the coordinates of the point on the shifted ellipse
      iP22.set_j ( center.get_j() + ce *j2 - se *i2 );
      iP22.set_i ( center.get_i() + se *j2 + ce *i2 );

      ( I.display )->displayLine(iP11, iP22, color, thickness);

      iP11 = iP22;

      k += incr;
    }
  }
}

template <class Type> void
vp_display_display_frame(const vpImage<Type> &I, const vpHomogeneousMatrix &cMo,
                         const vpCameraParameters &cam, double size, const vpColor &color,
                         unsigned int thickness, const vpImagePoint &offset)
{
  // used by display
  vpPoint o( 0.0,  0.0,  0.0);
  vpPoint x(size,  0.0,  0.0);
  vpPoint y( 0.0, size,  0.0);
  vpPoint z( 0.0,  0.0, size);

  o.track ( cMo );
  x.track ( cMo );
  y.track ( cMo );
  z.track ( cMo );

  vpImagePoint ipo, ip1;

  if ( color == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo);

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1);
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::red, 4*thickness, 2*thickness, thickness);

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1);
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::green, 4*thickness, 2*thickness, thickness);

    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1);
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::blue, 4*thickness, 2*thickness, thickness);
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo);

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1);
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness);

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1);
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness);

    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1);
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness);
  }
}

template <class Type> void
vp_display_display_line(const vpImage<Type> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                        const vpColor &color,  unsigned int thickness )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayLine ( ip1, ip2, color, thickness );
  }
}

template <class Type> void
vp_display_display_line(const vpImage<Type> &I, int i1, int j1, int i2, int j2,
                        const vpColor &color, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    vpImagePoint ip1, ip2;
    ip1.set_i( i1 );
    ip1.set_j( j1 );
    ip2.set_i( i2 );
    ip2.set_j( j2 );
    ( I.display )->displayLine ( ip1, ip2, color, thickness );
  }
}

template <class Type> void
vp_display_display_point(const vpImage<Type> &I, const vpImagePoint &ip,
                         const vpColor &color, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayPoint ( ip, color, thickness );
  }
}

template <class Type> void
vp_display_display_point(const vpImage<Type> &I, int i, int j,
                         const vpColor &color, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    vpImagePoint ip;
    ip.set_i( i );
    ip.set_j( j );
    ( I.display )->displayPoint ( ip, color, thickness );
  }
}

template <class Type> void
vp_display_display_polygon(const vpImage<Type> &I, const std::vector<vpImagePoint> &vip,
                           const vpColor &color, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    for (unsigned int i=0; i< vip.size(); i++)
      ( I.display )->displayLine ( vip[i], vip[(i+1)%vip.size()], color, thickness );
  }
}

template <class Type> void
vp_display_display_rectangle(const vpImage<Type> &I, const vpImagePoint &topLeft,
                             unsigned int width, unsigned int height,
                             const vpColor &color, bool fill, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    ( I.display )->displayRectangle ( topLeft, width, height, color,
                                      fill, thickness );
  }
}

template <class Type> void
vp_display_display_rectangle(const vpImage<Type> &I, const vpRect &rectangle,
                             const vpColor &color, bool fill, unsigned int thickness )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayRectangle ( rectangle, color, fill, thickness );
  }
}

template <class Type> void
vp_display_display_rectangle(const vpImage<Type> &I, const vpImagePoint &center,
                             float angle, unsigned int width, unsigned int height,
                             const vpColor &color, unsigned int thickness)
{
  if (I.display != NULL)
  {
    double i = center.get_i();
    double j = center.get_j();

    //A, B, C, D, corners of the rectangle clockwise
    vpImagePoint ipa, ipb, ipc, ipd;
    double cosinus = cos(angle);
    double sinus = sin(angle);
    ipa.set_u(j + 0.5*width*cosinus + 0.5*height*sinus);
    ipa.set_v(i + 0.5*width*sinus - 0.5*height*cosinus);
    ipb.set_u(j + 0.5*width*cosinus - 0.5*height*sinus);
    ipb.set_v(i + 0.5*width*sinus + 0.5*height*cosinus);
    ipc.set_u(j - 0.5*width*cosinus - 0.5*height*sinus);
    ipc.set_v(i - 0.5*width*sinus + 0.5*height*cosinus);
    ipd.set_u(j - 0.5*width*cosinus + 0.5*height*sinus);
    ipd.set_v(i - 0.5*width*sinus - 0.5*height*cosinus);

    ( I.display )->displayLine(I, ipa, ipb, color, thickness);
    ( I.display )->displayLine(I, ipa, ipd, color, thickness);
    ( I.display )->displayLine(I, ipc, ipb, color, thickness);
    ( I.display )->displayLine(I, ipc, ipd, color, thickness);
  }
}

template <class Type> void
vp_display_display_rectangle(const vpImage<Type> &I,
                             const vpImagePoint &topLeft, const vpImagePoint &bottomRight,
                             const vpColor &color, bool fill, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    ( I.display )->displayRectangle ( topLeft, bottomRight, color,
                                      fill, thickness );
  }
}

template <class Type> void
vp_display_display_rectangle(const vpImage<Type> &I,
                             int i, int j, unsigned int width, unsigned int height,
                             const vpColor &color, bool fill, unsigned int thickness)
{
  if ( I.display != NULL )
  {
    vpImagePoint topLeft;
    topLeft.set_i( i );
    topLeft.set_j( j );

    ( I.display )->displayRectangle ( topLeft, width, height,
                                      color, fill, thickness );
  }
}

template <class Type> void
vp_display_display_rectangle(const vpImage<Type> &I,
                             unsigned int i, unsigned int j, float angle, unsigned int width, unsigned int height,
                             const vpColor &color,unsigned int thickness)
{
  if (I.display != NULL)
  {
    //A, B, C, D, corners of the rectangle clockwise
    vpImagePoint ipa, ipb, ipc, ipd;
    float cosinus = cos(angle);
    float sinus = sin(angle);
    ipa.set_u(j + 0.5*width*cosinus + 0.5*height*sinus);
    ipa.set_v(i + 0.5*width*sinus - 0.5*height*cosinus);
    ipb.set_u(j + 0.5*width*cosinus - 0.5*height*sinus);
    ipb.set_v(i + 0.5*width*sinus + 0.5*height*cosinus);
    ipc.set_u(j - 0.5*width*cosinus - 0.5*height*sinus);
    ipc.set_v(i - 0.5*width*sinus + 0.5*height*cosinus);
    ipd.set_u(j - 0.5*width*cosinus + 0.5*height*sinus);
    ipd.set_v(i - 0.5*width*sinus - 0.5*height*cosinus);

    ( I.display )->displayLine(I, ipa, ipb, color, thickness);
    ( I.display )->displayLine(I, ipa, ipd, color, thickness);
    ( I.display )->displayLine(I, ipc, ipb, color, thickness);
    ( I.display )->displayLine(I, ipc, ipd, color, thickness);
  }
}

template <class Type> void
vp_display_display_roi(const vpImage<Type> &I, const vpRect &roi)
{
  double top = floor(roi.getTop());
  double left = floor(roi.getLeft());
  double roiheight = floor(roi.getHeight());
  double roiwidth = floor(roi.getWidth());
  double iheight = (double)(I.getHeight());
  double iwidth = (double)(I.getWidth());

  if (top < 0 || top > iheight || left < 0 || left > iwidth || top+roiheight > iheight || left+roiwidth > iwidth)
  {
    throw ( vpException ( vpException::dimensionError, "Region of interest outside of the image" ) );
  }

  if ( I.display != NULL )
  {
    ( I.display )->displayImageROI ( I , vpImagePoint(top,left), (unsigned int)roiwidth,(unsigned int)roiheight );
  }
}

template <class Type> void
vp_display_flush(const vpImage<Type> &I)
{
  if ( I.display != NULL )
  {
    ( I.display )->flushDisplay();
  }
}

template <class Type> void
vp_display_flush_roi(const vpImage<Type> &I, const vpRect &roi)
{
  if ( I.display != NULL )
  {
    ( I.display )->flushDisplayROI(roi.getTopLeft(),(unsigned int)roi.getWidth(),(unsigned int)roi.getHeight());
  }
}

template <class Type> bool
vp_display_get_click(const vpImage<Type> &I, bool blocking)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getClick(blocking);
  }
  return false;
}

template <class Type> bool
vp_display_get_click(const vpImage<Type> &I, vpImagePoint &ip, bool blocking)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getClick ( ip, blocking );
  }
  return false;
}

template <class Type> bool
vp_display_get_click(const vpImage<Type> &I, vpImagePoint &ip,
                     vpMouseButton::vpMouseButtonType& button, bool blocking)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getClick ( ip, button, blocking );
  }
  return false;
}

template <class Type> bool
vp_display_get_click_up(const vpImage<Type> &I, vpImagePoint &ip,
                        vpMouseButton::vpMouseButtonType& button, bool blocking )
{
  if ( I.display != NULL )
  {
    return ( I.display )->getClickUp ( ip, button, blocking );
  }
  return false;
}

template <class Type> bool
vp_display_get_keyboard_event(const vpImage<Type> &I, bool blocking)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getKeyboardEvent ( blocking );
  }
  return false;
}

template <class Type> bool
vp_display_get_keyboard_event(const vpImage<Type> &I, std::string &key, bool blocking)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getKeyboardEvent ( key, blocking );
  }
  return false;
}

template <class Type> bool
vp_display_get_keyboard_event(const vpImage<Type> &I, char *key, bool blocking)
{
  if ( I.display != NULL )
  {
    std::string str;
    bool ret = ( I.display )->getKeyboardEvent ( str, blocking );
    sprintf(key, "%s", str.c_str());
    return ret;
  }
  return false;
}

template <class Type> bool
vp_display_get_pointer_motion_event(const vpImage<Type> &I, vpImagePoint &ip)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getPointerMotionEvent ( ip );
  }
  return false;
}

template <class Type> bool
vp_display_get_pointer_position(const vpImage<Type> &I, vpImagePoint &ip)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getPointerPosition ( ip );
  }
  return false;
}

template <class Type> void
vp_display_set_background(const vpImage<Type> &I, const vpColor &color)
{
  if ( I.display != NULL )
  {
    ( I.display )->clearDisplay ( color );
  }
}

template <class Type> void
vp_display_display_text(const vpImage<Type> &I, const vpImagePoint &ip,
                        const std::string &s, const vpColor &color )
{
  if ( I.display != NULL )
  {
    ( I.display )->displayCharString ( ip, s.c_str(), color );
  }
}

template <class Type> void
vp_display_display_text(const vpImage<Type> &I, int i, int j,
                        const std::string &s, const vpColor &color)
{
  if ( I.display != NULL )
  {
    vpImagePoint ip;
    ip.set_i( i );
    ip.set_j( j );

    ( I.display )->displayCharString ( ip, s.c_str(), color );
  }
}

template <class Type> void
vp_display_set_font(const vpImage<Type> &I, const std::string &fontname )
{
  if ( I.display != NULL )
  {
    ( I.display )->setFont ( fontname);
  }
}


template <class Type> void
vp_display_set_title(const vpImage<Type> &I, const std::string &windowtitle)
{
  if ( I.display != NULL )
  {
    ( I.display )->setTitle ( windowtitle );
  }
}

template <class Type> void
vp_display_set_window_position(const vpImage<Type> &I, int winx, int winy)
{
  if ( I.display != NULL )
  {
    ( I.display )->setWindowPosition ( winx, winy );
  }
}

template <class Type> unsigned int
vp_display_get_down_scaling_factor(const vpImage<Type> &I)
{
  if ( I.display != NULL )
  {
    return ( I.display )->getDownScalingFactor();
  }
  else {
    throw(vpException(vpException::fatalError,"Cannot get the down scaling factor: Display is not initialized"));
  }
}
