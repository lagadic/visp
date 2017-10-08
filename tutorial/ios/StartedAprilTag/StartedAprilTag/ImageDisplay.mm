/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
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
 *****************************************************************************/

#import <Foundation/Foundation.h>

#import "ImageDisplay.h"

@implementation ImageDisplay

//! [display line]
// UIImage *image = <the image you want to add a line to>
// vpImagePoint &ip1 = Line first point
// vpImagePoint &ip2 = Line second point
// UIColor *color = <the color of the line>
// int tickness = <the tickness of the lines on the AprilTag contour>
+ (UIImage *)displayLine:(UIImage *)image :(vpImagePoint &)ip1 :(vpImagePoint &)ip2 :(UIColor*)color :(int)tickness
{
  UIGraphicsBeginImageContext(image.size);
  
  // Draw the original image as the background
  [image drawAtPoint:CGPointMake(0,0)];
  
  // Draw the line on top of original image
  CGContextRef context = UIGraphicsGetCurrentContext();
  CGContextSetLineWidth(context, tickness);
  CGContextSetStrokeColorWithColor(context, [color CGColor]);
  
  CGContextMoveToPoint(context, ip1.get_u(), ip1.get_v());
  CGContextAddLineToPoint(context, ip2.get_u(), ip2.get_v());
  
  CGContextStrokePath(context);
  
  // Create new image
  UIImage *retImage = UIGraphicsGetImageFromCurrentImageContext();
  
  // Tidy up
  UIGraphicsEndImageContext();
  return retImage;
}
//! [display line]

//! [display frame]
// UIImage *image = <the image you want to add a line to>
// vpHomogeneousMatrix cMo = <Homegeneous transformation>
// vpCameraParameters cam = <Camera parameters>
// double size = <Size of the frame in meter>
// int tickness = <the tickness of the lines describing the frame>
+ (UIImage *)displayFrame:(UIImage *)image :(const vpHomogeneousMatrix &)cMo :(const vpCameraParameters &)cam
                         :(double) size :(int)tickness
{
  UIGraphicsBeginImageContext(image.size);
  
  // Draw the original image as the background
  [image drawAtPoint:CGPointMake(0,0)];
  
  vpPoint o( 0.0,  0.0,  0.0);
  vpPoint x(size,  0.0,  0.0);
  vpPoint y( 0.0, size,  0.0);
  vpPoint z( 0.0,  0.0, size);
  
  o.track(cMo);
  x.track(cMo);
  y.track(cMo);
  z.track(cMo);
  
  vpImagePoint ipo, ip1;
  
  vpMeterPixelConversion::convertPoint (cam, o.p[0], o.p[1], ipo);
  
  // Draw red line on top of original image
  vpMeterPixelConversion::convertPoint (cam, x.p[0], x.p[1], ip1);
  CGContextRef context = UIGraphicsGetCurrentContext();
  CGContextSetLineWidth(context, tickness);
  CGContextSetStrokeColorWithColor(context, [[UIColor redColor] CGColor]);
  CGContextMoveToPoint(context, ipo.get_u(), ipo.get_v());
  CGContextAddLineToPoint(context, ip1.get_u(), ip1.get_v());
  CGContextStrokePath(context);
  
  // Draw green line on top of original image
  vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
  context = UIGraphicsGetCurrentContext();
  CGContextSetLineWidth(context, tickness);
  CGContextSetStrokeColorWithColor(context, [[UIColor greenColor] CGColor]);
  CGContextMoveToPoint(context, ipo.get_u(), ipo.get_v());
  CGContextAddLineToPoint(context, ip1.get_u(), ip1.get_v());
  CGContextStrokePath(context);
  
  // Draw blue line on top of original image
  vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
  context = UIGraphicsGetCurrentContext();
  CGContextSetLineWidth(context, tickness);
  CGContextSetStrokeColorWithColor(context, [[UIColor blueColor] CGColor]);
  CGContextMoveToPoint(context, ipo.get_u(), ipo.get_v());
  CGContextAddLineToPoint(context, ip1.get_u(), ip1.get_v());
  CGContextStrokePath(context);
  
  // Create new image
  UIImage *retImage = UIGraphicsGetImageFromCurrentImageContext();
  
  // Tidy up
  UIGraphicsEndImageContext();
  return retImage;
}
//! [display frame]

@end
