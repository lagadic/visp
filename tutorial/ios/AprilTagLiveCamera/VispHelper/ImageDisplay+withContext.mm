/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#import "ImageDisplay+withContext.h"

@implementation ImageDisplay (withContext)

//! [display line with context]
// UIColor *color = <the color of the line>
// int tickness = <the tickness of the lines on the AprilTag contour>
+ (void)displayLineWithContext:(CGContextRef)context :(std::vector<vpImagePoint>)polygon :(UIColor*)color :(int)tickness
{
    
    CGContextSetLineWidth(context, tickness);
    CGContextSetStrokeColorWithColor(context, [color CGColor]);
    for (size_t j = 0; j < polygon.size(); j++) {
        
        CGContextMoveToPoint(context, polygon[j].get_u(), polygon[j].get_v());
        CGContextAddLineToPoint(context, polygon[(j+1)%polygon.size()].get_u(), polygon[(j+1)%polygon.size()].get_v());
        
        CGContextStrokePath(context);
    }
    
    return;
}
//! [display line with context]

//! [display frame with context]
// vpHomogeneousMatrix cMo = <Homegeneous transformation>
// vpCameraParameters cam = <Camera parameters>
// double size = <Size of the frame in meter>
// int tickness = <the tickness of the lines describing the frame>
+ (void)displayFrameWithContext:(CGContextRef)context :(const vpHomogeneousMatrix &)cMo :(const vpCameraParameters &)cam
                         :(double) size :(int)tickness
{
    
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
    
    return;
}
//! [display frame with context]

//! [display text]
+ (void)displayText:(NSString*)text :(double)x :(double)y :(int)width :(int)height :(UIColor*)color :(UIColor*)bgColor{
    
    CGRect rect = CGRectMake(x,y,width,height);
    
    NSDictionary *attributes =
    @{
      NSForegroundColorAttributeName : color,
      NSFontAttributeName : [UIFont boldSystemFontOfSize:50],
      NSBackgroundColorAttributeName: bgColor
    };
    
    [text drawInRect:CGRectIntegral(rect) withAttributes:attributes];
    
    return;
}
//! [display text]

@end

