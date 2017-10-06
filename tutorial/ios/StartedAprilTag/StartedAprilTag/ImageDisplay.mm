//
//  DisplayUtil.m
//  StartedApriTag
//
//  Created by Fabien Spindler on 04/10/2017.
//  Copyright © 2017 MyOrganization. All rights reserved.
//

#import <Foundation/Foundation.h>

#import "ImageDisplay.h"

@implementation ImageDisplay

// UIImage *image = <the image you want to add a line to>
// vpDetectorAprilTag *detector = <AprilTag detector>
// UIColor *color = <the color of the AprilTag contour>
// int tickness = <the tickness of the lines on the AprilTag contour>
+ (UIImage *)display:(UIImage *)image :(vpDetectorBase &)detector :(UIColor*)color :(int)tickness
{
    UIGraphicsBeginImageContext(image.size);
    
    // Draw the original image as the background
    [image drawAtPoint:CGPointMake(0,0)];
    
    // Draw the line on top of original image
    CGContextRef context = UIGraphicsGetCurrentContext();
    CGContextSetLineWidth(context, tickness);
    CGContextSetStrokeColorWithColor(context, [color CGColor]);
    
    for (size_t i = 0; i < detector.getNbObjects() ; i++) {
        std::vector<vpImagePoint> polygon = detector.getPolygon(i);
        for (size_t j = 0; j < polygon.size(); j++) {
            CGContextMoveToPoint(context, polygon[j].get_u(), polygon[j].get_v());
            CGContextAddLineToPoint(context, polygon[(j+1)%polygon.size()].get_u(), polygon[(j+1)%polygon.size()].get_v());
        }
    }
    
    CGContextStrokePath(context);
    
    // Create new image
    UIImage *retImage = UIGraphicsGetImageFromCurrentImageContext();
    
    // Tidy up
    UIGraphicsEndImageContext();
    return retImage;
}

// UIImage *image = <the image you want to add a line to>
// vpHomogeneousMatrix cMo = <Homegeneous transformation>
// vpCameraParameters cam = <Camera parameters>
// double size = <Size of the frame in meter>
// int tickness = <the tickness of the lines describing the frame>
+ (UIImage *)display:(UIImage *)image :(const vpHomogeneousMatrix &)cMo :(const vpCameraParameters &)cam
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

@end
