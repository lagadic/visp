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

#import "ImageConversion.h"

@implementation ImageConversion


//! [vpImageColorFromUIImage]
// Converts an UIImage that could be in gray or color into a ViSP color image
+ (vpImage<vpRGBa>)vpImageColorFromUIImage:(UIImage *)image
{
  CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
  
  if (CGColorSpaceGetModel(colorSpace) == kCGColorSpaceModelMonochrome) {
    NSLog(@"Input UIImage is grayscale");
    vpImage<unsigned char> gray(image.size.height, image.size.width); // 8 bits per component, 1 channel
    
    CGContextRef contextRef = CGBitmapContextCreate(gray.bitmap,                // pointer to  data
                                                    image.size.width,           // width of bitmap
                                                    image.size.height,          // height of bitmap
                                                    8,                          // bits per component
                                                    image.size.width,           // bytes per row
                                                    colorSpace,                 // colorspace
                                                    kCGImageAlphaNone |
                                                    kCGBitmapByteOrderDefault); // bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
    CGContextRelease(contextRef);
    
    vpImage<vpRGBa> color;
    vpImageConvert::convert(gray, color);
    
    return color;
  }
  else {
    NSLog(@"Input UIImage is color");
    vpImage<vpRGBa> color(image.size.height, image.size.width); // 8 bits per component, 4 channels
    
    colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGContextRef contextRef = CGBitmapContextCreate(color.bitmap,               // pointer to  data
                                                    image.size.width,           // width of bitmap
                                                    image.size.height,          // height of bitmap
                                                    8,                          // bits per component
                                                    4 * image.size.width,       // bytes per row
                                                    colorSpace,                 // colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
    CGContextRelease(contextRef);
    
    return color;
  }
}
//! [vpImageColorFromUIImage]

//! [vpImageGrayFromUIImage]
// Converts an UIImage that could be in gray or color into a ViSP gray image
+ (vpImage<unsigned char>)vpImageGrayFromUIImage:(UIImage *)image
{
  CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
  
  if (CGColorSpaceGetModel(colorSpace) == kCGColorSpaceModelMonochrome) {
    NSLog(@"Input UIImage is grayscale");
    vpImage<unsigned char> gray(image.size.height, image.size.width); // 8 bits per component, 1 channel
    
    CGContextRef contextRef = CGBitmapContextCreate(gray.bitmap,                // pointer to  data
                                                    image.size.width,           // width of bitmap
                                                    image.size.height,          // height of bitmap
                                                    8,                          // bits per component
                                                    image.size.width,           // bytes per row
                                                    colorSpace,                 // colorspace
                                                    kCGImageAlphaNone |
                                                    kCGBitmapByteOrderDefault); // bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
    CGContextRelease(contextRef);
    
    return gray;
  } else {
    NSLog(@"Input UIImage is color");
    vpImage<vpRGBa> color(image.size.height, image.size.width); // 8 bits per component, 4 channels (color channels + alpha)
    
    colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGContextRef contextRef = CGBitmapContextCreate(color.bitmap,               // pointer to  data
                                                    image.size.width,           // width of bitmap
                                                    image.size.height,          // height of bitmap
                                                    8,                          // bits per component
                                                    4 * image.size.width,       // bytes per row
                                                    colorSpace,                 // colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
    CGContextRelease(contextRef);
    
    vpImage<unsigned char> gray;
    vpImageConvert::convert(color, gray);
    
    return gray;
  }
}
//! [vpImageGrayFromUIImage]

//! [UIImageFromVpImageColor]
// Converts a color ViSP image into a color UIImage
+ (UIImage *)UIImageFromVpImageColor:(const vpImage<vpRGBa> &)I
{
  NSData *data = [NSData dataWithBytes:I.bitmap length:I.getSize()*4];
  CGColorSpaceRef colorSpace;
  
  colorSpace = CGColorSpaceCreateDeviceRGB();
  
  CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
  
  // Creating CGImage from vpImage
  CGImageRef imageRef = CGImageCreate(I.getWidth(),                               // width
                                      I.getHeight(),                              // height
                                      8,                                          // bits per component
                                      8 * 4,                                      // bits per pixel
                                      4 * I.getWidth(),                           // bytesPerRow
                                      colorSpace,                                 // colorspace
                                      kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                      provider,                                   // CGDataProviderRef
                                      NULL,                                       // decode
                                      false,                                      // should interpolate
                                      kCGRenderingIntentDefault                   // intent
                                      );
  
  
  // Getting UIImage from CGImage
  UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
  CGImageRelease(imageRef);
  CGDataProviderRelease(provider);
  CGColorSpaceRelease(colorSpace);
  
  return finalImage;
}
//! [UIImageFromVpImageColor]

//! [UIImageFromVpImageGray]
// Converts a gray level ViSP image into a gray level UIImage
+ (UIImage *)UIImageFromVpImageGray:(const vpImage<unsigned char> &)I
{
  NSData *data = [NSData dataWithBytes:I.bitmap length:I.getSize()];
  CGColorSpaceRef colorSpace;
  
  colorSpace = CGColorSpaceCreateDeviceGray();
  
  CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
  
  // Creating CGImage from vpImage
  CGImageRef imageRef = CGImageCreate(I.getWidth(),                               // width
                                      I.getHeight(),                              // height
                                      8,                                          // bits per component
                                      8,                                          // bits per pixel
                                      I.getWidth(),                               // bytesPerRow
                                      colorSpace,                                 // colorspace
                                      kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                      provider,                                   // CGDataProviderRef
                                      NULL,                                       // decode
                                      false,                                      // should interpolate
                                      kCGRenderingIntentDefault                   // intent
                                      );
  
  
  // Getting UIImage from CGImage
  UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
  CGImageRelease(imageRef);
  CGDataProviderRelease(provider);
  CGColorSpaceRelease(colorSpace);
  
  return finalImage;
}
//! [UIImageFromVpImageGray]

@end
