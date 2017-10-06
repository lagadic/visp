//
//  Conversion.h
//  StartedApriTag
//
//  Created by Fabien Spindler on 04/10/2017.
//  Copyright © 2017 MyOrganization. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

#ifdef __cplusplus
#import <visp3/visp.h>
#endif

@interface ImageConversion : NSObject

+ (vpImage<vpRGBa>)vpImageColorFromUIImage:(UIImage *)image;
+ (vpImage<unsigned char>)vpImageGrayFromUIImage:(UIImage *)image;
+ (UIImage *)UIImageFromVpImageColor:(const vpImage<vpRGBa> &)I;
+ (UIImage *)UIImageFromVpImageGray:(const vpImage<unsigned char> &)I;

@end
