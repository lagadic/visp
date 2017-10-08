//
//  DisplayUtil.h
//  StartedApriTag
//
//  Created by Fabien Spindler on 04/10/2017.
//  Copyright © 2017 MyOrganization. All rights reserved.
//

#import <UIKit/UIKit.h>
#ifdef __cplusplus
#import <visp3/visp.h>
#endif

@interface ImageDisplay : NSObject

+ (UIImage *)displayLine:(UIImage *)image :(vpImagePoint &)ip1 :(vpImagePoint &)ip2 :(UIColor*)color :(int)tickness;
+ (UIImage *)displayFrame:(UIImage *)image :(const vpHomogeneousMatrix &)cMo :(const vpCameraParameters &)cam
                         :(double) size :(int)tickness;

@end
