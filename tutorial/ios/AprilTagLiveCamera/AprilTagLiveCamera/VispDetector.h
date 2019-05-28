#import <UIKit/UIKit.h>

@interface VispDetector : NSObject
- (UIImage *)detectAprilTag: (UIImage*)image px:(float)px py:(float)py;
@end

