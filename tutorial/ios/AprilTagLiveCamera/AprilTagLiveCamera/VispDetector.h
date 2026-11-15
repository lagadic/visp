#import <UIKit/UIKit.h>

#ifdef __cplusplus
#import <visp3/visp.h>
#endif

@interface VispDetector : NSObject
- (UIImage *)detectAprilTag: (UIImage*)image px:(float)px py:(float)py;
@end

