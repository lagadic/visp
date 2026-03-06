This version of libapriltag is a copy of this PR https://github.com/AprilRobotics/apriltag/pull/428

This PR introduces three major improvements to the AprilTag library: native support for ArUco tag families, deep-copy utility functions for better memory management, and an API to retrieve the alternative pose solution during pose estimation.

### 1. Native ArUco Support

Integrated the ArUco dictionaries into the AprilTag ecosystem. This allows users to detect ArUco markers using the robust AprilTag 3 detector without needing external conversion tools.

- Added families: 4x4, 5x5, 6x6, 7x7 (various sizes from 50 to 1000) and ArUcoMIP36h12.
- Updated apriltag_pywrap, apriltag_demo, and opencv_demo to support these new families.
- Added comprehensive tests in test_quick_decode.c.

### 2. Deep-Copy Utilities

Added functions to simplify memory management when detections need to be passed between threads or stored beyond the detector's lifecycle.

- apriltag_detection_copy(): Deep copy of a single detection.
- apriltag_detections_copy(): Deep copy of a zarray_t of detections.
- apriltag_detector_copy(): Clones a detector configuration.

### 3. Pose Ambiguity Handling

Planar markers often have two mathematically plausible pose solutions. I've exposed the second solution from the orthogonal iteration algorithm.

- Added get_second_solution() to apriltag_pose.h.
- This allows developers to use temporal filtering or external constraints to choose the correct orientation when the error difference is small.
