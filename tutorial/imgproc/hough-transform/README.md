# tutorial-cht

## Running the circle detection tutorial

To get the list of the available options, please run:
```
$ ./tutorial-circle-hough --help
```

### On an image

#### Using a JSON configuration file
To use a JSON file as configuration, you need to install [JSON for modern C++](https://visp-doc.inria.fr/doxygen/visp-daily/supported-third-parties.html#soft_tool_json) and compile ViSP with it.

To run the software on the image `coins2.jpg` provided with the tutorial, please run:
```
$ ./tutorial-circle-hough --input coins2.jpg --config config/detector_img.json
```

The configuration file `config/detector_img.json` has been tuned to detect circles in the image `coins2.jpg`.
If the detections seem a bit off on your own image, you might need to change the parameters in this file.

#### Using command lines

To run the software without a JSON configuration file, please run:
```
$ ./tutorial-circle-hough --input coins2.jpg \
    --averaging-window-size 5 \
    --canny-backend opencv-backend \
    --filtering-type gaussianblur+scharr-filtering \
    --canny-thresh -1 -1 \
    --lower-canny-ratio 0.6 \
    --upper-canny-ratio 0.9 \
    --gaussian-kernel 5 \
    --gaussian-sigma 1 \
    --dilatation-kernel-size 5 \
    --center-thresh 70 \
    --circle-probability-thresh 0.725 \
    --radius-limits 34 75 \
    --merging-thresh 5 5 \
    --circle-perfectness 0.65 \
    --center-xlim 0 1920 \
    --center-ylim 0 1080 \
    --expected-nb-centers -1 \
    --edge-filter 3 \
    --gradient-kernel 3
```

The default values of the program correspond to these fine-tuned parameters. Running the program without any
argument should give the same result:
```
$ ./tutorial-circle-hough
```

### On a video

You can use the software to run circle detection on a video saved as a sequence of images that are named `${BASENAME}%d.png`. For instance with `${BASENAME}` = `video_`, you can have the following list of images: `video_0001.png`, `video_0002.png` and so on.

#### Using a JSON configuration file

To run the software using a JSON configuration file, please run:
```
$ ./tutorial-circle-hough --input /path/to/video/${BASENAME}%d.png --config config/detector_img.json
```

#### Using command lines

To run the software using the command line arguments, please run:
```
$ ./tutorial-circle-hough --input /path/to/video/${BASENAME}%d.png \
    --averaging-window-size 5 \
    --canny-backend opencv-backend \
    --filtering-type gaussianblur+scharr-filtering \
    --canny-thresh -1 -1 \
    --lower-canny-ratio 0.6 \
    --upper-canny-ratio 0.9 \
    --gaussian-kernel 5 \
    --gaussian-sigma 1 \
    --dilatation-kernel-size 5 \
    --center-thresh 70 \
    --circle-probability-thresh 0.725 \
    --radius-limits 34 75 \
    --merging-thresh 5 5 \
    --circle-perfectness 0.65 \
    --center-xlim 0 1920 \
    --center-ylim 0 1080 \
    --expected-nb-centers -1 \
    --edge-filter 3 \
    --gradient-kernel 3
```

See the [circle Hough transform tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-imgproc-cht.html) for more details.
