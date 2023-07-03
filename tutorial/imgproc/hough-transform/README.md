# tutorial-cht

## Running the tutorial-cht software

### On synthetic images

#### Using a JSON file as configuration file
To use a JSON file as configuration, you need to install [JSON for modern C++](https://visp-doc.inria.fr/doxygen/visp-daily/supported-third-parties.html#soft_tool_json) and compile ViSP with it.

To run the software on the synthetic images, please run:
```
$ TARGET=full # or TARGET=half # or TARGET=quarter
$ ./tutorial-circle-hough --input ${TARGET}_disks --config config/detector_${TARGET}.json 
```

#### Using command lines

To run the software on the synthetic images without a JSON configuration file, please run:
```
$ TARGET=full # or TARGET=half # or TARGET=quarter
$ ./tutorial-circle-hough --input ${TARGET}_disks 
```

### On actual images

#### Using a JSON configuration file
To run the software on an actual image, please run:
```
$ ./tutorial-circle-hough --input /path/to/my/image --config config/detector_img.json
```

If the detections seem a bit off, you might need to change the parameters

#### Using command lines

To run the software on an actual image without a JSON configuration file, please run:
```
$ TARGET=full # or TARGET=half # or TARGET=quarter
$ ./tutorial-circle-hough --input /path/to/my/image --gaussian-kernel 5 --gaussian-sigma 1 --canny-thresh -1. --dilatation-repet 1 --center-thresh 200 --radius-bin 2 --radius-thresh 2 --radius-limits 80 90 --merging-thresh 15 2 --circle-perfectness 0.9
```

### On a video

You can use the software to run circle detection on a video saved as a sequence of images that are named `${BASENAME}%d.png`. For instance with `${BASENAME}` = `video_`, you can have the following list of images: `video_0001.png`, `video_0002.png` and so on.

#### Using a JSON file as input

To run the software using a JSON configuration file, please run:
```
$ ./tutorial-circle-hough --input /path/to/video/${BASENAME}%d.png --config config/detector_img.json
```

#### Using program arguments
To run the software using the command arguments, please run:

```
./tutorial-circle-hough --input /path/to/video/${BASENAME}%d.png --gaussian-kernel 5 --gaussian-sigma 1 --canny-thresh -1. --dilatation-repet 1 --center-thresh 200 --radius-bin 2 --radius-thresh 2 --radius-limits 80 90 --merging-thresh 15 2 --circle-perfectness 0.9
```