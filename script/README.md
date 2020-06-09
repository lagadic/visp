# Useful scripts

This folder contains scripts that may help ViSP user and developer.

## PerfCompare.py

Python script useful to compare benchmark performances before and after a potential source code modification/optimization or to compare performances using different third-party libraries.

For example in `$VISP_WS/visp-build/module/core` let us consider the `perfMatrixMultiplication` binary.

Depending on your python installation you may install support for enumerations:

```
$ pip install enum
```

This script works like this:

- run first the performance binary activating benchmark and xml output options:

    ```
    $ cd $VISP_WS/visp-build/module/core
    $ ./perfMatrixMultiplication --benchmark --reporter xml --out /tmp/log_perfMatrixMultiplication_MKL.xml
    $ ./perfMatrixMultiplication --benchmark --reporter xml --out /tmp/log_perfMatrixMultiplication_OpenBLAS.xml
    ```

- Use the script to compare performances:

    ```
    $ cd $VISP_WS/visp/script
    $ python PerfCompare.py --before /tmp/log_perfMatrixMultiplication_OpenBLAS.xml \
                            --after /tmp/log_perfMatrixMultiplication_MKL.xml       \
                            --before-label OpenBLAS --after-label MKL
    ```
    
## create_module.py

This script allows to create the structure of a new ViSP module as explained in this [tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-contrib-module.html).

## format-coding-style.sh

This script uses `clang-format` to format ViSP source code. It should be used carefully since a lot of file could be affected.

- To install `clang-format`:

  - On Ubuntu like OS:

      ```
      $ sudo apt-get install clang-format
      ```
    - On OSX:

      ```
      $ brew install clang-format
      ```
      
- To run this script:

    ```
    $ cd $VISP_WS/visp/script
    $ sh format-coding-style.sh
    ```
    
- To format a single file with extension `*.h` or `*.cpp`, you may rather run:

    ```
    $ cd $VISP_WS/visp
    $ clang-format -i <path to file.[h,cpp]>
    ```
