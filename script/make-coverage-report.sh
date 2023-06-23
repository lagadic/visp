# usage: $ script/make-coverage-report.sh path/to/build
# CMake should already have been run once in the build folder, with BUILD_COVERAGE turned on

if test "$#" -ne 1; then
  echo "Script should take the path to the build directory as input, and should be started from the root of the visp source directory"
  exit 1
fi

source_dir=`pwd`
build_dir=$1

if test -d $build_dir; then
  echo "Build directory exists: $build_dir"
else
  echo "Create build directory: $build_dir"
  mkdir -p $build_dir
fi

cd $build_dir
lcov --zerocounters --directory .
cmake $source_dir -DBUILD_COVERAGE=ON -DBUILD_DEPRECATED_FUNCTIONS=OFF
cmake --build . --target all -j$(nproc)
cmake --build . --target test -j$(nproc)
lcov --directory . --capture --output-file visp-coverage.info
lcov --remove visp-coverage.info '/usr/*' "$source_dir/3rdparty/*" "$source_dir/demo/*" "$source_dir/samples/*" "$source_dir/example/*" '*/test/*' '*/private/*' "$source_dir/modules/robot*" "$source_dir/modules/sensor*" --output-file visp-coverage.cleaned
genhtml visp-coverage.cleaned -o coverage

echo "Detected source directory: $source_dir"
echo "Coverage report generated in $build_dir/coverage"
