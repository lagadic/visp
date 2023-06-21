# usage: $ script/make-coverage-report.sh path/to/build
# CMake should already have been run once in the build folder, with BUILD_COVERAGE turned on

if test "$#" -ne 1; then
  echo "Script should take the path to the build directory as input, and should be started from the root of the visp source directory"
  exit 1
fi

source_dir=`pwd`
cd $1
lcov --zerocounters --directory .
cmake --build . --target all -j$(nproc)
cmake --build . --target test
lcov --directory . --capture --output-file visp-coverage.info
lcov --remove visp-coverage.info '/usr/*' "$source_dir/3rdparty/*" "$source_dir/demo/*" "$source_dir/samples/*" "$source_dir/example/*" '*/test/*' --output-file visp-coverage.cleaned
genhtml visp-coverage.cleaned -o coverage

echo "Detected source directory: $source_dir"
echo "Coverage report generated in $1/coverage"
