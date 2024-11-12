rm -rf visp-build-time
mkdir visp-build-time && cd visp-build-time
cmake ../visp -DBUILD_JAVA=OFF -DENABLE_AVX=ON

SECONDS=0
# make -j12
time make -j4
duration=$SECONDS
echo ""
echo "$((duration / 60)) minutes and $((duration % 60)) seconds elapsed."
