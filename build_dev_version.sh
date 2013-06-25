#!/bin/sh
COVERAGE_FOLDER_NAME=coverage
coverage=1

if [  -d $COVERAGE_FOLDER_NAME ]; then
 rm -r $COVERAGE_FOLDER_NAME
fi
if [  -d doc/html ]; then
 rm -r doc/html
fi
if [ ! -d "build" ]; then
 mkdir build
fi
cd build
cmake ../ -DCMAKE_BUILD_TYPE="Debug" -DWITH_COVERAGE=1 -DWITH_DOXYGEN=1
make
make doc

rm -rf ../test_coverage
lcov --base-directory . --directory . --zerocounters -q
make test
lcov --base-directory . --directory . -c -o project_test.info
lcov --remove project_test.info "/usr*" -o project_test.info
genhtml -o test_coverage -t "MAS test coverage" --num-spaces 4 project_test.info

cd ..
mv build/test_coverage ./$COVERAGE_FOLDER_NAME
rm -rf build doxygen/html doxygen.conf
