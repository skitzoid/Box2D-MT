#!/bin/bash
printf "\n Building Debug...\n\n"
if [ ! -d build ]; then
    mkdir build
fi
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
if [ "$1" = "-R" ]; then
	printf "\nFull rebuild, performing clean...\n\n"
	make clean
else
	printf "\nPass -R to force a full rebuild (clean all first)\n\n"
fi
make -j4

RESULT=$?

cd ..

if [ $RESULT -eq 0 ]; then
	printf "\n Success. Your files are in ./build/dist\n\n"
	exit 0
else
	printf "\n Errors encountered. \n\n"
	exit 1
fi
