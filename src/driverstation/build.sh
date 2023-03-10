#!/usr/bin/env bash

PROJECT_NAME=driverstation

mkdir -p build
cd build

if [ -f "src/$PROJECT_NAME" ]; then
	rm "src/$PROJECT_NAME"
fi

cmake -B . -S ..
make
