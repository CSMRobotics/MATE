#!/usr/bin/env bash

PROJECT_NAME=driverstation

if [ ! -f "build/src/$PROJECT_NAME" ]; then
	./build.sh
fi

"./build/src/$PROJECT_NAME"
