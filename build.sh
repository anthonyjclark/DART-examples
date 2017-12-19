#!/usr/bin/env bash

DYLIB_DIR="$HOME/.local/lib"

COMPILER="clang++"
CPP_FLAGS="-std=c++14 -Wall -Wextra"
BIN_DIR="bin"

# Using isystem to suppress DART warnings
INC_DIRS="-I/usr/local/include/eigen3 -isystem$HOME/.local/include -I/usr/local/include/bullet"
LIB_DIRS="-L$DYLIB_DIR"
LIBS="-ldart -lassimp -lBulletCollision -lLinearMath -ldart-collision-bullet"

export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$DYLIB_DIR

function erun () {
    echo $@
    $@
}

function compile () {
    erun $COMPILER $CPP_FLAGS $INC_DIRS $LIB_DIRS $LIBS $1/$1.cpp -o $BIN_DIR/$1
}

function build () {
    if [ -d "$1" ]; then
        compile "$1"
    else
        printf "Error: could not find \"""$1""\"\n"
    fi
}

function run () {
    if [ -f "$BIN_DIR"/"$1" ]; then
        "$BIN_DIR"/"$1"
    else
        printf "Error: could not find \"""$BIN_DIR"/"$1""\"\n"
    fi
}

# Create a bin directory
mkdir -p $BIN_DIR


case "$1" in
    clean )
        rm -rf "$BIN_DIR"
        ;;

    build )
        build ${2%/}
        ;;

    run )
        run ${2%/}
        ;;
    brun )
        printf "*** Building ***\n"
        build ${2%/}

        if [ $? -eq 0 ]; then
            printf "*** Running ***\n"
            run ${2%/}
        fi
        ;;

    * )
        printf "Error: unknown command \"""$1""\"\n"
        ;;
esac


