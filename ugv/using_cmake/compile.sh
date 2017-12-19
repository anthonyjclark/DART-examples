#!/usr/bin/env bash

/Library/Developer/CommandLineTools/usr/bin/c++     \
    -isystem/Users/ajc/.local/include               \
    -isystem /usr/local/include/eigen3              \
    -isystem /usr/local/include/bullet              \
    -std=c++14                                      \
    -o ugv.cpp.o                                    \
    -c ugv.cpp


/Library/Developer/CommandLineTools/usr/bin/c++                     \
    -Wl,-search_paths_first                                         \
    -Wl,-headerpad_max_install_names                                \
    ugv.cpp.o                                                       \
    -o ugv                                                          \
    -Wl,-rpath,/Users/ajc/.local/lib                                \
    /Users/ajc/.local/lib/libdart-collision-bullet.6.3.0.dylib      \
    /Users/ajc/.local/lib/libdart.6.3.0.dylib                       \
    /usr/local/Cellar/libccd/2.0_2/lib/libccd.dylib                 \
    /usr/local/Cellar/fcl/0.5.0/lib/libfcl.dylib                    \
    /usr/local/Cellar/assimp/4.0.1/lib/libassimp.dylib              \
    /usr/local/lib/libboost_regex-mt.dylib                          \
    /usr/local/lib/libboost_system-mt.dylib                         \
    /Users/ajc/.local/lib/libdart-external-odelcpsolver.6.3.0.dylib \
    /usr/local/lib/libBulletDynamics.dylib                          \
    /usr/local/lib/libBulletCollision.dylib                         \
    /usr/local/lib/libLinearMath.dylib                              \
    /usr/local/lib/libBulletSoftBody.dylib
