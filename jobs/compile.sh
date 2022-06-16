#!/bin/bash
#-----------------------------------------------------------
# Script to compile the project on Purdue RCAC computer
# Launch from the root of the repository from the login node
#-----------------------------------------------------------

module load cmake/3.20.6
module load gcc/10.2.0

# Launch the build
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --parallel 32

# Add a symlink to have the same path as in Windows
mkdir -p bin/Release
ln -s ../LeafTipTriangulation bin/Release/LeafTipTriangulation.exe

# Install datamash
wget https://ftp.gnu.org/gnu/datamash/datamash-1.7.tar.gz
tar -xzf datamash-1.7.tar.gz
rm datamash-1.7.tar.gz
cd datamash-1.7
./configure
make
make check
cd ..
