#!/bin/bash
#-----------------------------------------------------------
# Script to compile the project on Purdue RCAC computer
# Launch from the root of the repository from the login node
#-----------------------------------------------------------

module load cmake/3.20.6
module load gcc/9.3.0
module load gnuplot

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

# Install Make 4.3
wget http://ftp.gnu.org/gnu/make/make-4.3.tar.gz
tar xvf make-4.3.tar.gz
rm make-4.3.tar.gz
cd make-4.3
./configure --prefix=/usr --without-guile --host=$LFS_TGT --build=$(build-aux/config.guess)
make -j16
cd ..
./make-4.3/make --version
