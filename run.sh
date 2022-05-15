#!/bin/bash

./build/bin/Release/LeafTipTriangulation.exe runtime_points > plots/runtime_points.dat
gnuplot -e "filename='plots/runtime_points.dat'" "plots/runtime_points.pg" > plots/runtime_points.pdf

./build/bin/Release/LeafTipTriangulation.exe runtime_cameras > plots/runtime_cameras.dat
gnuplot -e "filename='plots/runtime_cameras.dat'" "plots/runtime_cameras.pg" > plots/runtime_cameras.pdf

./build/bin/Release/LeafTipTriangulation.exe accuracy_cameras > plots/accuracy_cameras.dat
gnuplot -e "filename='plots/accuracy_cameras.dat'" "plots/accuracy_cameras.pg" > plots/accuracy_cameras.pdf

./build/bin/Release/LeafTipTriangulation.exe correspondence_noise > plots/correspondence_noise.dat
gnuplot -e "filename='plots/correspondence_noise.dat'" "plots/correspondence_noise.pg" > plots/correspondence_noise.pdf

./build/bin/Release/LeafTipTriangulation.exe correspondence_threshold > plots/correspondence_threshold.dat
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_1.pg" > plots/correspondence_threshold_1.pdf
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_2.pg" > plots/correspondence_threshold_2.pdf

./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2018

./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2022
