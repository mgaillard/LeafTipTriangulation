#!/bin/bash

./x64/Release/LeafTipTriangulation.exe runtime_points > plots/runtime_points.dat
gnuplot -e "filename='plots/runtime_points.dat'" "plots/runtime_points.pg" > plots/runtime_points.png

./x64/Release/LeafTipTriangulation.exe runtime_cameras > plots/runtime_cameras.dat
gnuplot -e "filename='plots/runtime_cameras.dat'" "plots/runtime_cameras.pg" > plots/runtime_cameras.png

./x64/Release/LeafTipTriangulation.exe accuracy_cameras > plots/accuracy_cameras.dat
gnuplot -e "filename='plots/accuracy_cameras.dat'" "plots/accuracy_cameras.pg" > plots/accuracy_cameras.png

./x64/Release/LeafTipTriangulation.exe correspondence_noise > plots/correspondence_noise.dat
gnuplot -e "filename='plots/correspondence_noise.dat'" "plots/correspondence_noise.pg" > plots/correspondence_noise.png

./x64/Release/LeafTipTriangulation.exe correspondence_threshold > plots/correspondence_threshold.dat
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_1.pg" > plots/correspondence_threshold_1.png
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_2.pg" > plots/correspondence_threshold_2.png