#!/bin/bash

mkdir -p results

# Processing the sorghum 2018 dataset
mkdir -p results/sorghum_2018
./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2018 0.1 results/sorghum_2018/results.csv results/sorghum_2018/nb_annotations.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --input results/sorghum_2018/results.csv --truth Phenotyping/sorghum_2018/annotations/ground_truth.csv --output results/sorghum_2018

# Processing the sorghum 2022 dataset
mkdir -p results/sorghum_2022
python3 Phenotyping/scripts/convert_cvat_to_csv.py --input Phenotyping/sorghum_2022/annotations/annotations.xml > Phenotyping/sorghum_2022/leaf_tips.csv
./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2022 0.1 results/sorghum_2022/results.csv results/sorghum_2022/nb_annotations.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --input results/sorghum_2022/results.csv --truth Phenotyping/sorghum_2022/annotations/ground_truth.csv --output results/sorghum_2022

# Merge the results of the two datasets and compare to observations
cat results/sorghum_2018/results.csv results/sorghum_2022/results.csv > results/results.csv
cat Phenotyping/sorghum_2018/annotations/ground_truth.csv Phenotyping/sorghum_2022/annotations/ground_truth.csv > results/ground_truth.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --input results/results.csv --truth results/ground_truth.csv --output results

./build/bin/Release/LeafTipTriangulation.exe runtime_points > plots/runtime_points.dat
gnuplot -e "filename='plots/runtime_points.dat'" "plots/runtime_points.pg" > results/runtime_points.pdf

./build/bin/Release/LeafTipTriangulation.exe runtime_cameras > plots/runtime_cameras.dat
gnuplot -e "filename='plots/runtime_cameras.dat'" "plots/runtime_cameras.pg" > results/runtime_cameras.pdf

./build/bin/Release/LeafTipTriangulation.exe accuracy_cameras > plots/accuracy_cameras.dat
gnuplot -e "filename='plots/accuracy_cameras.dat'" "plots/accuracy_cameras.pg" > results/accuracy_cameras.pdf

./build/bin/Release/LeafTipTriangulation.exe correspondence_noise > plots/correspondence_noise.dat
gnuplot -e "filename='plots/correspondence_noise.dat'" "plots/correspondence_noise.pg" > results/correspondence_noise.pdf

./build/bin/Release/LeafTipTriangulation.exe correspondence_threshold > plots/correspondence_threshold.dat
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_1.pg" > results/correspondence_threshold_1.pdf
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_2.pg" > results/correspondence_threshold_2.pdf
