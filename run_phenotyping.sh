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
