#!/bin/bash

LeafTipTriangulationCmd="./build/bin/Release/LeafTipTriangulation.exe"
phenotype="tips"
# Best value of theta per dataset
theta2018="1250"
theta2022="2000"
theta2022_synthetic="1000"

copy_without_header_line() {
    # NR>1 skips the first line of the file, which is the header
    awk 'NR>1 {m=$2;for(i=2;i<=NF;i++)if($i>m)m=$i;print $1"\t"m}' $1 > $2
}

process_sorghum_dataset() {
    local directory=$1
    local theta=$2
    $LeafTipTriangulationCmd leaf_counting "Phenotyping/$directory" $phenotype $theta 0 0.0 "results/$directory/results.csv" "results/$directory/nb_annotations.csv"
    # Find the maximum number of annotated leaves in any view of each plant
    # This becomes the baseline for any single-view detection based method
    copy_without_header_line "results/$directory/nb_annotations.csv" "results/$directory/results_baseline.csv"
    # This is the result of the baseline
    python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input "results/$directory/results_baseline.csv" --truth "Phenotyping/$directory/annotations/ground_truth.csv" --output "results/$directory"
    # Rename the output files for the baseline
    mv "results/$directory/histogram.png" "results/$directory/histogram_baseline.png"
    mv "results/$directory/scatter.png" "results/$directory/scatter_baseline.png"
    python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input "results/$directory/results.csv" --truth "Phenotyping/$directory/annotations/ground_truth.csv" --output "results/$directory"
}

mkdir -p results

# Processing the sorghum 2018 dataset
mkdir -p results/sorghum_2018
process_sorghum_dataset sorghum_2018 $theta2018

# Processing the sorghum 2022 dataset
mkdir -p results/sorghum_2022
python3 Phenotyping/scripts/convert_cvat_to_csv.py --input Phenotyping/sorghum_2022/annotations/annotations.xml > Phenotyping/sorghum_2022/leaf_tips.csv
process_sorghum_dataset sorghum_2022 $theta2022

# Processing the sorghum 2022 (synthetic) dataset
mkdir -p results/sorghum_2022_synthetic
process_sorghum_dataset sorghum_2022_synthetic $theta2022_synthetic

# Merge the results of the two datasets and compare to observations
cat results/sorghum_2018/results.csv results/sorghum_2022/results.csv > results/results.csv
cat Phenotyping/sorghum_2018/annotations/ground_truth.csv Phenotyping/sorghum_2022/annotations/ground_truth.csv > results/ground_truth.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/results.csv --truth results/ground_truth.csv --output results
