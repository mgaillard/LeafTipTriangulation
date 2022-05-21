#!/bin/bash

mkdir -p results

# Processing the sorghum 2018 dataset
mkdir -p results/sorghum_2018
./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2018 0 0.0 results/sorghum_2018/results.csv results/sorghum_2018/nb_annotations.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/sorghum_2018/results.csv --truth Phenotyping/sorghum_2018/annotations/ground_truth.csv --output results/sorghum_2018

# Processing the sorghum 2022 dataset
mkdir -p results/sorghum_2022
python3 Phenotyping/scripts/convert_cvat_to_csv.py --input Phenotyping/sorghum_2022/annotations/annotations.xml > Phenotyping/sorghum_2022/leaf_tips.csv
./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2022 0 0.0 results/sorghum_2022/results.csv results/sorghum_2022/nb_annotations.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/sorghum_2022/results.csv --truth Phenotyping/sorghum_2022/annotations/ground_truth.csv --output results/sorghum_2022

# Merge the results of the two datasets and compare to observations
cat results/sorghum_2018/results.csv results/sorghum_2022/results.csv > results/results.csv
cat Phenotyping/sorghum_2018/annotations/ground_truth.csv Phenotyping/sorghum_2022/annotations/ground_truth.csv > results/ground_truth.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/results.csv --truth results/ground_truth.csv --output results

directory="results/sorghum_2022"
resultFile="$directory/results_all.dat"
seeds=(14117 4173 6468)
probabilities=("0" "5" "10" "15" "20" "25" "30")

# Clear the file with all results
> $resultFile

for p in ${probabilities[@]};
do
    probability=$(bc <<< "scale=2 ; $p / 100")
    resultProbabilityFile="$directory/results_${p}.dat"

    for s in ${seeds[@]};
    do    
        resultCsvFile="$directory/results_${s}_${p}.csv"
        annotationCsvFile="$directory/nb_annotations_${s}_${p}.csv"
        ./build/bin/Release/LeafTipTriangulation.exe leaf_counting Phenotyping/sorghum_2022 $s $probability $resultCsvFile $annotationCsvFile
        python3 Phenotyping/scripts/compare_to_ground_truth.py --command values --input $resultCsvFile --truth Phenotyping/sorghum_2022/annotations/ground_truth.csv --output results/sorghum_2022 >> $resultProbabilityFile
        rm $resultCsvFile
        rm $annotationCsvFile
    done

    # Use awk to compute the mean of each column of the result fole for probability $p
    # Source: https://unix.stackexchange.com/questions/307168/using-awk-to-calculate-mean-and-variance-of-columns
    awk -v P=$p '{ for(i=1;i<=NF;i++) total[i]+=$i ; } END { printf "%d\t",P ; for(i=1;i<=NF;i++) printf "%f ",total[i]/NR ; printf "\n" ; }' $resultProbabilityFile >> $resultFile

    # Remove the result file for the probability $p
    rm $resultProbabilityFile
done

# Plot with gnuplot
gnuplot -e "filename='$resultFile'" "plots/phenotyping_agreement.pg" > $directory/agreement.pdf
