#!/bin/bash

LeafTipTriangulationCmd="./build/bin/Release/LeafTipTriangulation.exe"
phenotype="tips"
theta="0"

mkdir -p results

# Processing the sorghum 2018 dataset
mkdir -p results/sorghum_2018
$LeafTipTriangulationCmd leaf_counting Phenotyping/sorghum_2018 $phenotype $theta 0 0.0 results/sorghum_2018/results.csv results/sorghum_2018/nb_annotations.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/sorghum_2018/results.csv --truth Phenotyping/sorghum_2018/annotations/ground_truth.csv --output results/sorghum_2018

# Processing the sorghum 2022 dataset
mkdir -p results/sorghum_2022
python3 Phenotyping/scripts/convert_cvat_to_csv.py --input Phenotyping/sorghum_2022/annotations/annotations.xml > Phenotyping/sorghum_2022/leaf_tips.csv
$LeafTipTriangulationCmd leaf_counting Phenotyping/sorghum_2022 $phenotype $theta 0 0.0 results/sorghum_2022/results.csv results/sorghum_2022/nb_annotations.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/sorghum_2022/results.csv --truth Phenotyping/sorghum_2022/annotations/ground_truth.csv --output results/sorghum_2022

# Merge the results of the two datasets and compare to observations
cat results/sorghum_2018/results.csv results/sorghum_2022/results.csv > results/results.csv
cat Phenotyping/sorghum_2018/annotations/ground_truth.csv Phenotyping/sorghum_2022/annotations/ground_truth.csv > results/ground_truth.csv
python3 Phenotyping/scripts/compare_to_ground_truth.py --command graphs --input results/results.csv --truth results/ground_truth.csv --output results


# Function that computes agreement with varying probability of discarding a 2D points
compute_agreement_wrt_probability() {
    seeds=(14117 4173 6468 306 2456)
    probabilities=("0" "5" "10" "15" "20" "25" "30" "35" "40" "45" "50")

    # Arguments of the function
    local theta=$1
    local directory=$2
    local resultFile=$3

    resultTriangulationFile="$directory/results_triangulation_all.dat"
    resultBaseFile="$directory/results_baseline_all.dat"

    # Clear the file with all results
    > $resultTriangulationFile
    > $resultBaseFile

    for p in ${probabilities[@]};
    do
        probability=$(bc <<< "scale=2 ; $p / 100")
        resultProbabilityFile="$directory/results_${p}.dat"
        resultBaseProbabilityFile="$directory/results_baseline_${p}.dat"

        for s in ${seeds[@]};
        do
            resultCsvFile="$directory/results_${s}_${p}.csv"
            resultBaseCsvFile="$directory/results_baseline_${s}_${p}.csv"
            annotationCsvFile="$directory/nb_annotations_${s}_${p}.csv"
            $LeafTipTriangulationCmd leaf_counting Phenotyping/sorghum_2022 $phenotype $theta $s $probability $resultCsvFile $annotationCsvFile
            # Find the maximum number of annotated leaves in any view of each plant
            # This becomes the baseline for any single-view detection based method
            # NR>1 skips the first line of the file, which is the header
            awk 'NR>1 {m=$2;for(i=2;i<=NF;i++)if($i>m)m=$i;print $1"\t"m}' $annotationCsvFile > $resultBaseCsvFile
            # Compare the number of leaves from the result file
            # This is the result of our method
            python3 Phenotyping/scripts/compare_to_ground_truth.py --command values --input $resultCsvFile --truth Phenotyping/sorghum_2022/annotations/ground_truth.csv --output results/sorghum_2022 >> $resultProbabilityFile
            # This is the result of the baseline
            python3 Phenotyping/scripts/compare_to_ground_truth.py --command values --input $resultBaseCsvFile --truth Phenotyping/sorghum_2022/annotations/ground_truth.csv --output results/sorghum_2022 >> $resultBaseProbabilityFile
            rm $resultCsvFile
            rm $resultBaseCsvFile
            rm $annotationCsvFile
        done

        # First column of the result file is the probability of discarding a points
        printf '%s\t' "$p" >> $resultTriangulationFile
        # Datamash is a GNU command line tool to compute statistics on input data files
        # Compute the min, first quartile, median, last quartile, max, mean of the agreement
        datamash min 3 q1 3 median 3 q3 3 max 3 mean 3 sstdev 3 < $resultProbabilityFile >> $resultTriangulationFile

        # Compute the mean and standard deviation of the baseline
        datamash min 3 q1 3 median 3 q3 3 max 3 mean 3 sstdev 3 < $resultBaseProbabilityFile >> $resultBaseFile

        # Remove the result files for the probability $p
        rm $resultProbabilityFile
        rm $resultBaseProbabilityFile
    done

    # Merge the two result files into one result file
    paste $resultTriangulationFile $resultBaseFile > $resultFile
    rm $resultTriangulationFile
    rm $resultBaseFile
}

directory="results/sorghum_2022"

# Plot the agreement with varying probability of discarding points for different values of theta
resultThetaFile="$directory/results_theta.dat"
# Clear the result file
> $resultThetaFile
thetaValues=(0 3000 2000 1000)
for t in ${thetaValues[@]};
do
    resultCurrThetaFile="$directory/results_theta_$t.dat"
    compute_agreement_wrt_probability $t $directory $resultCurrThetaFile
    cat $resultCurrThetaFile >> $resultThetaFile
    # Delete the temporary file unless it is the one for which theta=0
    if [[ $t -ne 0 ]]; then
        rm $resultCurrThetaFile
    fi
done
# Plot the agreement against the baseline with a threshold of theta=0
resultThetaEqualZeroFile="$directory/results_theta_0.dat"
gnuplot -e "filename='$resultThetaEqualZeroFile'" "plots/phenotyping_agreement.pg" > $directory/agreement.pdf
# Plot the mean agreement with a varying threshold 
gnuplot -e "filename='$resultThetaFile'" "plots/phenotyping_agreement_theta.pg" > $directory/agreement_theta.pdf
