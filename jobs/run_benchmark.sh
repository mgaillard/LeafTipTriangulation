#!/bin/sh -l
#FILENAME: run_benchmark.sub
#SBATCH --nodes=1
#SBATCH --ntasks=128
#SBATCH --time=0:30:00
#SBATCH --job-name LeafTipTriangulation

module load gcc/9.3.0
module load gnuplot

./build/bin/Release/LeafTipTriangulation.exe correspondence_threshold > plots/correspondence_threshold.dat
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_1.pg" > results/correspondence_threshold_1.pdf
gnuplot -e "filename='plots/correspondence_threshold.dat'" "plots/correspondence_threshold_2.pg" > results/correspondence_threshold_2.pdf
