#!/usr/bin/env python3
# coding: utf-8

"""
Script to compute statistics of results with respect to ground-truth.
"""
import logging
import sys
import argparse
import logging
from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


def read_csv(csv_path: Path):
    """
    Read results or ground-truth from a CSV file
    Example:
    [
        {'name': '3-10-22-Schnable-Sorghum_310-201', 'result': 4},
    ]
    """
    result_lines = []

    with open(str(csv_path)) as csvfile:
        reader = csv.reader(csvfile, delimiter='\t')
        for row in reader:
            if len(row) == 2:
                result_lines.append({
                    'name': row[0],
                    'result': row[1]
                })

    return result_lines


def merge_results_with_ground_truth(result_lines, truth_lines):
    """
    Merge the lines of results with the lines from ground-truth
    Example:
    [
        {'name': '3-10-22-Schnable-Sorghum_310-201', 'result': 4, 'truth': 5},
    ]
    """
    merged_result_lines = []

    for result in result_lines:
        for truth in truth_lines:
            if result['name'] == truth['name']:
                merged_result_lines.append({
                    'name': result['name'],
                    'result': result['result'],
                    'truth': truth['result']
                })

    return merged_result_lines


def vectors_results_truth(merged_result_lines):
    """
    Return a vector X of results, and a vector Y of true values
    """
    x = []
    y = []

    for result in merged_result_lines:
        x.append(int(result['result']))
        y.append(int(result['truth']))

    return x, y


def generate_scatter_plot(results, truths):
    """
    Generate a scatter plot with a fitting line
    results = [4, 6, 5]
    truths = [3, 6, 4]
    """
    # Convert to Numpy
    original_x = np.array(truths)
    original_y = np.array(results)
    # Compute the number of occurrence of the pairs (observation, prediction)
    # Each object in the array histogram is like {'x': 2, 'y': 3, 'n': 1}
    histogram = []
    for i in range(len(original_x)):
        found = False
        for j in range(len(histogram)):
            if histogram[j]['x'] == original_x[i] and histogram[j]['y'] == original_y[i]:
                histogram[j]['n'] = histogram[j]['n'] + 1
                found = True
                break
        if not found:
            histogram.append({
                'x': original_x[i],
                'y': original_y[i],
                'n': 1
            })

    # Size multiplier for size of markers
    size_multiplier = 32.0

    # Convert to Numpy format
    x = np.zeros(len(histogram))
    y = np.zeros(len(histogram))
    n = np.zeros(len(histogram))
    for i in range(len(histogram)):
        x[i] = histogram[i]['x']
        y[i] = histogram[i]['y']
        n[i] = size_multiplier * histogram[i]['n']

    # Bounds for the axis of the plot
    lower_bound = min(np.min(x), np.min(y))
    higher_bound = max(np.max(x), np.max(y))

    # Linear regression
    res = stats.linregress(original_x, original_y)
    line_a = res.slope
    line_b = res.intercept
    r_sq = res.rvalue ** 2
    line_x = np.linspace(lower_bound, higher_bound, 10)
    line_y = line_a * line_x + line_b

    # RMSE computation
    rmse = np.sqrt(np.mean((original_y - original_x) ** 2))

    # Agreement in percentage
    agreement = int(100.0 * np.count_nonzero(original_x == original_y) / len(original_x))

    # Plot
    fig, ax = plt.subplots(figsize=(4, 4), dpi=216)

    # Text box in upper left in axes coords, with info about regression
    ax.text(0.05, 0.95,
            'R^2: {:.3f}\nRMSE: {:.3f}\nAgreement: {:d} %'.format(r_sq, rmse, agreement),
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment='top')

    # Plot the regression line
    ax.plot(line_x, line_y, '-r', label='y={:.2f}*x+{:.2f}'.format(line_a, line_b))
    ax.legend(loc='lower right')

    # Scatter plot
    ax.scatter(x, y, s=n)

    ax.set(xlim=(lower_bound - 0.5, higher_bound + 0.5),
           xticks=np.arange(lower_bound, higher_bound + 1),
           xlabel='Human observation',
           ylim=(lower_bound - 0.5, higher_bound + 0.5),
           yticks=np.arange(lower_bound, higher_bound + 1),
           ylabel='Prediction')

    plt.show()


def generate_histogram(results, truths):
    """
    Generate a histogram of differences between observations and predictions
    results = [4, 6, 5]
    truths = [3, 6, 4]
    """
    # Convert to Numpy
    x = np.array(truths)
    y = np.array(results)
    # Difference between human observations and predictions
    diff = x - y
    # Maximum absolute value of the difference, to infer the number of bins
    n_bins = np.max(np.abs(x - y))

    # Plot
    fig, ax = plt.subplots(figsize=(4, 4), dpi=216)

    ax.hist(diff, bins=np.arange(-n_bins, n_bins + 1), rwidth=0.95)

    ax.set_xlabel('Human observation - Predictions')
    ax.set_ylabel('Count')

    plt.show()


def compare_to_ground_truth(input_path: Path, truth_path: Path):
    result_lines = read_csv(input_path)
    truth_lines = read_csv(truth_path)
    results_with_truth = merge_results_with_ground_truth(result_lines, truth_lines)

    for result in results_with_truth:
        print('{}\t{}\t{}'.format(result['name'], result['result'], result['truth']))

    results, truths = vectors_results_truth(results_with_truth)

    print(results)
    print(truths)

    # Linear regression between result and ground-truth
    generate_scatter_plot(results, truths)
    # Histogram of the absolute difference in leaf counted
    generate_histogram(results, truths)


def add_logging_arguments(parser):
    # Argument for logging level
    parser.add_argument('--log', type=str, help='Logging level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'])

    # Argument for logging to file rather than stdout
    parser.add_argument('--log_file', type=str, help='Logging file')


def init_logging(args_log_level: str, args_log_file: str):
    """
    Initialize logging according to the command line argument
    """
    log_format = '[%(asctime)s] [%(levelname)s] %(message)s'
    # By default, the logging level is INFO
    numeric_level = logging.INFO

    # If the log level argument is defined, we retrieve the numeric level from the argument
    if args_log_level:
        numeric_level = getattr(logging, args_log_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % args_log_level)

    if args_log_file:
        # Logging in file
        logging.basicConfig(filename=args_log_file, format=log_format, level=numeric_level)
    else:
        # Logging in stdout
        logging.basicConfig(format=log_format, level=numeric_level)


def main() -> int:
    parser = argparse.ArgumentParser()

    # Arguments for logging
    add_logging_arguments(parser)
    # Argument for the input CSV file
    parser.add_argument('--input', type=str, help='Path to the result CSV file (required)', default='')
    # Argument for the ground-truth CSV file
    parser.add_argument('--truth', type=str, help='Path to the ground-truth CSV file (required)', default='')

    args = parser.parse_args()

    init_logging(args.log, args.log_file)

    # Check arguments exist
    if not args.input or not args.truth:
        parser.print_help(sys.stderr)
        return 1

    input_path = Path(args.input)
    truth_path = Path(args.truth)

    # Check arguments are valid
    if not input_path.exists() or not truth_path.exists():
        logging.error('The input file {} or the ground-truth file {} does not exist'.format(input_path, truth_path))
        return 1

    # Execute the command
    compare_to_ground_truth(input_path, truth_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
