#!/usr/bin/env python3
# coding: utf-8

"""
Script to compute statistics of results with respect to ground-truth.
"""
import logging
import sys
import argparse
import logging
import re
from _operator import truth
from pathlib import Path
import csv


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
        x.append(result['result'])
        y.append(result['truth'])

    return x, y


def compare_to_ground_truth(input_path: Path, truth_path: Path):
    result_lines = read_csv(input_path)
    truth_lines = read_csv(truth_path)
    results_with_truth = merge_results_with_ground_truth(result_lines, truth_lines)

    for result in results_with_truth:
        print('{}\t{}\t{}'.format(result['name'], result['result'], result['truth']))

    results, truths = vectors_results_truth(results_with_truth)

    print(results)
    print(truths)

    # TODO: Histogram of the absolute difference in leaf counted
    # TODO: Linear regression between result and ground-truth
    #       generate the scatter plot, X axis is the human observation, Y axis is the prediction (Figure 5 paper Chenyong)


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
