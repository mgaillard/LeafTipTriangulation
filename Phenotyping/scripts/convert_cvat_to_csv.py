#!/usr/bin/env python3
# coding: utf-8

"""
Script to convert annotations exported from CVAT in XML to CSV. 
"""
import logging
import sys
import argparse
import logging
import re
from pathlib import Path
from xml.dom.minidom import parse


def convert_image_name(name: str):
    """
    Convert the image name from format '0_0_0.jpg' to 'Vis_SV_0.png'
    If the name is of unknown format, return the input name
    """
    # Match side views
    view_matches = re.match('^(top_)?0_(\d+)_0(?:_\d+)?\.(?:jpg|png)$', name)
    if view_matches:
        # If the image name comes from a side view, this will be None
        # If it is a top view, this will be 'top_'
        top_or_side_view = view_matches.group(1)
        # This matches the angle of the view: 0, 72, ..., 90
        side_angle = int(view_matches.group(2))
        # Return the translated name of the view
        if top_or_side_view == 'top_':
            return 'Vis_TV_' + str(side_angle) + '.png'
        else:
            return 'Vis_SV_' + str(side_angle) + '.png'
    # If no match, return the original name of the view
    return name


def read_xml_tasks(document):
    """
    Read the list of all tasks in the XML document
    Return the list of all tasks as objects
    {'id': 258559, 'name': '3-10-22-Schnable-Sorghum_310-201', 'images': []}
    """
    tasks = []

    task_elements = document.getElementsByTagName('task')
    for task in task_elements:
        task_id = 0
        task_name = ''
        for child in task.childNodes:
            if child.nodeName == 'id':
                task_id = int(child.firstChild.nodeValue)
            elif child.nodeName == 'name':
                task_name = child.firstChild.nodeValue

        if task_id > 0 and task_name:
            tasks.append({
                'id': task_id,
                'name': task_name,
                'images': []
            })

    return tasks


def read_xml_images(document):
    """
    Read the list of all images in the XML document
    Return the list of images as objects
    {
        'id': 1,
        'task_id': 258559,
        'name': '0_252_0.jpg',
        'tip_points': [],
        'junction_points': []
    }
    """
    images = []

    image_elements = document.getElementsByTagName('image')
    for image in image_elements:
        # Check that attributes for the image are present
        if 'id' not in image.attributes or 'task_id' not in image.attributes or 'name' not in image.attributes:
            continue

        image_id = int(image.attributes['id'].value)
        image_task_id = int(image.attributes['task_id'].value)
        image_name = image.attributes['name'].value
        image_tip_points = []
        image_junction_points = []

        # Read leaf tips and junctions for the current image
        for child in image.childNodes:
            if child.nodeName == 'points':
                if 'label' in child.attributes and 'points' in child.attributes:
                    label = child.attributes['label'].value
                    point_str = child.attributes['points'].value
                    point_coords_str = point_str.split(',')
                    point_x = int(round(float(point_coords_str[0])))
                    point_y = int(round(float(point_coords_str[1])))
                    if label == 'Leaf tip':
                        image_tip_points.append({'x': point_x, 'y': point_y})
                    elif label == 'Leaf junction':
                        image_junction_points.append({'x': point_x, 'y': point_y})

        # If image has no point, discard it
        if len(image_tip_points) == 0 and len(image_junction_points) == 0:
            continue

        images.append({
            'id': image_id,
            'task_id': image_task_id,
            'name': image_name,
            'tip_points': image_tip_points,
            'junction_points': image_junction_points
        })

    return images


def join_tasks_and_images(tasks, images):
    """
    Add images related to tasks to the list of tasks
    """
    for task in tasks:
        for image in images:
            if image['task_id'] == task['id']:
                task['images'].append(image)


def print_task_in_csv_format(task):
    """
    Print images in a task in CSV format
    """
    for image in task['images']:
        # The name of the line in the CSV file
        line_name = task['name'] + '_' + convert_image_name(image['name'])
        # Write tips and junction points
        tip_strs = []
        for point in image['tip_points']:
            tip_strs.append('[{}, {}]'.format(point['x'], point['y']))
        junction_strs = []
        for point in image['junction_points']:
            junction_strs.append('[{}, {}]'.format(point['x'], point['y']))
        print('{}\t[{}]\t[{}]'.format(line_name, ', '.join(tip_strs), ', '.join(junction_strs)))


def convert_xml_annotations_to_csv(filepath: Path):
    """
    Open and parse annotations in XML format and print the equivalent CSV annotations in stdout
    """
    # Parse the XML file with minidom
    with open(str(filepath)) as file:
        document = parse(file)
    # Find tasks and images
    tasks = read_xml_tasks(document)
    images = read_xml_images(document)
    # Associate tasks and images
    join_tasks_and_images(tasks, images)
    # Write the annotations in CSV format
    for task in tasks:
        print_task_in_csv_format(task)


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
    # Argument for the input XML file
    parser.add_argument('--input', type=str, help='Path to the XML file (required)', default='')

    args = parser.parse_args()

    init_logging(args.log, args.log_file)

    # Check arguments exist
    if not args.input:
        parser.print_help(sys.stderr)
        return 1

    input_path = Path(args.input)

    # Check arguments are valid
    if not input_path.exists():
        logging.error('The input file {} does not exist'.format(input_path))
        return 1

    # Execute the command
    convert_xml_annotations_to_csv(input_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
