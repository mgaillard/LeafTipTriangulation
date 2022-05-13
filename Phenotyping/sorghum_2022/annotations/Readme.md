Annotations for the Sorghum 2022 dataset
===

## Annotations of leaf tips and leaf junctions
Annotations were made on the CVAT online application.
The export format is `CVAT for Images 1.1`.
Annotations are exported in the XML file: `annotations.xml`.
To convert the annotations in XML format to CSV format, use the Python script `convert_cvat_to_csv.py`.
```bash
$ python convert_cvat_to_csv.py --input annotations.xml > annotations.csv
```
The file `annotations.csv` can later be processed for 3D triangulation.

## Ground-truth
The file `ground_truth.csv` contains the ground-truth number of leaves measured by humans on the real plants.
