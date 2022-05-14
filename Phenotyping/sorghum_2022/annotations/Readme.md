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
To compare the results to the ground-truth, launch the triangulation program, and save the result in a CSV file.
```PowerShell
cd LeafTipTriangulation/Phenotyping
../build/bin/Release/LeafTipTriangulation.exe leaf_counting > results.csv
```
Use the script `compare_to_ground_truth.py` to compute the measures:
```bash
$ python compare_to_ground_truth.py --input results.csv --truth ground_truth.csv
```
Make sure the encoding is right if the script cannot read one of the CSV files.

## Special plants
In this section, descriptions of plants with problems that may affect the leaf triangulation:
- `3-10-22-Schnable-Sorghum_310-204-16-1-P850029_2022-03-11_14-42-08.383_5207800` touches the sides of the imaging chamber
- `3-10-22-Schnable-Sorghum_310-205-16-2-P850029_2022-03-11_14-44-09.928_5207900` touches the sides of the imaging chamber
- `3-10-22-Schnable-Sorghum_310-206-17-6-AS_4601_Pawaga_2022-03-11_14-46-07.851_5208000` many leaf tips are close to each other, and occluded by the pot
- `3-10-22-Schnable-Sorghum_310-211-22-2-R_LINE_AR2002_2022-03-11_14-59-49.733_5208500` has many meaf touching the ground with self occlusion, leaf tips move a lot between shots and some are not visible
- `3-10-22-Schnable-Sorghum_310-212-23-1-SC_1439_2022-03-11_15-01-44.995_5208600` has leaves touching the floor of the machine
- `3-10-22-Schnable-Sorghum_310-213-23-2-SC_1439_2022-03-11_15-04-00.664_5208700` has two panicles 
