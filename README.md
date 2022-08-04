# LeafTipTriangulation
Triangulation of points from multiple views without correspondences

## Purpose
Triangulate leaf tips of Sorghum plants captured in a phenotyping facility.

More generally, triangulation of 3D points:
- from mulitple views
- with occlusions
- without correspondences
- without assuming a rigid motion of points between views

## Related work
> Cheng, Y. Q., Collins, R., Hanson, A., & Riseman, E. (1994, November). Triangulation without correspondences. In *DARPA Image Understanding Workshop (IUW)*.

## Run the software
```bash
# Build the software on a Linux platform
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ cmake --build . --config Release --parallel 16
# Run tests
$ ctest
# Run the leaf counting
$ cd ../Phenotyping
$ ../build/bin/LeafTipTriangulation leaf_counting
```

## Run the software on the Purdue RCAC cluster
```bash
# Run from the root of the repository
# Compile
$ bash jobs/compile.sh
# Run the phenotyping example
$ sbatch -A standby jobs/run_phenotyping.sub
```

## Run the software using Python
This snippet triangulates the leaf tips visible from two views.

```python
import sys
from pathlib import Path
import numpy as np

# Add the module to the path to import it
file_path = Path(__file__).resolve()
project_path = file_path.parent
# Path to the workspace folder
module_path = project_path / 'build' / 'LeafTipTriangulationPython' / 'Release'
sys.path.append(str(module_path))
from LeafTipTriangulationPython import loadCamerasFromFiles, \
                                       computeRays, \
                                       matchRaysAndTriangulate

camera_directory = project_path / 'Phenotyping' / 'sorghum_2018' / 'cameras'
cameras = loadCamerasFromFiles([
        str(camera_directory / 'camera_0_72_0.txt'),
        str(camera_directory / 'camera_0_216_0.txt')
    ])

points2d = [
    # Camera 72
    [
        np.array([803, 884]),   np.array([877, 576]),
        np.array([931, 1260]),  np.array([1195, 1258]),
        np.array([1285, 1094]), np.array([1481, 1356]),
        np.array([1609, 1116]), np.array([1627, 686])
    ],
    # Camera 216
    [
        np.array([815, 696]),   np.array([841, 1112]),
        np.array([1189, 1096]), np.array([1007, 1364]),
        np.array([1449, 1280]), np.array([1627, 1266]),
        np.array([1731, 880]),  np.array([1575, 584])
    ]
]

rays = computeRays(cameras, points2d)
# Theta is high to enforce matchings
theta = 4000.0
points3d = matchRaysAndTriangulate(cameras, points2d, rays, theta)

print("Triangulated {:d} points:".format(len(points3d)))

for i in range(len(points3d)):
    print(points3d[i])
```

See example Python files in folder `LeafTipTriangulationPython` for more python examples.

## Author
Mathieu Gaillard  
[CGVLAB](https://www.cs.purdue.edu/cgvlab/www/)  
Department of Computer Science  
Purdue University  

## License
See the LICENSE file.
