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

## Author
Mathieu Gaillard
[CGVLAB](https://www.cs.purdue.edu/cgvlab/www/)
Department of Computer Science
Purdue University

## License
See the LICENSE file.
