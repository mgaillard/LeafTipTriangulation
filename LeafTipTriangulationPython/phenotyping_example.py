import sys
from pathlib import Path
import numpy as np

# Add the module to the path to import it
file_path = Path(__file__).resolve()
project_path = file_path.parent.parent
# Path to the workspace folder
module_path = project_path / 'build' / 'LeafTipTriangulationPython' / 'Release'
sys.path.append(str(module_path))
from LeafTipTriangulationPython import loadCamerasFromFiles, \
                                       computeRays, \
                                       matchRaysAndTriangulate

camera_directory = project_path / 'Phenotyping' / 'sorghum_2018' / 'cameras'
cameras = loadCamerasFromFiles([
        str(camera_directory / 'camera_0_0_0.txt'),
        str(camera_directory / 'camera_0_72_0.txt'),
        str(camera_directory / 'camera_0_144_0.txt'),
        str(camera_directory / 'camera_0_216_0.txt'),
        str(camera_directory / 'camera_0_288_0.txt'),
        str(camera_directory / 'camera_top_0_90_0.txt')
    ])

points2d = [
    # Camera 0
    [
        np.array([1009, 560]),  np.array([813, 866]),
        np.array([883, 1268]),  np.array([909, 1258]),
        np.array([1319, 1308]), np.array([1445, 1056]),
        np.array([1473, 71])
    ],
    # Camera 72
    [
        np.array([803, 884]),   np.array([877, 576]),
        np.array([931, 1260]),  np.array([1195, 1258]),
        np.array([1285, 1094]), np.array([1481, 1356]),
        np.array([1609, 1116]), np.array([1627, 686])
    ],
    # Camera 144
    [
        np.array([1348, 888]),  np.array([1228, 1112]),
        np.array([1294, 1366]), np.array([1364, 1252]),
        np.array([1514, 1258])
    ],
    # Camera 216
    [
        np.array([815, 696]),   np.array([841, 1112]),
        np.array([1189, 1096]), np.array([1007, 1364]),
        np.array([1449, 1280]), np.array([1627, 1266]),
        np.array([1731, 880]),  np.array([1575, 584])
    ],
    # Camera 288
    [
        np.array([999, 711]),   np.array([993, 1099]),
        np.array([1027, 1349]), np.array([1051, 1281]),
        np.array([1327, 1277]), np.array([1435, 865]),
        np.array([1463, 549])
    ],
    # Camera top
    [
        np.array([1600, 1492]), np.array([1572, 1525]),
        np.array([1373, 1491]), np.array([1220, 1113]),
        np.array([698, 1171]),  np.array([670, 683]),
        np.array([633, 596]),   np.array([944, 628])
    ]
]

rays = computeRays(cameras, points2d)
# Theta is high to enforce matchings
theta = 4000.0
points3d = matchRaysAndTriangulate(cameras, points2d, rays, theta)

print("Triangulated {:d} points:".format(len(points3d)))

for i in range(len(points3d)):
    print(points3d[i])
