import sys
from pathlib import Path
import numpy as np
import cv2

# Add the module to the path to import it
file_path = Path(__file__).resolve()
project_path = file_path.parent.parent
# Path to the workspace folder
module_path = project_path / 'build' / 'LeafTipTriangulationPython'
sys.path.append(str(module_path))
from LeafTipTriangulationPython import undistortAndFlipYAxis, \
                                       generateCamerasFromOpenCV, \
                                       computeRays, \
                                       matchRaysAndTriangulate

crocodile_path = project_path / 'Images' / 'crocodile'
image_files = ['1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg']

print("Calibration pattern detection:")

# Configuration for the Charuco board
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
board = cv2.aruco.CharucoBoard_create(11, 8, 0.02, 0.015, aruco_dict)
# Sub-pixel corner detection criterion
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

allCorners = []
allIds = []

# Read images and detect markers
for image_file in image_files:
    image = cv2.imread(str(crocodile_path / image_file))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict)

    if len(corners) > 0:
        # Sub-pixel refinement
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)
        
        retval, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        if charucoCorners is not None and charucoIds is not None and len(charucoCorners) > 3:
            allCorners.append(charucoCorners)
            allIds.append(charucoIds)

imsize = gray.shape
imWidth = imsize[1]
imHeight = imsize[0]

print("Calibration:")

cameraMatrixInit = np.array([[ 1000.0,    0.0, imWidth/2.0],
                             [    0.0, 1000.0, imHeight/2.0],
                             [    0.0,    0.0,           1.0]])
distCoeffsInit = np.zeros((5,1))
flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
(ret, camera_matrix, distortion_coefficients0,
rotation_vectors, translation_vectors,
stdDeviationsIntrinsics, stdDeviationsExtrinsics,
perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                charucoCorners=allCorners,
                charucoIds=allIds,
                board=board,
                imageSize=imsize,
                cameraMatrix=cameraMatrixInit,
                distCoeffs=distCoeffsInit,
                flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

# Points on cameras
points2d = [
    # Camera 0
    [np.array([1877, 1237]),  np.array([2185, 1056])],
    # Camera 1
    [np.array([1687, 1164]),   np.array([2108, 1147])],
    # Camera 2
    [np.array([1352, 1078]),  np.array([1701, 1154])],
    # Camera 3
    [np.array([1714, 821]),   np.array([1944, 1067])],
    # Camera 4
    [np.array([2198, 1334]),   np.array([2300, 1102])]
]

# Undistort and Flip-Y axis points
undistortedPoints2d = undistortAndFlipYAxis(camera_matrix, distortion_coefficients0, imHeight, points2d)

# Generate cameras from the calibrated views
cameras = generateCamerasFromOpenCV(imWidth, imHeight, 5.76, 4.29, camera_matrix, rotation_vectors, translation_vectors)

# Triangulate the two points
rays = computeRays(cameras, undistortedPoints2d)
# Theta is high to enforce matchings
theta = 4000.0
points3d = matchRaysAndTriangulate(cameras, points2d, rays, theta)

# Output the L2 distance between the two triangulated points
print("Triangulated {:d} points:".format(len(points3d)))

dist = np.linalg.norm(points3d[0] - points3d[1])
print("Distance between the two first points: {:.4f} m".format(dist))
