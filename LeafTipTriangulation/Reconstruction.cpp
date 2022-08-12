#include "Reconstruction.h"

#include <utils/warnoff.h>
#include <glm/gtx/string_cast.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <utils/warnon.h>

#include "RayMatching.h"
#include "Triangulation.h"

double measureTwoPointsCharuco(
    int imageWidth,
    int imageHeight,
    const cv::Mat1d& cameraMatrix,
    const cv::Mat1d& distCoeffs,
    const std::vector<cv::Mat1d>& rvecs,
    const std::vector<cv::Mat1d>& tvecs,
    const SetsOfVec2& inputPoints2d)
{
    assert(inputPoints2d.size() == tvecs.size());
    assert(inputPoints2d.size() == rvecs.size());
    
    // Undistort 2D points and change the frame of reference
    const auto points2d = undistortCenterAndFlipYAxis(imageWidth, imageHeight, cameraMatrix, distCoeffs, inputPoints2d);

    // For a Google Pixel 3, the sensor is 5.76 mm by 4.29 mm
    const cv::Size2d sensorSize(5.76, 4.29);

    // Generate the cameras
    const auto cameras = generateCamerasFromOpenCV(imageWidth,
                                                   imageHeight,
                                                   sensorSize,
                                                   cameraMatrix,
                                                   rvecs,
                                                   tvecs);

    // Compute rays in 3D from camera matrices and 2D points
    const auto rays = computeRays(cameras, points2d);

    // Matching and triangulation of points
    const auto [triangulatedPoints3D, setsOfCorrespondences, viewOrder] = matchRaysAndTriangulate(cameras, points2d, rays);

    return glm::distance(triangulatedPoints3D[0], triangulatedPoints3D[1]);
}
