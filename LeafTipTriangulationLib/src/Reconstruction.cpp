#include "Reconstruction.h"

#include <iostream>
#define  _USE_MATH_DEFINES
#include <cmath>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <utils/warnon.h>

#include "ConvertUtils.h"
#include "RayMatching.h"
#include "Triangulation.h"


double radiansToDegrees(double radians)
{
	return radians * (180.0 / M_PI);
}

std::pair<double, double> focalLengthInMm(const cv::Mat1d& cameraMatrix, const cv::Size& imageSize, const cv::Size2d& sensorSize)
{
	return {
		cameraMatrix.at<double>(0, 0) * sensorSize.width / imageSize.width,
		cameraMatrix.at<double>(1, 1) * sensorSize.height / imageSize.height
	};
}

cv::Mat rotationX180(const cv::Mat1d& matrix)
{
	// 180 degrees rotation matrix
	cv::Mat1d R(3, 3, 0.0);

	R.at<double>(0, 0) = 1.0;
	R.at<double>(1, 1) = -1.0;
	R.at<double>(2, 2) = -1.0;

	const cv::Mat1d rotationResult = R * matrix;

	return rotationResult.clone();
}

cv::Mat1d removeZProjectionMatrix(const cv::Mat& projectionMatrix)
{
	cv::Mat1d H(3, 3, 0.0);

	H.at<double>(0, 0) = projectionMatrix.at<double>(0, 0);
	H.at<double>(1, 0) = projectionMatrix.at<double>(1, 0);
	H.at<double>(2, 0) = projectionMatrix.at<double>(2, 0);

	H.at<double>(0, 1) = projectionMatrix.at<double>(0, 1);
	H.at<double>(1, 1) = projectionMatrix.at<double>(1, 1);
	H.at<double>(2, 1) = projectionMatrix.at<double>(2, 1);

	H.at<double>(0, 2) = projectionMatrix.at<double>(0, 3);
	H.at<double>(1, 2) = projectionMatrix.at<double>(1, 3);
	H.at<double>(2, 2) = projectionMatrix.at<double>(2, 3);

	H /= H.at<double>(2, 2);

	return H;
}


cv::Mat1d cameraPose(const cv::Mat1d& rvec, const cv::Mat1d& tvec)
{
	cv::Mat1d R;
	cv::Rodrigues(rvec, R); // R is 3x3

	auto invTvec = -R.t() * tvec; // translation of inverse
	R = rotationX180(R.t());  // rotation of inverse

	cv::Mat1d T = cv::Mat1d::eye(4, 4); // T is 4x4
	T(cv::Range(0, 3), cv::Range(0, 3)) = R.t() * 1; // copies R into T
	T(cv::Range(0, 3), cv::Range(3, 4)) = invTvec * 1; // copies tvec into T

	// To get the Euler angles (XYZ)
	// Go to https://www.andre-gaschler.com/rotationconverter/
	// Copy the rotation matrix R
	// Input the angles (in degrees) in blender

	return T;
}

cv::Vec3d cameraPoseVectorX(const cv::Mat1d& rvec)
{
	cv::Mat1d R;
	cv::Rodrigues(rvec, R); // R is 3x3

	// X is the first column of the R transposed matrix
	cv::Mat1d T = R.t() * 1.0; // copies R into T

	return {
		T.at<double>(0, 0),
		T.at<double>(1, 0),
		T.at<double>(2, 0)
	};
}

// Source: https://answers.opencv.org/question/162932/create-a-stereo-projection-matrix-using-rvec-and-tvec/
cv::Mat1d computeProjectionMatrix(const cv::Mat1d& cameraMatrix, const cv::Mat1d& rvec, const cv::Mat1d& tvec)
{
	cv::Mat1d rotMat(3, 3), rotTransMat(3, 4);
	// Convert rotation vector into rotation matrix 
	cv::Rodrigues(rvec, rotMat);
	// Append translation vector to rotation matrix
	cv::hconcat(rotMat, tvec, rotTransMat);
	// Compute projection matrix by multiplying intrinsic parameter 
	// matrix (A) with 3 x 4 rotation and translation pose matrix (RT).
	// Formula: Projection Matrix = A * RT;
	return cameraMatrix * rotTransMat;
}

cv::Vec3d lookAtPoint(const cv::Mat1d& homography, const cv::Mat1d& cameraMatrix)
{
	assert(homography.rows == 3);
	assert(homography.cols == 4);

	// For the linear system Ax=b to solve
	cv::Mat1d A(3, 3, 0.0);
	A.at<double>(0, 0) = homography.at<double>(0, 0);
	A.at<double>(0, 1) = homography.at<double>(0, 1);
	A.at<double>(1, 0) = homography.at<double>(1, 0);
	A.at<double>(1, 1) = homography.at<double>(1, 1);
	A.at<double>(2, 0) = homography.at<double>(2, 0);
	A.at<double>(2, 1) = homography.at<double>(2, 1);

	A.at<double>(0, 2) = -cameraMatrix.at<double>(0, 2);
	A.at<double>(1, 2) = -cameraMatrix.at<double>(1, 2);
	A.at<double>(2, 2) = -1.0;

	cv::Mat1d b(3, 1, 0.0);
	b.at<double>(0) = -homography.at<double>(0, 3);
	b.at<double>(1) = -homography.at<double>(1, 3);
	b.at<double>(2) = -homography.at<double>(2, 3);

	// Solve for linear least squares
	cv::Mat1d x;
	cv::solve(A, b, x, cv::DECOMP_SVD);

	// Convert to vector
	return {
		x.at<double>(0),
		x.at<double>(1),
		0.0
	};
}

std::tuple<glm::vec3, glm::vec3, glm::vec3> cameraEyeAtUpFromPose(
	const cv::Mat1d& cameraMatrix,
	const cv::Mat1d& rvec,
	const cv::Mat1d& tvec
)
{
	const auto homography = computeProjectionMatrix(cameraMatrix, rvec, tvec);
	const auto pose = cameraPose(rvec, tvec);

	// Find position of the camera in world coordinates
	const glm::vec3 eye(
		pose.at<double>(0, 3),
		pose.at<double>(1, 3),
		pose.at<double>(2, 3)
	);
	
	// Find the look at point (intersection between optical center and plane z=0)
	const auto at = convertToGlm(lookAtPoint(homography, cameraMatrix));

	// Get the X vector of the camera in world coordinates
	const auto x = convertToGlm(cameraPoseVectorX(rvec));
	// From the X vector, we can get the up vector
	const auto eyeToAt = glm::normalize(at - eye);
	const auto up = glm::normalize(glm::cross(x, eyeToAt));

	return std::make_tuple(eye, at, up);
}

float measureTwoPointsCharuco(const std::vector<std::vector<glm::vec2>>& inputPoints2D)
{
	// Camera properties
	const int nbImages = 5;
	const int imageWidth = 4032;
	const int imageHeight = 3024;
	const cv::Size imageSize(imageWidth, imageHeight);

	// Camera calibration
	cv::Mat1d cameraMatrix = (cv::Mat1d(3, 3) <<
		3153.273013526734, 0.0, 2005.9916092061376, // fx, 0, cx
		0.0, 3105.260406995229, 1540.0981727685203, // 0, fy, cy
		0.0, 0.0, 1.0);

	cv::Mat1d distCoeffs = (cv::Mat1d(5, 1) <<
		0.280605688854058, // k1
		-1.6945809491702621, // k2
		0.0024575553828127643, // p1
		-0.0021983284657425975, // p2
		3.2574845170325277); // k3

	// Cameras poses
	const std::vector<cv::Mat1d> rvecs = {
		(cv::Mat1d(3, 1) << 2.40327, -0.457555, 0.335426),
		(cv::Mat1d(3, 1) << 2.40059, 0.350462, -0.116547),
		(cv::Mat1d(3, 1) << 2.2446, 0.816225, -0.373654),
		(cv::Mat1d(3, 1) << 2.16444, 1.58991, -0.220916),
		(cv::Mat1d(3, 1) << 2.21499, -1.05024, 0.374562),
	};

	const std::vector<cv::Mat1d> tvecs = {
		(cv::Mat1d(3, 1) << -0.0581547, 0.0542219, 0.163637),
		(cv::Mat1d(3, 1) << -0.119149, 0.0101526, 0.220561),
		(cv::Mat1d(3, 1) << -0.156187, -0.0288829, 0.281005),
		(cv::Mat1d(3, 1) << -0.112906, -0.111428, 0.307856),
		(cv::Mat1d(3, 1) << 0.0137574, 0.0700162, 0.207419),
	};

	// Undistort 2D points and change the frame of reference
	// Copy points
	auto points2D = inputPoints2D;
	for (auto& cameraPoints2D : points2D)
	{
		for (auto& point2D : cameraPoints2D)
		{
			// Distorted point
			std::vector<cv::Point2d> inputDistortedPoints = { cv::Point2d(point2D.x, point2D.y) };
			std::vector<cv::Point2d> outputUndistortedPoints;

			cv::undistortPoints(inputDistortedPoints, outputUndistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

			// Save undistorted point
			point2D.x = static_cast<float>(outputUndistortedPoints.front().x);
			point2D.y = static_cast<float>(imageHeight) - static_cast<float>(outputUndistortedPoints.front().y);
		}
	}

	// Since images are undistorted, we can now set the distortion coefficients to zero
	distCoeffs.setTo(0.0);

	// Compute aspect ratio from images
	const auto aspectRatio = float(imageWidth) / float(imageHeight);

	// For a Google Pixel 3, the sensor is 5.76 mm by 4.29 mm
	const cv::Size2d sensorSize(5.76, 4.29);
	const auto focalLength = focalLengthInMm(cameraMatrix, imageSize, sensorSize);
	const auto fovy = 2.0 * std::atan(sensorSize.height / (2.0 * focalLength.second)); 

	std::vector<Camera> cameras;
	for (int i = 0; i < nbImages; i++)
	{
		glm::vec3 eye, at, up;
		std::tie(eye, at, up) = cameraEyeAtUpFromPose(cameraMatrix, rvecs[i], tvecs[i]);

		cameras.emplace_back(eye, at, up,
			                 static_cast<float>(fovy),
			                 static_cast<float>(aspectRatio),
			                 glm::vec2(imageWidth, imageHeight));
	}

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(cameras, points2D);

	// Matching and triangulation of points
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	std::tie(triangulatedPoints3D, setsOfRays) = matchRaysAndTriangulate(cameras, points2D, rays);

	return glm::distance(triangulatedPoints3D[0], triangulatedPoints3D[1]);
}
