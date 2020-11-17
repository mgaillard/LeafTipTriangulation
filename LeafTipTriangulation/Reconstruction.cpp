#include "Reconstruction.h"

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>


#include "ExportScene.h"
#include "RayMatching.h"
#include "Triangulation.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

float radiansToDegrees(float radians)
{
	return radians * float(180.0 / M_PI);
}

std::pair<float, float> focalLengthInMm(const cv::Mat1f& cameraMatrix, const cv::Size& imageSize, const cv::Size2f& sensorSize)
{
	return {
		cameraMatrix.at<float>(0, 0) * sensorSize.width / imageSize.width,
		cameraMatrix.at<float>(1, 1) * sensorSize.height / imageSize.height
	};
}

cv::Mat rotationX180(const cv::Mat1f& matrix)
{
	// 180 degrees rotation matrix
	cv::Mat1f R(3, 3, 0.0f);

	R.at<float>(0, 0) = 1.0f;
	R.at<float>(1, 1) = -1.0f;
	R.at<float>(2, 2) = -1.0f;

	const cv::Mat1f rotationResult = R * matrix;

	return rotationResult.clone();
}

cv::Mat1f removeZProjectionMatrix(const cv::Mat& projectionMatrix)
{
	cv::Mat1f H(3, 3, 0.0f);

	H.at<float>(0, 0) = projectionMatrix.at<float>(0, 0);
	H.at<float>(1, 0) = projectionMatrix.at<float>(1, 0);
	H.at<float>(2, 0) = projectionMatrix.at<float>(2, 0);

	H.at<float>(0, 1) = projectionMatrix.at<float>(0, 1);
	H.at<float>(1, 1) = projectionMatrix.at<float>(1, 1);
	H.at<float>(2, 1) = projectionMatrix.at<float>(2, 1);

	H.at<float>(0, 2) = projectionMatrix.at<float>(0, 3);
	H.at<float>(1, 2) = projectionMatrix.at<float>(1, 3);
	H.at<float>(2, 2) = projectionMatrix.at<float>(2, 3);

	H /= H.at<float>(2, 2);

	return H;
}


cv::Mat1f cameraPose(const cv::Mat1f& rvec, const cv::Mat1f& tvec)
{
	cv::Mat1f R;
	cv::Rodrigues(rvec, R); // R is 3x3

	auto invTvec = -R.t() * tvec; // translation of inverse
	R = rotationX180(R.t());  // rotation of inverse

	cv::Mat1f T = cv::Mat1f::eye(4, 4); // T is 4x4
	T(cv::Range(0, 3), cv::Range(0, 3)) = R.t() * 1; // copies R into T
	T(cv::Range(0, 3), cv::Range(3, 4)) = invTvec * 1; // copies tvec into T

	// To get the Euler angles (XYZ)
	// Go to https://www.andre-gaschler.com/rotationconverter/
	// Copy the rotation matrix R
	// Input the angles (in degrees) in blender

	return T;
}

cv::Vec3f cameraPoseVectorX(const cv::Mat1f& rvec)
{
	cv::Mat1f R;
	cv::Rodrigues(rvec, R); // R is 3x3

	// X is the first column of the R transposed matrix
	cv::Mat1f T = R.t() * 1.f; // copies R into T

	return {
		T.at<float>(0, 0),
		T.at<float>(1, 0),
		T.at<float>(2, 0)
	};
}

// Source: https://answers.opencv.org/question/162932/create-a-stereo-projection-matrix-using-rvec-and-tvec/
cv::Mat1f computeProjectionMatrix(const cv::Mat1f& cameraMatrix, const cv::Mat1f& rvec, const cv::Mat1f& tvec)
{
	cv::Mat1f rotMat(3, 3), rotTransMat(3, 4);
	// Convert rotation vector into rotation matrix 
	cv::Rodrigues(rvec, rotMat);
	// Append translation vector to rotation matrix
	cv::hconcat(rotMat, tvec, rotTransMat);
	// Compute projection matrix by multiplying intrinsic parameter 
	// matrix (A) with 3 x 4 rotation and translation pose matrix (RT).
	// Formula: Projection Matrix = A * RT;
	return cameraMatrix * rotTransMat;
}

cv::Vec3f lookAtPoint(const cv::Mat1f& homography, const cv::Mat1f& cameraMatrix)
{
	assert(homography.rows == 3);
	assert(homography.cols == 4);

	// For the linear system Ax=b to solve
	cv::Mat1f A(3, 3, 0.0f);
	A.at<float>(0, 0) = homography.at<float>(0, 0);
	A.at<float>(0, 1) = homography.at<float>(0, 1);
	A.at<float>(1, 0) = homography.at<float>(1, 0);
	A.at<float>(1, 1) = homography.at<float>(1, 1);
	A.at<float>(2, 0) = homography.at<float>(2, 0);
	A.at<float>(2, 1) = homography.at<float>(2, 1);

	A.at<float>(0, 2) = -cameraMatrix.at<float>(0, 2);
	A.at<float>(1, 2) = -cameraMatrix.at<float>(1, 2);
	A.at<float>(2, 2) = -1.0f;

	cv::Mat1f b(3, 1, 0.0f);
	b.at<float>(0) = -homography.at<float>(0, 3);
	b.at<float>(1) = -homography.at<float>(1, 3);
	b.at<float>(2) = -homography.at<float>(2, 3);

	// Solve for linear least squares
	cv::Mat1f x;
	cv::solve(A, b, x, cv::DECOMP_SVD);

	// Convert to vector
	return {
		x.at<float>(0),
		x.at<float>(1),
		0.0
	};
}

std::tuple<glm::vec3, glm::vec3, glm::vec3> cameraEyeAtUpFromPose(
	const cv::Mat1f& cameraMatrix,
	const cv::Mat1f& rvec,
	const cv::Mat1f& tvec
)
{
	const auto homography = computeProjectionMatrix(cameraMatrix, rvec, tvec);
	const auto pose = cameraPose(rvec, tvec);

	// Find position of the camera in world coordinates
	const glm::vec3 eye(
		pose.at<float>(0, 3),
		pose.at<float>(1, 3),
		pose.at<float>(2, 3)
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

float measureTwoPointsCharuco(
	const std::vector<std::string>& imageFiles,
	const std::vector<std::vector<glm::vec2>>& inputPoints2D)
{
	// Configuration of the Charuco board
	const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	const auto board = cv::aruco::CharucoBoard::create(11, 8, 0.02f, 0.015f, dictionary);
	auto params = cv::aruco::DetectorParameters::create();
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;

	// Directories for images
	const std::string directory = "crocodile/pictures/";
	const std::string directoryDetected = "crocodile/detected/";
	const std::string directoryUndistorted = "crocodile/undistorted/";

	// Generate a board image
	cv::Mat boardImage;
	board->draw(cv::Size(2159, 2794), boardImage, 10, 1);
	cv::imwrite("crocodile/charuco_pattern.png", boardImage);

	// Original images
	std::vector<cv::Mat> imagesRaw(imageFiles.size());
	// Undistorded images
	std::vector<cv::Mat> images(imageFiles.size());

	// Markers and corners for calibration patterns
	std::vector<std::vector<std::vector<cv::Point2f>>> markerCorners(imageFiles.size());
	std::vector<std::vector<int>> markerIds(imageFiles.size());
	std::vector<std::vector<cv::Point2f>> charucoCorners(imageFiles.size());
	std::vector<std::vector<int>> charucoIds(imageFiles.size());

	// Camera calibration
	cv::Mat cameraMatrix, distCoeffs;
	// Cameras poses
	std::vector<cv::Mat> rvecs, tvecs;

	// Load original images
#pragma omp parallel for
	for (int i = 0; i < imageFiles.size(); i++)
	{
		// Read image from file
		imagesRaw[i] = cv::imread(directory + imageFiles[i]);

		// Detect markers
		cv::aruco::detectMarkers(imagesRaw[i],
			board->dictionary,
			markerCorners[i],
			markerIds[i],
			params);

		// If markers are available
		if (!markerIds[i].empty())
		{
			// Detect corners
			cv::aruco::interpolateCornersCharuco(markerCorners[i],
				markerIds[i],
				imagesRaw[i],
				board,
				charucoCorners[i],
				charucoIds[i]);
		}
	}

	// Camera properties
	const auto imageWidth = imagesRaw.front().cols;
	const auto imageHeight = imagesRaw.front().rows;
	const cv::Size imageSize(imageWidth, imageHeight);

	// Run calibration
	cv::aruco::calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

	// Undistort images and save diagnostic images
#pragma omp parallel for
	for (int i = 0; i < imagesRaw.size(); i++)
	{
		cv::Mat diagImage;
		imagesRaw[i].copyTo(diagImage);
		// Draw information on diagnostic image
		cv::aruco::drawDetectedMarkers(diagImage, markerCorners[i], markerIds[i]);
		cv::aruco::drawDetectedCornersCharuco(diagImage, charucoCorners[i], charucoIds[i], cv::Scalar(255, 0, 0));
		cv::aruco::drawAxis(diagImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1f);
		// Save diagnostic image
		cv::imwrite(directoryDetected + imageFiles[i], diagImage);

		// Undistort each image
		cv::undistort(imagesRaw[i], images[i], cameraMatrix, distCoeffs);
		cv::imwrite(directoryUndistorted + imageFiles[i], images[i]);
	}

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
			point2D.x = outputUndistortedPoints.front().x;
			point2D.y = float(imageHeight) - outputUndistortedPoints.front().y;
		}
	}

	// Since images are undistorted, we can now set the distortion coefficients to zero
	distCoeffs.setTo(0.0f);

	// Compute aspect ratio from images
	const auto aspectRatio = float(imageWidth) / float(imageHeight);

	// For a Google Pixel 3, the sensor is 5.76 mm by 4.29 mm
	const cv::Size2f sensorSize(5.76f, 4.29f);
	const auto focalLength = focalLengthInMm(cameraMatrix, imageSize, sensorSize);
	const auto fovy = 2.0f * std::atan(sensorSize.height / (2.0f * focalLength.second));

	std::vector<Camera> cameras;
	for (int i = 0; i < images.size(); i++)
	{
		glm::vec3 eye, at, up;
		std::tie(eye, at, up) = cameraEyeAtUpFromPose(cameraMatrix, rvecs[i], tvecs[i]);

		cameras.emplace_back(eye, at, up,
			                 fovy, aspectRatio,
			                 glm::vec2(imageWidth, imageHeight));
	}

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(cameras, points2D);

	// Matching and triangulation of points
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	std::tie(triangulatedPoints3D, setsOfRays) = matchRaysAndTriangulate(cameras, points2D, rays);

	// Export the scene
	exportSplitSceneAsOBJ(rays, setsOfRays, triangulatedPoints3D);

	return glm::distance(triangulatedPoints3D[0], triangulatedPoints3D[1]);
}
