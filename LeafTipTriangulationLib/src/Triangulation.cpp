#include "Triangulation.h"

#include <utils/warnoff.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/norm.hpp>

#include <dlib/optimization.h>
#include <utils/warnon.h>

using parameterVector = dlib::matrix<float, 3, 1>;

glm::mat4x3 removeZRow(const glm::mat4& m)
{
	glm::mat4x3 output(0.0f);

	for (int c = 0; c < 4; c++)
	{
		output[c][0] = m[c][0];
		output[c][1] = m[c][1];
		output[c][2] = m[c][3];
	}
	
	return output;
}

cv::Mat1f convertToOpenCV(const glm::mat4x3& m)
{
	cv::Mat1f output(3, 4);

	// OpenCV is row major, glm is column major
	output.at<float>(0, 0) = m[0][0];
	output.at<float>(0, 1) = m[1][0];
	output.at<float>(0, 2) = m[2][0];
	output.at<float>(0, 3) = m[3][0];

	output.at<float>(1, 0) = m[0][1];
	output.at<float>(1, 1) = m[1][1];
	output.at<float>(1, 2) = m[2][1];
	output.at<float>(1, 3) = m[3][1];

	output.at<float>(2, 0) = m[0][2];
	output.at<float>(2, 1) = m[1][2];
	output.at<float>(2, 2) = m[2][2];
	output.at<float>(2, 3) = m[3][2];
	
	return output;
}

cv::Vec2f convertToOpenCV(const glm::vec2& v)
{
	cv::Vec2f output;

	output[0] = v.x;
	output[1] = v.y;
	
	return output;
}

cv::Vec3f convertToOpenCV(const glm::vec3& v)
{
	cv::Vec3f output;

	output[0] = v.x;
	output[1] = v.y;
	output[2] = v.z;

	return output;
}

glm::vec3 convertToGlm(const cv::Vec3f& v)
{
	return {v[0], v[1], v[2]};
}

glm::vec3 convertToGlm(const cv::Vec3d& v)
{
	return { v[0], v[1], v[2] };
}

glm::vec4 convertToGlm(const cv::Vec4f& v)
{
	return { v[0], v[1], v[2], v[3] };
}

glm::mat4x3 convertToGlm(const cv::Mat1f& v)
{
	assert(v.rows == 3);
	assert(v.cols == 4);

	// OpenCV is row major, glm is column major
	return glm::mat4x3(
		// Column X
		v.at<float>(0, 0),
		v.at<float>(1, 0),
		v.at<float>(2, 0),

		// Column Y
		v.at<float>(0, 1),
		v.at<float>(1, 1),
		v.at<float>(2, 1),

		// Column Z
		v.at<float>(0, 2),
		v.at<float>(1, 2),
		v.at<float>(2, 2),

		// Column W
		v.at<float>(0, 3),
		v.at<float>(1, 3),
		v.at<float>(2, 3)
	);
}

cv::Vec2f projectPoint(const cv::Mat1f& H, const cv::Vec4f& m)
{
	assert(H.rows == 3);
	assert(H.cols == 4);

	cv::Mat1f result = H * m;

	// Divide by W
	return cv::Vec2f(
		result.at<float>(0) / result.at<float>(2),
		result.at<float>(1) / result.at<float>(2)
	);
}

parameterVector vectorToParameters(const cv::Vec3f& vector)
{
	parameterVector x;
	x(0) = vector[0];
	x(1) = vector[1];
	x(2) = vector[2];

	return x;
}

cv::Vec3f parametersToVector(const parameterVector& parameters)
{
	return {
		parameters(0),
		parameters(1),
		parameters(2)
	};
}

// Takes an input, run it through the model and compare it to the output
float residual(const std::pair<cv::Mat1f, cv::Vec2f>& data, const parameterVector& parameters)
{
	// Reconstruct the vector with the parameters
	const auto point = parametersToVector(parameters);
	const cv::Vec4f homogeneousPoint(point[0], point[1], point[2], 1.0f);

	// Get points from data
	const auto& homography = data.first;
	const auto& trueProjectedPoint = data.second;

	// Run the projection using glm, which is much faster
	const auto projectedPoint = projectPoint(
		convertToGlm(homography),
		convertToGlm(homogeneousPoint)
	);

	const auto error = cv::norm(convertToOpenCV(projectedPoint), trueProjectedPoint, cv::NORM_L2SQR);

	if (std::isnan(error))
	{
		std::cout << trans(parameters) << std::endl;
		// std::cout << projectedPoint << " " << trueProjectedPoint << " " << error << std::endl;
		std::cout << std::endl << std::endl;
	}

	return error;
}

float reprojectionAdjustment(
	const std::vector<cv::Mat1f>& homographies,
	const std::vector<cv::Vec2f>& points,
	cv::Vec3f& pointToAdjust)
{
	assert(homographies.size() == points.size());

	std::vector<std::pair<cv::Mat1f, cv::Vec2f>> data;
	data.reserve(homographies.size() + points.size());
	for (unsigned int i = 0; i < homographies.size(); i++)
	{
		data.emplace_back(homographies[i], points[i]);
	}

	auto x = vectorToParameters(pointToAdjust);

	// Optimization
	const float finalCost = dlib::solve_least_squares(dlib::objective_delta_stop_strategy(1e-8),
		                                              residual,
		                                              derivative(residual, 1e-6),
		                                              data,
		                                              x);

	// Override the initial guess with the adjusted point
	pointToAdjust = parametersToVector(x);

	return finalCost;
}

float reprojectionError(
	const std::vector<cv::Mat1f>& homographies,
	const std::vector<cv::Vec2f>& points,
	const cv::Vec3f& point3d)
{
	assert(homographies.size() == points.size());

	const auto x = vectorToParameters(point3d);

	// Optimization
	float finalCost = 0.f; 

	for (unsigned int i = 0; i < homographies.size(); i++)
	{
		finalCost += residual({ homographies[i], points[i] }, x);
	}

	return finalCost;
}

std::tuple<float, cv::Vec3f> reconstructPointFromViews(
	const std::vector<cv::Mat1f>& homographies,
	const std::vector<cv::Vec2f>& points)
{
	assert(homographies.size() == points.size());

	const auto numberViews = int(homographies.size());

	// For the linear system Ax=b to solve
	cv::Mat1f A(3 * numberViews, 3 + numberViews, 0.0f);
	cv::Mat1f b(3 * numberViews, 1, 0.0f);
	for (int v = 0; v < numberViews; v++)
	{
		// Copy the 3x3 rotation matrix from this view, to the left of the matrix A
		const auto& R = homographies[v](cv::Range(0, 3), cv::Range(0, 3));
		A(cv::Range(3 * v, 3 * (v + 1)), cv::Range(0, 3)) = R * 1.f;

		b.at<float>(3 * v) = -homographies[v].at<float>(0, 3);
		b.at<float>(3 * v + 1) = -homographies[v].at<float>(1, 3);
		b.at<float>(3 * v + 2) = -homographies[v].at<float>(2, 3);

		// Copy the coordinates of the corresponding point in the matrix A
		A.at<float>(3 * v, 3 + v) = -points[v][0];
		A.at<float>(3 * v + 1, 3 + v) = -points[v][1];
		A.at<float>(3 * v + 2, 3 + v) = -1.f;
	}

	// Solve for linear least squares
	cv::Mat1f x;
	cv::solve(A, b, x, cv::DECOMP_SVD);

	cv::Vec3f triangulatedPoint(
		x.at<float>(0),
		x.at<float>(1),
		x.at<float>(2)
	);
	
	// Non-linear optimization to refine the result
	const auto finalError = reprojectionAdjustment(homographies, points, triangulatedPoint);

	// Alternative: no refinement, better speed but less accuracy
	// const auto finalError = reprojectionError(homographies, points, triangulatedPoint);

	// Return the triangulated point and its reprojection error
	return { finalError, triangulatedPoint };
}

// -----------------------------------------
// New interface with GLM instead of OpenCV
// -----------------------------------------

// ------------------------ Private functions ------------------------

parameterVector glmVec3ToParameters(const glm::vec3& vector)
{
	parameterVector x;
	x(0) = vector.x;
	x(1) = vector.y;
	x(2) = vector.z;

	return x;
}

glm::vec3 parametersToGlmVec3(const parameterVector& parameters)
{
	return {
		parameters(0),
		parameters(1),
		parameters(2)
	};
}

/**
 * \brief Compute the residual for one point in the bundle adjustment
 * \param data The projection matrix and the 2D projected point
 * \param parameters Coordinates of the 3D point for which to compute the residual
 * \return The reprojection error of the 3D point on the given 2D view
 */
float triangulationBundleAdjustmentResidual(
	const std::pair<glm::mat4, glm::vec2>& data,
	const parameterVector& parameters)
{
	// Get points from data
	const auto& projectionMatrix = data.first;
	const auto& trueProjectedPoint = data.second;

	// Reconstruct the vector with the parameters
	const auto point = parametersToGlmVec3(parameters);
	const glm::vec4 homogeneousPoint(point[0], point[1], point[2], 1.0f);

	const auto projectedPoint = projectPoint(projectionMatrix, homogeneousPoint);

	// Squared distance
	const auto error = glm::distance2(projectedPoint, trueProjectedPoint);

	if (std::isnan(error))
	{
		// TODO: use spdlog instead
		std::cout << trans(parameters) << std::endl;
		// std::cout << projectedPoint << " " << trueProjectedPoint << " " << error << std::endl;
		std::cout << std::endl << std::endl;
	}

	return error;
}

/**
 * \brief Refine the triangulation of a point using bundle adjustment
 * \param projectionMatrices The projection matrices of all views, projecting 3D points to 2D points.
 *                           Instead of 3x4 projection matrices, the input is the full 4x4 projection matrix.
 *							 The algorithm simply ignore the third row, which corresponds to the Z coordinate.
 * \param points2d The 2D points from all views
 * \param pointToAdjust The 3D point to refine
 * \return The final reprojection error
 */
float triangulationBundleAdjustment(
	const std::vector<glm::mat4>& projectionMatrices,
	const std::vector<glm::vec2>& points2d,
	glm::vec3& pointToAdjust
)
{
	assert(projectionMatrices.size() == points2d.size());

	std::vector<std::pair<glm::mat4, glm::vec2>> data;
	data.reserve(projectionMatrices.size() + points2d.size());
	for (unsigned int i = 0; i < projectionMatrices.size(); i++)
	{
		data.emplace_back(projectionMatrices[i], points2d[i]);
	}

	auto x = glmVec3ToParameters(pointToAdjust);

	// Optimization
	const float finalCost = dlib::solve_least_squares(dlib::objective_delta_stop_strategy(1e-8),
	                                                  triangulationBundleAdjustmentResidual,
	                                                  derivative(triangulationBundleAdjustmentResidual, 1e-6),
	                                                  data,
	                                                  x);

	// Override the initial guess with the adjusted point
	pointToAdjust = parametersToGlmVec3(x);

	return finalCost;
}

// ------------------------ Public functions ------------------------

glm::vec2 projectPoint(const glm::mat4x3& projectionMatrix, const glm::vec4& point)
{
	const auto result = projectionMatrix * point;

	return glm::vec2(
		result[0] / result[2],
		result[1] / result[2]
	);
}

glm::vec2 projectPoint(const glm::mat4& projectionMatrix, const glm::vec4& point)
{
	const auto result = projectionMatrix * point;

	return glm::vec2(
		result[0] / result[3],
		result[1] / result[3]
	);
}

float reprojectionErrorFromMultipleViews(
	const std::vector<glm::mat4>& projectionMatrices,
	const std::vector<glm::vec2>& points2d,
	const glm::vec3& point3d)
{
	assert(projectionMatrices.size() == points2d.size());

	const auto x = glmVec3ToParameters(point3d);
	
	float finalCost = 0.f;
	for (unsigned int i = 0; i < projectionMatrices.size(); i++)
	{
		finalCost += triangulationBundleAdjustmentResidual({ projectionMatrices[i], points2d[i] }, x);
	}

	return finalCost;
}

std::tuple<float, glm::vec3> triangulatePointFromMultipleViews(
	const std::vector<glm::mat4>& projectionMatrices,
	const std::vector<glm::vec2>& points2d)
{
	assert(projectionMatrices.size() == points2d.size());

	const auto numberViews = static_cast<int>(projectionMatrices.size());

	// For the linear system Ax=b to solve
	cv::Mat1f A(3 * numberViews, 3 + numberViews, 0.0f);
	cv::Mat1f b(3 * numberViews, 1, 0.0f);
	for (int v = 0; v < numberViews; v++)
	{
		// Copy the 3x3 rotation matrix from this view, to the left of the matrix A

		// First row of the 3x3 rotation matrix
		A.at<float>(3 * v, 0) = projectionMatrices[v][0][0];
		A.at<float>(3 * v, 1) = projectionMatrices[v][1][0];
		A.at<float>(3 * v, 2) = projectionMatrices[v][2][0];

		// Second row of the 3x3 rotation matrix
		A.at<float>(3 * v + 1, 0) = projectionMatrices[v][0][1];
		A.at<float>(3 * v + 1, 1) = projectionMatrices[v][1][1];
		A.at<float>(3 * v + 1, 2) = projectionMatrices[v][2][1];

		// The third row is the fourth row in the projection matrix,
		// because we ignore its third row since it corresponds to the Z coordinate.
		A.at<float>(3 * v + 2, 0) = projectionMatrices[v][0][3];
		A.at<float>(3 * v + 2, 1) = projectionMatrices[v][1][3];
		A.at<float>(3 * v + 2, 2) = projectionMatrices[v][2][3];

		// Copy the coordinates of the corresponding point in the matrix A
		A.at<float>(3 * v, 3 + v) = -points2d[v].x;
		A.at<float>(3 * v + 1, 3 + v) = -points2d[v].y;
		A.at<float>(3 * v + 2, 3 + v) = -1.f;

		b.at<float>(3 * v) = -projectionMatrices[v][3][0];
		b.at<float>(3 * v + 1) = -projectionMatrices[v][3][1];
		b.at<float>(3 * v + 2) = -projectionMatrices[v][3][3];
	}

	// Solve for linear least squares
	cv::Mat1f x;
	cv::solve(A, b, x, cv::DECOMP_SVD);

	glm::vec3 triangulatedPoint(
		x.at<float>(0),
		x.at<float>(1),
		x.at<float>(2)
	);
	
	// Non-linear optimization to refine the result
	const auto finalError = triangulationBundleAdjustment(projectionMatrices, points2d, triangulatedPoint);

	return { finalError, triangulatedPoint};
}

std::tuple<float, std::vector<glm::vec3>> triangulateManyPointsFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	float totalError = 0.f;

	std::vector<glm::vec3> points3D;

	points3D.reserve(setsOfRays.size());
	for (const auto& pointProjections : setsOfRays)
	{
		if (pointProjections.size() <= 1)
		{
			// It's impossible to triangulate a point with only one projection
			// Add an infinity point
			points3D.emplace_back(
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max()
			);
			// Go on with the next point to triangulate
			continue;
		}

		// A point should at least have two projections to be triangulated
		assert(pointProjections.size() >= 2);

		std::vector<cv::Mat1f> homographies;
		std::vector<cv::Vec2f> points;

		homographies.reserve(pointProjections.size());
		points.reserve(pointProjections.size());

		for (const auto& pointProjection : pointProjections)
		{
			const auto& cameraIndex = pointProjection.first;
			const auto& pointIndex = pointProjection.second;
			const auto& camera = cameras[cameraIndex];
			// Camera matrix from 3D to viewport coordinates
			const auto& cameraMat = camera.mat();
			// 2D point in viewport coordinates
			auto point2D = camera.windowToViewport(points2D[cameraIndex][pointIndex]);

			homographies.push_back(convertToOpenCV(removeZRow(cameraMat)));
			points.push_back(convertToOpenCV(point2D));
		}

		// Triangulate the point
		float error;
		cv::Vec3f point3D;
		std::tie(error, point3D) = reconstructPointFromViews(homographies, points);

		// Sum up the re-projection errors
		totalError += error;

		// Add the point to 
		points3D.push_back(convertToGlm(point3D));
	}

	return { totalError, points3D };
}
