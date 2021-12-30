#include "Triangulation.h"

#include <utils/warnoff.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/norm.hpp>

#include <dlib/optimization.h>
#include <utils/warnon.h>

using parameterVector = dlib::matrix<float, 3, 1>;

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
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	float totalError = 0.f;

	std::vector<glm::vec3> points3d;

	points3d.reserve(setsOfRays.size());
	for (const auto& pointProjections : setsOfRays)
	{
		if (pointProjections.size() <= 1)
		{
			// It's impossible to triangulate a point with only one projection
			// Add an infinity point
			points3d.emplace_back(
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max()
			);
			// Go on with the next point to triangulate
			continue;
		}

		// A point should at least have two projections to be triangulated
		assert(pointProjections.size() >= 2);

		std::vector<glm::mat4> currentProjectionMatrices;
		std::vector<glm::vec2> currentPoints2d;

		currentProjectionMatrices.reserve(pointProjections.size());
		currentPoints2d.reserve(pointProjections.size());

		for (const auto& pointProjection : pointProjections)
		{
			const auto& cameraIndex = pointProjection.first;
			const auto& pointIndex = pointProjection.second;

			const auto& camera = cameras[cameraIndex];
			// Camera matrix from 3D to viewport coordinates
			const auto& projectionMatrix = camera.mat();
			// 2D point in viewport coordinates
			auto point2D = camera.windowToViewport(points2d[cameraIndex][pointIndex]);

			currentProjectionMatrices.push_back(projectionMatrix);
			currentPoints2d.push_back(point2D);
		}

		// Triangulate the point
		float error;
		glm::vec3 point3d;
		std::tie(error, point3d) = triangulatePointFromMultipleViews(currentProjectionMatrices, currentPoints2d);

		// Sum up the re-projection errors
		totalError += error;

		// Add the point to the list of all points
		points3d.push_back(point3d);
	}

	return { totalError, points3d };
}
