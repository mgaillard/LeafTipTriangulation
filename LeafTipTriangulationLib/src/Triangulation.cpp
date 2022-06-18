#include "Triangulation.h"

#include <utils/warnoff.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/norm.hpp>

#include <dlib/optimization.h>

#include <spdlog/spdlog.h>
#include <utils/warnon.h>

using parameterVector = dlib::matrix<double, 3, 1>;

// ------------------------ Private functions ------------------------

parameterVector glmVec3ToParameters(const glm::dvec3& vector)
{
	parameterVector x;
	x(0) = vector.x;
	x(1) = vector.y;
	x(2) = vector.z;

	return x;
}

glm::dvec3 parametersToGlmVec3(const parameterVector& parameters)
{
	return {
		parameters(0),
		parameters(1),
		parameters(2)
	};
}

std::tuple<std::vector<glm::dmat4>, std::vector<glm::dvec2>>
getProjectionMatricesAndPoints(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::pair<int, int>>& setOfRays
)
{
	std::vector<glm::dmat4> currentProjectionMatrices;
	std::vector<glm::dvec2> currentPoints2d;

	currentProjectionMatrices.reserve(setOfRays.size());
	currentPoints2d.reserve(setOfRays.size());

	for (const auto& pointProjection : setOfRays)
	{
		const auto& cameraIndex = pointProjection.first;
		const auto& pointIndex = pointProjection.second;

		const auto& camera = cameras[cameraIndex];
		// Camera matrix from 3D to viewport coordinates
		const auto& projectionMatrix = camera.mat();
		// 2D point in viewport coordinates
		const auto point2d = camera.windowToViewport(points2d[cameraIndex][pointIndex]);

		currentProjectionMatrices.emplace_back(projectionMatrix);
		currentPoints2d.emplace_back(point2d);
	}

	return { currentProjectionMatrices, currentPoints2d };
}

/**
 * \brief Compute the residual for one point in the bundle adjustment
 * \param data The projection matrix and the 2D projected point
 * \param parameters Coordinates of the 3D point for which to compute the residual
 * \return The reprojection error of the 3D point on the given 2D view
 */
double triangulationBundleAdjustmentResidual(
	const std::pair<glm::dmat4, glm::dvec2>& data,
	const parameterVector& parameters)
{
	// Get points from data
	const auto& projectionMatrix = data.first;
	const auto& trueProjectedPoint = data.second;

	// Reconstruct the vector with the parameters
	const auto point = parametersToGlmVec3(parameters);
	const glm::dvec4 homogeneousPoint(point[0], point[1], point[2], 1.0);

	const auto projectedPoint = projectPoint(projectionMatrix, homogeneousPoint);

	// Squared distance
	const auto error = glm::distance(projectedPoint, trueProjectedPoint);

	if (std::isnan(error))
	{
		spdlog::error("nan value found during optimization");
		std::cout << trans(parameters) << "\n" << std::endl;
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
double triangulationBundleAdjustment(
	const std::vector<glm::dmat4>& projectionMatrices,
	const std::vector<glm::dvec2>& points2d,
	glm::dvec3& pointToAdjust
)
{
	assert(projectionMatrices.size() == points2d.size());

	std::vector<std::pair<glm::dmat4, glm::dvec2>> data;
	data.reserve(projectionMatrices.size() + points2d.size());
	for (unsigned int i = 0; i < projectionMatrices.size(); i++)
	{
		data.emplace_back(projectionMatrices[i], points2d[i]);
	}

	auto x = glmVec3ToParameters(pointToAdjust);

	// Optimization
	const auto finalCost = dlib::solve_least_squares(dlib::objective_delta_stop_strategy(1e-8),
	                                                 triangulationBundleAdjustmentResidual,
	                                                 derivative(triangulationBundleAdjustmentResidual, 1e-6),
	                                                 data,
	                                                 x);

	// Override the initial guess with the adjusted point
	pointToAdjust = parametersToGlmVec3(x);

	return finalCost;
}

// ------------------------ Public functions ------------------------

glm::dvec2 projectPoint(const glm::dmat4x3& projectionMatrix, const glm::dvec4& point)
{
	const auto result = projectionMatrix * point;

	return {
		result[0] / result[2],
		result[1] / result[2]
	};
}

glm::dvec2 projectPoint(const glm::dmat4& projectionMatrix, const glm::dvec4& point)
{
	const auto result = projectionMatrix * point;

	return {
		result[0] / result[3],
		result[1] / result[3]
	};
}

double reprojectionErrorFromMultipleViews(
	const std::vector<glm::dmat4>& projectionMatrices,
	const std::vector<glm::dvec2>& points2d,
	const glm::dvec3& point3d)
{
	assert(projectionMatrices.size() == points2d.size());

	const auto x = glmVec3ToParameters(point3d);
	
	double finalCost = 0.0;
	for (unsigned int i = 0; i < projectionMatrices.size(); i++)
	{
		// Reprojection error of the point3d with the point2d
		const auto dist = triangulationBundleAdjustmentResidual({ projectionMatrices[i], points2d[i] }, x);
		// Squared distance to compute the non-linear least square objective function
		finalCost += dist * dist;
	}

	return finalCost / 2.0;
}

std::tuple<double, glm::dvec3> triangulatePointFromMultipleViews(
	const std::vector<glm::dmat4>& projectionMatrices,
	const std::vector<glm::dvec2>& points2d)
{
	assert(projectionMatrices.size() == points2d.size());

	const auto numberViews = static_cast<int>(projectionMatrices.size());

	// For the linear system Ax=b to solve
	cv::Mat1d A(3 * numberViews, 3 + numberViews, 0.0);
	cv::Mat1d b(3 * numberViews, 1, 0.0);
	for (int v = 0; v < numberViews; v++)
	{
		// Copy the 3x3 rotation matrix from this view, to the left of the matrix A

		// First row of the 3x3 rotation matrix
		A.at<double>(3 * v, 0) = projectionMatrices[v][0][0];
		A.at<double>(3 * v, 1) = projectionMatrices[v][1][0];
		A.at<double>(3 * v, 2) = projectionMatrices[v][2][0];

		// Second row of the 3x3 rotation matrix
		A.at<double>(3 * v + 1, 0) = projectionMatrices[v][0][1];
		A.at<double>(3 * v + 1, 1) = projectionMatrices[v][1][1];
		A.at<double>(3 * v + 1, 2) = projectionMatrices[v][2][1];

		// The third row is the fourth row in the projection matrix,
		// because we ignore its third row since it corresponds to the Z coordinate.
		A.at<double>(3 * v + 2, 0) = projectionMatrices[v][0][3];
		A.at<double>(3 * v + 2, 1) = projectionMatrices[v][1][3];
		A.at<double>(3 * v + 2, 2) = projectionMatrices[v][2][3];

		// Copy the coordinates of the corresponding point in the matrix A
		A.at<double>(3 * v, 3 + v) = -points2d[v].x;
		A.at<double>(3 * v + 1, 3 + v) = -points2d[v].y;
		A.at<double>(3 * v + 2, 3 + v) = -1.0;

		b.at<double>(3 * v) = -projectionMatrices[v][3][0];
		b.at<double>(3 * v + 1) = -projectionMatrices[v][3][1];
		b.at<double>(3 * v + 2) = -projectionMatrices[v][3][3];
	}

	// Solve for linear least squares
	cv::Mat1d x;
	cv::solve(A, b, x, cv::DECOMP_SVD);

	glm::dvec3 triangulatedPoint(
		x.at<double>(0),
		x.at<double>(1),
		x.at<double>(2)
	);
	
	// Non-linear optimization to refine the result
	const auto finalError = triangulationBundleAdjustment(projectionMatrices, points2d, triangulatedPoint);

	return { finalError, triangulatedPoint};
}

std::tuple<float, glm::vec3> triangulatePointFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::pair<int, int>>& setOfRays)
{
	if (setOfRays.size() <= 1)
	{
		// It's impossible to triangulate a point with only one projection
		// Use an infinity point instead
		return {
			std::numeric_limits<float>::max(),
			{
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max()
			}
		};
	}

	// A point should at least have two projections to be triangulated
	assert(setOfRays.size() >= 2);

	const auto [currentProjectionMatrices, currentPoints2d] = getProjectionMatricesAndPoints(cameras, points2d, setOfRays);
	return triangulatePointFromMultipleViews(currentProjectionMatrices, currentPoints2d);
}

float reprojectionErrorManyPointsFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	const std::vector<glm::vec3>& points3d)
{
	double totalError = 0.f;
	
	for (unsigned int i = 0; i < setsOfRays.size(); i++)
	{
		const auto& pointProjections = setsOfRays[i];

		if (pointProjections.size() <= 1)
		{
			// Go on with the next point to triangulate
			continue;
		}

		// A point should at least have two projections to be triangulated
		assert(pointProjections.size() >= 2);

		const auto [currentProjectionMatrices, currentPoints2d] = getProjectionMatricesAndPoints(cameras, points2d, pointProjections);
		const auto error = reprojectionErrorFromMultipleViews(currentProjectionMatrices, currentPoints2d, points3d[i]);

		// Sum up the re-projection errors
		totalError += error;
	}
	
	return static_cast<float>(totalError);
}

std::tuple<float, std::vector<glm::vec3>> triangulateManyPointsFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	float totalError = 0.f;

	std::vector<glm::vec3> points3d;

	points3d.reserve(setsOfRays.size());
	for (const auto& setOfRays : setsOfRays)
	{
		const auto [error, point3d] = triangulatePointFromMultipleViews(cameras, points2d, setOfRays);

		if (error < std::numeric_limits<float>::max())
		{
			// Sum up the re-projection errors if the point could be triangulated
			totalError += error;
		}

		// Add the point to the list of all points
		points3d.push_back(point3d);
	}

	return { totalError, points3d };
}
