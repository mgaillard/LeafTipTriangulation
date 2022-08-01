#include "RayMatching.h"

#include <numeric>

#include <utils/warnoff.h>
#include <glm/gtc/matrix_transform.hpp>

#include <dlib/optimization/max_cost_assignment.h>

#include <spdlog/spdlog.h>
#include <utils/warnon.h>


#include "Triangulation.h"

// ------------------------------------------
// Headers for private members of this module
// ------------------------------------------

/**
 * \brief Convert a vector of booleans to a an unsigned integer
 * \param v A vector of bool
 * \return The integer value of the vector of bool
 */
std::size_t convertVectorBoolToInt(const std::vector<bool>& v);

/**
 * \brief Strategy for measuring errors in the similarity function 
 */
enum class SimilarityStrategy
{	
	/**
	 * \brief Output only the re-projection error of point 0
	 */
	OnlyPoint0,

	/**
	 * \brief Output only the re-projection error of point 1
	 */
	OnlyPoint1,

	/**
	 * \brief Output the re-projection error of both points (default strategy)
	 */
	BothPoints
};

/**
 * \brief Compute a similarity coefficient between two points projected on two cameras
 * \param camera0 Camera A
 * \param ray0 Ray A
 * \param point0 2D Point A
 * \param camera1 Camera B
 * \param ray1 Ray B
 * \param point1 2D Point B
 * \param strategy Strategy to measure the error, see SimilarityStrategy
 * \return The similarity coefficient between two rays
 */
float similarity(const Camera& camera0,
                 const Ray& ray0,
                 const glm::vec2& point0,
                 const Camera& camera1,
                 const Ray& ray1,
                 const glm::vec2& point1,
				 SimilarityStrategy strategy = SimilarityStrategy::BothPoints);

/**
 * \brief Structure to hold a cache entry for the function matchRaysAndTriangulate.
 */
struct MatchRaysAndTriangulateDPCacheEntry
{
	bool valid;
	std::vector<glm::vec3> triangulatedPoints;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;

	MatchRaysAndTriangulateDPCacheEntry() : valid(false)
	{
	}

	void set(
		const std::vector<glm::vec3>& newTriangulatedPoints,
		const std::vector<std::vector<std::pair<int, int>>>& newSetsOfRays)
	{
		triangulatedPoints = newTriangulatedPoints;
		setsOfRays = newSetsOfRays;
		valid = true;
	}

	std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> asTuple() const
	{
		return {triangulatedPoints, setsOfRays};
	}
};

/**
 * \brief Find a matching of rays between multiple cameras.
 *		  Warning: This function is an approximation and does not work in the general case.
 * \param cameras A list of cameras
 * \param points2D A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param thresholdNoPair Threshold in px above which two rays can't be paired together
 * \return The matching of rays that best triangulates 2D points projected by cameras
 */
std::vector<std::vector<std::pair<int, int>>> findSetsOfRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	float thresholdNoPair);

/**
 * \brief Triangulate many points in 3D from two 2D views using the pseudo intersection of rays
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param setsOfRays Correspondences of the points in the 2D views
 * \return All 3D triangulated points
 */
std::vector<glm::vec3> pseudoIntersectionManyPointsFromTwoViews(
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Match each ray to a 3D point based on the re-projected error
 * \param cameras A list of cameras
 * \param points2d A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param points3d A set of points in 3D
 * \param setsOfRays The matching of rays used to triangulate the 3D points
 * \param cameraIndex The index of the camera for which to do the matching
 * \param thresholdNoPair Threshold in px above which two rays/points can't be paired together
 * \return The new set of rays taking in account the assignment between the 3D points and the rays
 */
std::vector<std::vector<std::pair<int, int>>> pointsRaysMatching(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<glm::vec3>& points3d,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	int cameraIndex,
	float thresholdNoPair);


/**
 * \brief Dynamic programming implementation of the function matchRaysAndTriangulate
 *        Recursively divide the problem into sub problems
 *        A cache structure is used for memoization
 * \param cacheDP Cache to store the results of this function for reusing them later
 * \param cameras The list of cameras
 * \param points2D List of 2D points per camera
 * \param rays List of rays associated to 2D points per camera
 * \param activeCameraSet The active set of cameras as a vector of bool: true, camera activated false camera deactivated
 * \param thresholdNoPair Threshold in px above which two rays/points can't be paired together
 * \return The triangulated points and matching of rays for the current active camera set
 */
std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulateDP(
	std::vector<MatchRaysAndTriangulateDPCacheEntry>& cacheDP,
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<bool>& activeCameraSet,
	float thresholdNoPair);

// ------------------------------------------------
// Implementation of private members of this module
// ------------------------------------------------

std::size_t convertVectorBoolToInt(const std::vector<bool>& v)
{
	// std::size_t can only fit 64 bits
	assert(v.size() <= sizeof(std::size_t) * 8);

	return std::accumulate(v.rbegin(), v.rend(), std::size_t(0),
	                       [](std::size_t x, std::size_t y)
	                       {
		                       return (x << 1) + y;
	                       });
}

float similarity(
	const Camera& camera0,
	const Ray& ray0,
	const glm::vec2& point0,
	const Camera& camera1,
	const Ray& ray1,
	const glm::vec2& point1,
	SimilarityStrategy strategy)
{
	// See: Triangulation without correspondences from Cheng et al.
	const std::array<glm::vec3, 2> u = {{ray0.direction, ray1.direction}};

	// Closest point between the two lines
	glm::vec3 q;
	const auto pseudoIntersectionExists = raysPseudoIntersection(ray0, ray1, q);

	// Failure case 1: ray0 and ray1 are parallel
	if (!pseudoIntersectionExists)
	{
		return std::numeric_limits<float>::max();
	}
	
	// Compute the re-projection of q on the two views
	const auto q0 = camera0.project(q);
	const auto q1 = camera1.project(q);

	// Failure case 2: the pseudo-intersection is on the negative side of one of the rays
	const std::array<glm::vec3, 2> dirQ = {
		glm::normalize(q - ray0.origin),
		glm::normalize(q - ray1.origin)
	};
	const auto dotProduct0 = glm::dot(u[0], dirQ[0]);
	const auto dotProduct1 = glm::dot(u[1], dirQ[1]);
	
	// Check depth: the point must be visible from the camera and not behind it
	if ((dotProduct0 <= 0.f) || (dotProduct1 <= 0.f)
	 || (q0.z <= 0.f) || (q1.z <= 0.f))
	{
		return std::numeric_limits<float>::max();
	}

	// Compute re-projection distance
	auto result = std::numeric_limits<float>::max();

	switch (strategy)
	{
		// Use only point 0 to compute the re-projection error
	case SimilarityStrategy::OnlyPoint0:
		result = glm::distance(point0, glm::vec2(q0));
		break;
		
		// Use only point 1 to compute the re-projection error
	case SimilarityStrategy::OnlyPoint1:
		result = glm::distance(point1, glm::vec2(q1));
		break;

		// Use both point 0 and point 1 to compute the re-projection error
	case SimilarityStrategy::BothPoints:
	default:
		result = glm::distance(point0, glm::vec2(q0)) + glm::distance(point1, glm::vec2(q1));
		break;
	};

	return result;
}

std::vector<std::vector<std::pair<int, int>>> findSetsOfRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	float thresholdNoPair)
{
	// Compute an upper bound on the maximum distance between two pixel with the camera
	const auto maximumDistancePixels = computeMaximumCameraResolution(cameras) * std::sqrt(2.f);
	// Make sure that this distance is about 1G for the maximum distance in the image
	constexpr long maximumSimilarity = 1000000000;

	// Multiplier used to convert a floating point value to an integer value
	const double realToLongMultiplier = static_cast<double>(maximumSimilarity) / maximumDistancePixels;

	// Find the view with the maximum number of points in 2D
	unsigned int referenceCamera = 0;
	for (unsigned int i = 0; i < points2D.size(); i++)
	{
		if (points2D[i].size() > points2D[referenceCamera].size())
		{
			referenceCamera = i;
		}
	}

	// A list of 3D points defined by a list of index of rays
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;

	// Setup rays for the reference Camera
	for (unsigned int i = 0; i < rays[referenceCamera].size(); i++)
	{
		// Add a 3D point defined by the intersection of 1 ray from the reference camera
		// After matching these rays with other cameras, other rays will be added
		setsOfRays.push_back(
			{ {referenceCamera, i} }
		);
	}

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		if (c == referenceCamera)
		{
			continue;
		}

		const int costSize = static_cast<int>(rays[referenceCamera].size());
		const int costCols = static_cast<int>(rays[c].size());

		assert(costSize >= costCols);

		// Compare the referenceCamera to this camera
		// Since the cost matrix is not necessarily square
		// costSize is the size of the square
		// costCols is the true number of columns, which is less than costSize
		// We add dummy columns to account for the fact that
		// we are looking for a matching between two sets of different size
		dlib::matrix<long> cost(costSize, costSize);
		for (int i = 0; i < costSize; i++)
		{
			for (int j = 0; j < costCols; j++)
			{
				const auto dist = similarity(cameras[referenceCamera],
					rays[referenceCamera][i],
					points2D[referenceCamera][i],
					cameras[c],
					rays[c][j],
					points2D[c][j]);

				if (dist < thresholdNoPair)
				{
					const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
					cost(i, j) = -integerDist;
				}
				else
				{
					// if dist == std::numeric_limits<float>::max()
					// or if above a threshold, we push the algorithm not to match the two points
					cost(i, j) = -maximumSimilarity;
				}
			}

			// Add dummy columns, filled with min - 1
			for (int j = costCols; j < costSize; j++)
			{
				cost(i, j) = -maximumSimilarity;
			}
		}

		// Compute best assignment between pairs of points
		const auto assignment = dlib::max_cost_assignment(cost);

		for (unsigned int i = 0; i < assignment.size(); i++)
		{
			// The cost of the assignment: row `i` with column `assignment[i]`
			const auto associatedCost = cost(i, assignment[i]);

			// If the assigned ray is not one of the dummy ray we added
			if (assignment[i] < costCols)
			{
				// If it's not a match we prevented from happening because the two rays are two different
				if (associatedCost > -maximumSimilarity)
				{
					// Ray i from the reference camera is in setsOfRays[i]
					// We add the corresponding ray assignment[i] from camera c
					setsOfRays[i].emplace_back(c, assignment[i]);
				}
				else
				{
					// We found a ray in camera c that matches to no ray in the reference camera
					setsOfRays.push_back(
						{ {c, assignment[i]} }
					);
				}
			}
		}
	}

	return setsOfRays;
}

std::vector<glm::vec3> pseudoIntersectionManyPointsFromTwoViews(
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
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

		// A point should have exactly two projections to be triangulated
		assert(pointProjections.size() == 2);

		const auto& [cameraIndex0, rayIndex0] = pointProjections[0];
		const auto& [cameraIndex1, rayIndex1] = pointProjections[1];
		const auto& ray0 = rays[cameraIndex0][rayIndex0];
		const auto& ray1 = rays[cameraIndex1][rayIndex1];

		glm::vec3 pseudoIntersection;
		raysPseudoIntersection(ray0, ray1, pseudoIntersection);
		points3d.push_back(pseudoIntersection);
	}

	return points3d;
}

std::vector<std::vector<std::pair<int, int>>> pointsRaysMatching(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2d,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<glm::vec3>& points3d,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	int cameraIndex,
	float thresholdNoPair)
{
	// Check that we receive coherent sets 
	assert(points3d.size() == setsOfRays.size());
	
	// The camera index is not out of bounds
	assert(cameraIndex >= 0 && cameraIndex < cameras.size());
	
	const auto& currentCamera = cameras[cameraIndex];
	const auto& currentPoints2D = points2d[cameraIndex];
	const auto& currentRays = rays[cameraIndex];

	// The number of 3D points and single rays that we can match to the current camera
	// is greater than the number of 2D points projected on the camera
	assert(setsOfRays.size() >= currentPoints2D.size());
	assert(currentPoints2D.size() == currentRays.size());

	// Compute an upper bound on the reprojection error between two 2D points in viewport coordinates (between -1 and 1)
	// The maximum distance between two points is: 2*sqrt(2)
	// The reprojection error for this distance is: 0.5*2*sqrt(2)*2*sqrt(2) = 4
	constexpr auto maximumDeltaError = 4.0;

	// Number of 3D points and single rays available for matching
	const int costSize = static_cast<int>(setsOfRays.size());
	// Number of 2D points to match 
	const int costCols = static_cast<int>(currentPoints2D.size());

	// Compute the change in reprojection error for all assignments
	dlib::matrix<double> realCost(costSize, costSize);
	for (int i = 0; i < costSize; i++)
	{
		// Compute the current reprojection error for the 3D point i
		// If the 3D point i is a single ray, the reprojection is 0.0 by default
		double currentError = 0.0;

		if (setsOfRays[i].size() > 1)
		{
			currentError = reprojectionErrorFromMultipleViews(cameras, points2d, setsOfRays[i], points3d[i]);
		}

		// Try to assign a ray from the other camera to the point i
		for (int j = 0; j < costCols; j++)
		{
			// The change in error caused by assigning the 2D point j to the 3D point i
			double deltaError = 0.0;

			// Reprojection error of the 3D point i (or single ray) on 2D point j from the current camera
			// By default the distance is maximum to prevent the matching
			double dist = maximumDeltaError;
			
			// Check if point i is a 3D point or a single ray
			if (setsOfRays[i].size() > 1)
			{
				// Copy the current set of rays and add the candidate set of rays
				auto newSetOfRays = setsOfRays[i];
				newSetOfRays.emplace_back(cameraIndex, j);
				// Triangulate with this new set of rays
				const auto [newError, newTriangulatedPoint] = triangulatePointFromMultipleViews(cameras, points2d, newSetOfRays);

				// Compare to the 2D point j
				// Compute the re-projection error of the 3D point on the camera
				deltaError = static_cast<double>(newError) - currentError;
				
				// Project the triangulated 3D point i on the current camera
				const auto projectedPoint = glm::vec2(currentCamera.project(newTriangulatedPoint));
				dist = static_cast<double>(glm::distance(projectedPoint, currentPoints2D[j]));
			}
			else
			{
				// It's a single ray
				const auto& singleRay = setsOfRays[i].front();
				const auto singleRayCameraIndex = singleRay.first;
				const auto singleRayIndex = singleRay.second;

				// Compute the sum of the re-projection error for both views
				// Since the threshold is divided by too later, we need to also divide the similarity by 2
				// We do this to be consistent with the other case, when matching two views
				dist = similarity(cameras[singleRayCameraIndex],
				                  rays[singleRayCameraIndex][singleRayIndex],
				                  points2d[singleRayCameraIndex][singleRayIndex],
				                  currentCamera,
				                  currentRays[j],
				                  currentPoints2D[j],
								  SimilarityStrategy::BothPoints) / 2.0;


				// Triangulate the point from two views using the pseudo intersection
				glm::vec3 newTriangulatedPoint;
				const auto pseudoIntersectionExists = raysPseudoIntersection(rays[singleRayCameraIndex][singleRayIndex],
				                                                             currentRays[j],
				                                                             newTriangulatedPoint);
				// If the triangulated point exists, compute the deltaError
				// If the triangulated point does not exists, the deltaError will be set to the maximum value
				// because the similarity is equal to infinity
				if (pseudoIntersectionExists)
				{
					// Copy the current set of rays and add the candidate set of rays
					auto setOfRays = setsOfRays[i];
					setOfRays.emplace_back(cameraIndex, j);

					// Triangulate with this new set of rays
					const auto newError = reprojectionErrorFromMultipleViews(cameras, points2d, setOfRays, newTriangulatedPoint);

					// Compare to the 2D point j
					// Compute the re-projection error of the 3D point on the camera
					deltaError = static_cast<double>(newError);
				}
			}

			// Since we look at the reprojection error of the 3D point on only one 2D point, the threshold is divided by 2
			if (dist < static_cast<double>(thresholdNoPair) / 2.0)
			{
				realCost(i, j) = deltaError;
			}
			else
			{
				// if dist == std::numeric_limits<float>::max()
				// or if above the threshold, we push the algorithm not to match the two points
				realCost(i, j) = maximumDeltaError;
			}
		}

		// Add dummy columns
		for (int j = costCols; j < costSize; j++)
		{
			realCost(i, j) = maximumDeltaError;
		}
	}

	// Convert the real matrix to a integer matrix
	dlib::matrix<long> cost(costSize, costSize);
	// Compute the maximum magnitude of the change in reprojection error
	// Most likely maxMagnitudeCost = maximumDeltaError
	const auto maxMagnitudeCost = std::max(std::abs(dlib::max(realCost)), std::abs(dlib::min(realCost)));
	// Make sure that this distance is about 1G for the maximum distance in the image
	constexpr long maximumSimilarity = 1000000000;
	// Multiplier used to convert a floating point value to an integer value
	const double realToLongMultiplier = static_cast<double>(maximumSimilarity) / maxMagnitudeCost;

	for (int i = 0; i < costSize; i++)
	{
		for (int j = 0; j < costSize; j++)
		{
			const auto& deltaError = realCost(i, j);
			// Convert the deltaError to integer
			const auto integerDist = static_cast<long>(std::round(deltaError * realToLongMultiplier));
			cost(i, j) = -integerDist;
		}
	}

	// Compute best assignment between pairs of points
	const auto assignment = dlib::max_cost_assignment(cost);

	// Copy the set of rays and update it with 
	auto newSetsOfRays = setsOfRays;

	for (unsigned int i = 0; i < assignment.size(); i++)
	{
		// The cost of the assignment: row `i` with column `assignment[i]`
		const auto associatedCost = cost(i, assignment[i]);
		
		// If the assigned ray is not one of the dummy ray we added
		if (assignment[i] < costCols)
		{
			// If it's not a match we prevented from happening because the two rays are two different
			if (associatedCost > -maximumSimilarity)
			{
				// Ray i from the reference camera is in setsOfRays[i]
				// We add the corresponding ray assignment[i] from camera c
				newSetsOfRays[i].emplace_back(cameraIndex, assignment[i]);
			}
			else
			{
				// We found a ray in camera c that matches to no ray in the reference camera
				newSetsOfRays.push_back(
					{ {cameraIndex, assignment[i]} }
				);
			}
		}
	}

	return newSetsOfRays;
}

std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulateDP(
	std::vector<MatchRaysAndTriangulateDPCacheEntry>& cacheDP,
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<bool>& activeCameraSet,
	float thresholdNoPair)
{
	assert(points2D.size() == cameras.size());
	assert(rays.size() == cameras.size());
	assert(activeCameraSet.size() == cameras.size());

	// Lookup in the cache if the solution has already been computed
	// The cache is a map with the input active set converted as an integer and the output of the function
	const auto cacheIndex = convertVectorBoolToInt(activeCameraSet);
	assert(cacheIndex < cacheDP.size());
	if (cacheDP[cacheIndex].valid)
	{
		// If this sub problem as already been solved, return its cached value
		return cacheDP[cacheIndex].asTuple();
	}

	const auto numberActiveCameras = std::count(activeCameraSet.begin(), activeCameraSet.end(), true);

	// If we end up in the case with 2 cameras only
	if (numberActiveCameras == 2)
	{
		// Create vectors of camera only with the activeCameras
		std::vector<Camera> activateCameras;
		std::vector<std::vector<glm::vec2>> activatePoints2D;
		std::vector<std::vector<Ray>> activeRays;
		std::vector<int> indexActiveCamera;

		// Isolate the two active cameras, points and rays
		for (unsigned int i = 0; i < activeCameraSet.size(); i++)
		{
			if (activeCameraSet[i])
			{
				activateCameras.push_back(cameras[i]);
				activatePoints2D.push_back(points2D[i]);
				activeRays.push_back(rays[i]);
				indexActiveCamera.push_back(i);
			}
		}

		// Then run the already existing algorithm with only two cameras
		auto setsOfRays = findSetsOfRays(activateCameras, activatePoints2D, activeRays, thresholdNoPair);

		// Change the index of cameras in the setOfRays array to match the true index of cameras
		for (auto& setOfRays : setsOfRays)
		{
			for (auto& ray : setOfRays)
			{
				// Change the camera index
				ray.first = indexActiveCamera[ray.first];
			}
		}

		// Use pseudo-intersection to triangulate points instead of multiple-view triangulation
		const auto triangulatedPoints3D = pseudoIntersectionManyPointsFromTwoViews(rays, setsOfRays);

		// Store in cache
		cacheDP[cacheIndex].set(triangulatedPoints3D, setsOfRays);

		return {triangulatedPoints3D, setsOfRays};
	}
	// If there are more than 2 cameras, we split to smaller cases
	else if (numberActiveCameras > 2)
	{
		float bestReprojError = std::numeric_limits<float>::max();
		std::vector<std::vector<std::pair<int, int>>> bestSetsOfRays;
		std::vector<glm::vec3> bestTriangulatedPoints;

		// We deactivate one camera, and get the answer with one camera less
		for (unsigned int c = 0; c < activeCameraSet.size(); c++)
		{
			if (activeCameraSet[c])
			{
				// Copy the active camera vector and deactivate camera i
				auto subActiveCameraSet = activeCameraSet;
				subActiveCameraSet[c] = false;

				// Get the setOfRays and triangulated points from the sub problems with one camera less
				std::vector<glm::vec3> subPoints;
				std::vector<std::vector<std::pair<int, int>>> setsOfRays;
				std::tie(subPoints, setsOfRays) = matchRaysAndTriangulateDP(cacheDP,
					                                                        cameras,
					                                                        points2D,
					                                                        rays,
					                                                        subActiveCameraSet,
					                                                        thresholdNoPair);

				// If the number of points and rays identified in the sub problem
				// are less that the number of points in this camera, cancel the reconstruction
				if (rays[c].size() > setsOfRays.size())
				{
					// TODO: Try to add the ray alone in the pointsRaysMatching function
					continue;
				}

				// Match rays to subPoints and update setsOfRays with the rays in camera c
				setsOfRays = pointsRaysMatching(cameras, points2D, rays, subPoints, setsOfRays, c, thresholdNoPair);

				// Re-triangulate the points with the new rays
				// TODO: maybe just bundle adjusting is necessary
				float triangulationError;
				std::vector<glm::vec3> triangulatedPoints;
				std::tie(triangulationError, triangulatedPoints) = triangulateManyPointsFromMultipleViews(cameras, points2D, setsOfRays);

				// Compute the total re-projection error and update the best solution
				if (triangulationError < bestReprojError)
				{
					bestReprojError = triangulationError;
					bestSetsOfRays = setsOfRays;
					bestTriangulatedPoints = triangulatedPoints;
				}
			}
		}

		// Store in cache
		cacheDP[cacheIndex].set(bestTriangulatedPoints, bestSetsOfRays);

		// Return the best solution
		return {bestTriangulatedPoints, bestSetsOfRays};
	}

	// If there are less than 2 cameras, we can't triangulate anything
	return {};
}

// -----------------------------------------------
// Implementation of public members of this module
// -----------------------------------------------

void removeSingleRays(std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	// Clear points with only one ray associated to it
	for (auto it = setsOfRays.begin(); it != setsOfRays.end();)
	{
		// If less than 2 rays are associated to this 3D point, we can't triangulate it
		if (it->size() < 2)
		{
			it = setsOfRays.erase(it);
		}
		else
		{
			++it;
		}
	}
}

void removePointsFromSingleRays(
	std::vector<glm::vec3>& points,
	std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	auto itRays = setsOfRays.begin();
	auto itPoints = points.begin();
	for (; itRays != setsOfRays.end();)
	{
		if (itRays->size() > 1)
		{
			++itRays;
			++itPoints;
		}
		else
		{
			itRays = setsOfRays.erase(itRays);
			itPoints = points.erase(itPoints);
		}
	}
}

void sortSetsOfRays(std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	for (auto& setOfRays : setsOfRays)
	{
		std::sort(setOfRays.begin(), setOfRays.end());
	}
}

bool computeDistributionOfSimilarities(
	const std::string& filename,
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	// Distances between rays of the same group
	std::vector<float> similaritiesInGroup;
	// Distances between rays of different groups
	std::vector<float> similaritiesOutOfGroup;

	// For each set of rays corresponding to a 3D point
	for (unsigned int p = 0; p < setsOfRays.size(); p++)
	{
		for (unsigned int q = 0; q < setsOfRays.size(); q++)
		{
			for (unsigned int i = 0; i < setsOfRays[p].size(); i++)
			{
				for (unsigned int j = 0; j < setsOfRays[q].size(); j++)
				{
					// Ignore the similarities between the same rays
					if (p == q && i == j)
					{
						continue;
					}

					const auto& [cameraI, rayI] = setsOfRays[p][i];
					const auto& [cameraJ, rayJ] = setsOfRays[q][j];
					const auto s = similarity(cameras[cameraI],
					                          rays[cameraI][rayI],
					                          points2D[cameraI][rayI],
					                          cameras[cameraJ],
					                          rays[cameraJ][rayJ],
					                          points2D[cameraJ][rayJ]);

					if (p == q)
					{
						// The two rays belong to the same group
						// We expect them to be similar
						similaritiesInGroup.push_back(s);
					}
					else
					{
						// The two rays belong to two different groups
						// We expect them to not be similar
						similaritiesOutOfGroup.push_back(s);
					}
				}
			}
		}
	}

	// Sort the two distributions so that it is easier to compute stats
	std::sort(similaritiesInGroup.begin(), similaritiesInGroup.end());
	std::sort(similaritiesOutOfGroup.begin(), similaritiesOutOfGroup.end());

	// Report the min, max, and median for each group
	spdlog::info("In group similarity distribution: min = {:.2f}, max = {:.2f}",
	             similaritiesInGroup.front(),
	             similaritiesInGroup.back());

	spdlog::info("Out of group similarity distribution: min = {:.2f}, max = {:.2f}",
	             similaritiesOutOfGroup.front(),
	             similaritiesOutOfGroup.back());

	// Export the two distributions
	std::ofstream file(filename, std::fstream::out);

	if (!file.is_open())
	{
		return false;
	}

	file << "In group" << std::endl;
	for (const auto& s : similaritiesInGroup)
	{
		file << s << "\n";
	}

	file << "Out of group" << std::endl;
	for (const auto& s : similaritiesOutOfGroup)
	{
		file << s << "\n";
	}

	file.close();

	return true;
}

std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulate(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	float thresholdNoPair)
{
	// TODO: run the DP bottom up in parallel for 2 cameras, then 3 cameras,
	// TODO: then delete all entries with 2 cameras and run in parallel for 4 cameras, etc...
	
	// Build the active set vector
	const std::vector<bool> activeCameraSet(cameras.size(), true);

	// Declare the cache for memoization of dynamic programming
	const auto cacheSize = static_cast<int>(pow(2.0, cameras.size()));
	std::vector<MatchRaysAndTriangulateDPCacheEntry> cacheDP(cacheSize);

	// Call the dynamic programming function to solve the problem
	return matchRaysAndTriangulateDP(cacheDP, cameras, points2D, rays, activeCameraSet, thresholdNoPair);
}
