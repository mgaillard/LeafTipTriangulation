#include "RayMatching.h"

#include <numeric>

#include <glm/gtc/matrix_transform.hpp>

#include <dlib/optimization/max_cost_assignment.h>

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
 * \brief Compute a similarity coefficient between two points projected on two cameras
 * \param camera0 Camera A
 * \param ray0 Ray A
 * \param point0 2D Point A
 * \param camera1 Camera B
 * \param ray1 Ray B
 * \param point1 2D Point B
 * \return The similarity coefficient between two rays
 */
float similarity(const Camera& camera0,
                 const Ray& ray0,
                 const glm::vec2& point0,
                 const Camera& camera1,
                 const Ray& ray1,
                 const glm::vec2& point1);

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
 * \brief Match each ray to a 3D point based on the re-projected error
 * \param cameras A list of cameras
 * \param points2D A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param points3D A set of points in 3D
 * \param setsOfRays The matching of rays used to triangulate the 3D points
 * \param cameraIndex The index of the camera for which to do the matching
 * \return The new set of rays taking in account the assignment between the 3D points and the rays
 */
std::vector<std::vector<std::pair<int, int>>> pointsRaysMatching(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<glm::vec3>& points3D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	int cameraIndex);


/**
 * \brief Dynamic programming implementation of the function matchRaysAndTriangulate
 *        Recursively divide the problem into sub problems
 *        A cache structure is used for memoization
 * \param cacheDP Cache to store the results of this function for reusing them later
 * \param cameras The list of cameras
 * \param points2D List of 2D points per camera
 * \param rays List of rays associated to 2D points per camera
 * \param activeCameraSet The active set of cameras as a vector of bool: true, camera activated false camera deactivated
 * \return The triangulated points and matching of rays for the current active camera set
 */
std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulateDP(
	std::vector<MatchRaysAndTriangulateDPCacheEntry>& cacheDP,
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<bool>& activeCameraSet);

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
	const glm::vec2& point1)
{
	// See: Triangulation without correspondences from Cheng et al.
	const std::array<glm::vec3, 2> u = {{ray0.direction, ray1.direction}};

	// Failure case 1: ray0 and ray1 are parallel
	// Epsilon is scaled to the magnitude of u0 + u1, which is 2.0f
	if (glm::length(glm::cross(u[0], u[1])) <= 2.f * glm::epsilon<float>())
	{
		// We can't solve the equation system
		return std::numeric_limits<float>::max();
	}

	std::array<glm::mat3, 2> A = {glm::mat3(0.f), glm::mat3(0.f)};
	for (unsigned int i = 0; i < A.size(); i++)
	{
		// glm::mat3 is column major
		auto& Ai = A[i];
		const auto& ui = u[i];

		// First row
		Ai[0][0] = (ui.y * ui.y) + (ui.z * ui.z);
		Ai[1][0] = -ui.x * ui.y;
		Ai[2][0] = -ui.x * ui.z;

		// Second row
		Ai[0][1] = -ui.x * ui.y;
		Ai[1][1] = (ui.x * ui.x) + (ui.z * ui.z);
		Ai[2][1] = -ui.y * ui.z;

		// Third row
		Ai[0][2] = -ui.x * ui.z;
		Ai[1][2] = -ui.y * ui.z;
		Ai[2][2] = (ui.x * ui.x) + (ui.y * ui.y);
	}

	// Closest point between the two lines
	const auto q = glm::inverse(A[0] + A[1]) * ((A[0] * ray0.origin) + (A[1] * ray1.origin));

	// Failure case 2: the pseudo-intersection is on the negative side of one of the rays
	const std::array<glm::vec3, 2> dirQ = {
		glm::normalize(q - ray0.origin),
		glm::normalize(q - ray1.origin)
	};
	if ((glm::dot(u[0], dirQ[0]) <= 0.f) || (glm::dot(u[1], dirQ[1]) <= 0.f))
	{
		return std::numeric_limits<float>::max();
	}

	// Compute the re-projection of q on the two views
	const auto q0 = camera0.project(q);
	const auto q1 = camera1.project(q);

	// Check depth: the point must be visible from the camera and not behind it
	assert(q0.z > 0.f);
	assert(q1.z > 0.f);

	// Compute re-projection distance
	return glm::distance(point0, glm::vec2(q0)) + glm::distance(point1, glm::vec2(q1));
}

/**
 * \brief Match each ray to a 3D point based on the re-projected error
 * \param points3D A set of points in 3D
 * \param camera A camera
 * \param points2D A set of 2D points projected on the camera
 * \param rays A set of rays associated to each 2D point projected on the camera
 * \return The assignment between the 3D points and the rays
 */
std::vector<std::vector<std::pair<int, int>>> pointsRaysMatching(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<glm::vec3>& points3D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	int cameraIndex)
{
	// Check that we receive coherent sets 
	assert(points3D.size() == setsOfRays.size());
	
	// The camera index is not out of bounds
	assert(cameraIndex >= 0 && cameraIndex < cameras.size());
	
	const auto& currentCamera = cameras[cameraIndex];
	const auto& currentPoints2D = points2D[cameraIndex];
	const auto& currentRays = rays[cameraIndex];

	// The number of 3D points and single rays that we can match to the current camera
	// is greater than the number of 2D points projected on the camera
	assert(setsOfRays.size() >= currentPoints2D.size());
	assert(currentPoints2D.size() == currentRays.size());

	// Compute an upper bound on the maximum distance between two pixel with the camera
	const auto maximumDistancePixels = computeMaximumCameraResolution(currentCamera) * std::sqrt(2.f);
	// Make sure that this distance is about 1G for the maximum distance in the image
	const long maximumSimilarity = 1e9;

	// Multiplier used to convert a floating point value to an integer value
	const double realToLongMultiplier = static_cast<double>(maximumSimilarity) / maximumDistancePixels;

	// Number of 3D points and single rays available for matching
	const int costSize = setsOfRays.size();
	// Number of 2D points to match 
	const int costCols = currentPoints2D.size();

	dlib::matrix<long> cost(costSize, costSize);
	for (int i = 0; i < costSize; i++)
	{
		for (int j = 0; j < costCols; j++)
		{
			// Project the 3D point i (or single ray) on 2D point j from the current camera
			// By default the distance is maximum to prevent the matching
			double dist = maximumDistancePixels;
			
			// Check if point i is a 3D point or a single ray
			if (setsOfRays[i].size() > 1)
			{
				// It's a 3D point
				// Project the 3D point i on the current camera
				const auto projectedPoint = glm::vec2(currentCamera.project(points3D[i]));

				// Compare to the 2D point j
				dist = static_cast<double>(glm::distance(projectedPoint, currentPoints2D[j]));
			}
			else
			{
				// It's a single ray
				const auto& singleRay = setsOfRays[i].front();
				const auto singleRayCameraIndex = singleRay.first;
				const auto singleRayIndex = singleRay.second;

				// TODO: Warning similarity computes the sum of the reprojection error on the two views
				dist = similarity(cameras[singleRayCameraIndex],
				                  rays[singleRayCameraIndex][singleRayIndex],
				                  points2D[singleRayCameraIndex][singleRayIndex],
				                  currentCamera,
				                  currentRays[j],
				                  currentPoints2D[j]);

				// TODO: Improve this, see how similarity behaves
				dist /= 2.0;
			}

			// The distance is the ratio between the point distance and the longest distance in the image
			dist /= maximumDistancePixels;

			// Make the distance integer
			const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
			cost(i, j) = -integerDist;
		}

		// Add dummy columns, filled with min - 1
		for (int j = costCols; j < costSize; j++)
		{
			cost(i, j) = -maximumSimilarity;
		}
	}

	std::cout << cost << std::endl;

	// Compute best assignment between pairs of points
	const auto assignment = dlib::max_cost_assignment(cost);

	// Copy the set of rays and update it with 
	auto newSetsOfRays = setsOfRays;

	for (unsigned int i = 0; i < assignment.size(); i++)
	{
		// If the assigned ray is not one of the dummy ray we added
		if (assignment[i] < costCols)
		{
			// Ray i from the reference camera is in setsOfRays[i]
			// We add the corresponding ray assignment[i] from camera c
			newSetsOfRays[i].emplace_back(cameraIndex, assignment[i]);
		}
	}

	return newSetsOfRays;
}

std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulateDP(
	std::vector<MatchRaysAndTriangulateDPCacheEntry>& cacheDP,
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<bool>& activeCameraSet)
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
		auto setsOfRays = findSetsOfRays(activateCameras, activatePoints2D, activeRays);

		// Change the index of cameras in the setOfRays array to match the true index of cameras
		for (auto& setOfRays : setsOfRays)
		{
			for (auto& ray : setOfRays)
			{
				// Change the camera index
				ray.first = indexActiveCamera[ray.first];
			}
		}

		float triangulationError;
		std::vector<glm::vec3> triangulatedPoints3D;
		std::tie(triangulationError, triangulatedPoints3D) = triangulatePoints(cameras, points2D, setsOfRays);

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
				std::tie(subPoints, setsOfRays) = matchRaysAndTriangulateDP(
					cacheDP, cameras, points2D, rays, subActiveCameraSet);

				// If the number of points and rays identified in the sub problem
				// are less that the number of points in this camera, cancel the reconstruction
				if (rays[c].size() > setsOfRays.size())
				{
					// TODO: Try to add the ray alone in the pointsRaysMatching function
					continue;
				}

				// Match rays to subPoints and update setsOfRays with the rays in camera c
				setsOfRays = pointsRaysMatching(cameras, points2D, rays, subPoints, setsOfRays, c);				

				// Re-triangulate the points with the new rays
				// TODO: maybe just bundle adjusting is necessary
				float triangulationError;
				std::vector<glm::vec3> triangulatedPoints;
				std::tie(triangulationError, triangulatedPoints) = triangulatePoints(cameras, points2D, setsOfRays);

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

std::vector<std::vector<std::pair<int, int>>> findSetsOfRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays)
{
	// Multiplier used to convert a floating point value to an integer value
	const float realToLongMultiplier = 1000.f;

	// Compute the maximum distance between two pixel in any camera
	const float maximumDistancePixels = computeMaximumCameraResolution(cameras) * std::sqrt(2.f);
	const long maximumSimilarity = static_cast<long>(std::round(maximumDistancePixels * realToLongMultiplier));

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
			{{referenceCamera, i}}
		);
	}

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		if (c == referenceCamera)
		{
			continue;
		}

		const int costSize = rays[referenceCamera].size();
		const int costCols = rays[c].size();

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

				const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
				cost(i, j) = -integerDist;
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
			// If the assigned ray is not one of the dummy ray we added
			if (assignment[i] < costCols)
			{
				// Ray i from the reference camera is in setsOfRays[i]
				// We add the corresponding ray assignment[i] from camera c
				setsOfRays[i].emplace_back(c, assignment[i]);
			}
		}
	}

	
	return setsOfRays;
}

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

std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulate(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays)
{
	// Build the active set vector
	const std::vector<bool> activeCameraSet(cameras.size(), true);

	// Declare the cache for memoization of dynamic programming
	const auto cacheSize = static_cast<int>(pow(2.0, cameras.size()));
	std::vector<MatchRaysAndTriangulateDPCacheEntry> cacheDP(cacheSize);

	// Call the dynamic programming function to solve the problem
	return matchRaysAndTriangulateDP(cacheDP, cameras, points2D, rays, activeCameraSet);
}
