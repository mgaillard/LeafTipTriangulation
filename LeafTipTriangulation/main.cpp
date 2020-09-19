#include <iostream>
#include <array>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/closest_point.hpp>
#include <glm/gtx/string_cast.hpp>

#include <dlib/optimization/max_cost_assignment.h>

#include "Camera.h"
#include "Ray.h"
#include "OBJWriter.h"
#include "Triangulation.h"

/**
 * \brief Generate 3D points in a sphere centered around the origin
 * \param n The number of points to generate
 * \param radius The radius of the sphere
 * \return A vector of 3D points
 */
std::vector<glm::vec3> generatePointsInSphere(int n, float radius)
{
	std::vector<glm::vec3> points3D;

	points3D.reserve(n);
	for (int i = 0; i < n; i++)
	{
		points3D.push_back(glm::ballRand(radius));
	}
	
	return points3D;
}

/**
 * \brief Generate cameras pointing to the origin on a sphere
 * \param n Number of cameras
 * \param radius Radius of the sphere
 * \return A vector of camera
 */
std::vector<Camera> generateCamerasOnSphere(int n, float radius)
{
	std::vector<Camera> cameras;

	cameras.reserve(n);
	for (int i = 0; i < n; i++)
	{
		// Generate the location of the camera
		const auto eye = glm::sphericalRand(radius);

		// Generate the up vector (unit vector around the eye, orthogonal to atToEye)
		glm::vec3 up;
		do
		{
			up = glm::cross(glm::sphericalRand(1.f), eye);
		} while (glm::length(up) <= 0.f);
		up = glm::normalize(up);

		cameras.push_back(Camera(eye, glm::vec3(0.f, 0.f, 0.f), up));
	}
	
	return cameras;
}

/**
 * \brief Load camera from a list of files
 * \param files A list of files containing the camera information
 * \param viewportSize Size of the viewport of images in pixels
 * \return A vector of camera
 */
std::vector<Camera> loadCamerasFromFiles(
	const std::vector<std::string>& files,
	const glm::vec2& viewportSize)
{
	std::vector<Camera> cameras;

	cameras.reserve(files.size());

	for (const auto& filename : files)
	{
		std::ifstream file(filename);

		if (file.is_open())
		{
			glm::vec3 eye, at, up;
			glm::mat4 matV, matP;

			file >> eye.x >> eye.y >> eye.z;
			file >> at.x >> at.y >> at.z;
			file >> up.x >> up.y >> up.z;

			file >> matV[0][0] >> matV[1][0] >> matV[2][0] >> matV[3][0];
			file >> matV[0][1] >> matV[1][1] >> matV[2][1] >> matV[3][1];
			file >> matV[0][2] >> matV[1][2] >> matV[2][2] >> matV[3][2];
			file >> matV[0][3] >> matV[1][3] >> matV[2][3] >> matV[3][3];

			file >> matP[0][0] >> matP[1][0] >> matP[2][0] >> matP[3][0];
			file >> matP[0][1] >> matP[1][1] >> matP[2][1] >> matP[3][1];
			file >> matP[0][2] >> matP[1][2] >> matP[2][2] >> matP[3][2];
			file >> matP[0][3] >> matP[1][3] >> matP[2][3] >> matP[3][3];
			
			file.close();

			cameras.emplace_back(eye, at, up, matV, matP, viewportSize);
		}
	}

	return cameras;
}

/**
 * \brief Project 3D points on cameras. The viewport is 1000*1000 px.
 * \param points A list of 3D points
 * \param cameras A list of cameras
 * \return 2D points projected on cameras
 */
std::vector<std::vector<glm::vec2>> projectPoints(
	const std::vector<glm::vec3>& points,
	const std::vector<Camera>& cameras
)
{	
	std::vector<std::vector<glm::vec2>> projected(cameras.size());

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		for (auto point : points)
		{
			const auto point2D = cameras[c].project(point);

			// Check depth: the point must be visible from the camera and not behind it
			assert(point2D.z > 0.f);

			projected[c].emplace_back(point2D);
		}
	}
	
	return projected;
}

/**
 * \brief Add gaussian noise to 2D points
 * \param points A list of 2D points
 * \param cameras The list of cameras projecting the two points
 * \param noiseStd Standard deviation of the gaussian noise added to 2D points
 * \return The list of 2D points, with added noise
 */
std::vector<std::vector<glm::vec2>> addNoise(
	const std::vector<std::vector<glm::vec2>>& points,
	const std::vector<Camera>& cameras,
	float noiseStd)
{
	std::vector<std::vector<glm::vec2>> newPoints;

	newPoints.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
	{
		std::vector<glm::vec2> newCameraPoints;

		newCameraPoints.reserve(points[i].size());
		for (const auto& point : points[i])
		{
			// Viewport for the current camera
			const auto& view = cameras[i].viewport();
			
			// 2D point + noise
			const auto x = glm::clamp(point.x + glm::gaussRand(0.f, noiseStd), view.x, view.z);
			const auto y = glm::clamp(point.y + glm::gaussRand(0.f, noiseStd), view.y, view.w);
			
			newCameraPoints.emplace_back(x, y);
		}

		newPoints.push_back(newCameraPoints);
	}

	return newPoints;
}

/**
 * \brief Randomly remove points from the list to simulation occlusion
 * \param points A list of 2D points
 * \param probabilityKeep A probability to keep a 2D point
 * \return The list of 2D points minus some points that are removed
 */
std::vector<std::vector<glm::vec2>> removePoints(
	const std::vector<std::vector<glm::vec2>>& points,
	float probabilityKeep)
{
	assert(probabilityKeep >= 0.f && probabilityKeep <= 1.f);
	
	std::vector<std::vector<glm::vec2>> newPoints;

	newPoints.reserve(points.size());
	for (const auto& cameraPoints : points)
	{
		std::vector<glm::vec2> newCameraPoints;

		newCameraPoints.reserve(cameraPoints.size());
		for (const auto& point : cameraPoints)
		{
			if (glm::linearRand(0.f, 1.f) <= probabilityKeep)
			{
				newCameraPoints.push_back(point);
			}
		}

		newPoints.push_back(newCameraPoints);
	}

	return newPoints;
}

/**
 * \brief Compute rays going from camera to the 3D space
 * \param cameras A list of cameras
 * \param points A list of 2D points per camera
 * \return A list of rays from cameras
 */
std::vector<std::vector<Ray>> computeRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points
)
{	
	std::vector<std::vector<Ray>> rays(cameras.size());

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		for (auto point : points[c])
		{
			const glm::vec3 points3D(point.x, point.y, 1.f);

			const auto unProjected = cameras[c].unProject(points3D);

			const auto origin = cameras[c].eye();
			const auto direction = glm::normalize(unProjected - origin);

			rays[c].push_back(Ray{origin, direction});
		}
	}
	
	return rays;
}

bool checkUnProject(
	const std::vector<glm::vec3>& points,
	const std::vector<std::vector<Ray>>& rays)
{
	for (unsigned int i = 0; i < points.size(); i++)
	{
		for (unsigned int c = 0; c < rays.size(); c++)
		{
			const auto closestPoint = glm::closestPointOnLine(points[i],
				                                              rays[c][i].origin,
				                                              rays[c][i].at(10.f));

			if (glm::distance(points[i], closestPoint) >= 1e-3)
			{
				std::cout << "Warning, re-projection not accurate" << std::endl;
				return false;
			}
		}
	}

	return true;
}

bool exportSceneAsOBJ(
	const std::vector<glm::vec3>& points,
	const std::vector<std::vector<Ray>>& rays,
	const std::string& filename)
{
	OBJWriter objWriter;

	// Add 3D points
	for (const auto& point : points)
	{
		objWriter.addVertex(point);
	}

	// Add rays
	for (const auto& cameraRays : rays)
	{
		for (const auto& ray : cameraRays)
		{
			objWriter.addLine(ray.origin, ray.at(6.f));
		}
	}
	
	return objWriter.save(filename);
}

void exportSplitSceneAsOBJ(
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	const std::vector<glm::vec3>& triangulatedPoints3D)
{
	// Draw the scene in OBJ for Debugging
	exportSceneAsOBJ(triangulatedPoints3D, {}, "points.obj");
	// Draw rays from each camera separately
	for (unsigned int i = 0; i < rays.size(); i++)
	{
		exportSceneAsOBJ({}, { rays[i] }, "camera_" + std::to_string(i) + ".obj");
	}
	// Draw rays from each point separately
	for (unsigned int i = 0; i < setsOfRays.size(); i++)
	{
		std::vector<Ray> raysToDisplay;
		for (const auto& pointRays : setsOfRays[i])
		{
			// Camera
			const auto& c = pointRays.first;
			// Ray
			const auto& r = pointRays.second;

			raysToDisplay.push_back(rays[c][r]);
		}

		exportSceneAsOBJ({}, { raysToDisplay }, "rays_" + std::to_string(i) + ".obj");
	}
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

	std::array<glm::mat3, 2> A = { glm::mat3(0.f), glm::mat3(0.f) };
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

float computeMaximumCameraResolution(const Camera& camera)
{
	return std::max(
		camera.viewport().z,
		camera.viewport().w
	);
}

float computeMaximumCameraResolution(const std::vector<Camera>& cameras)
{
	float maximumResolution = 0;
	
	for (const auto& camera : cameras)
	{
		maximumResolution = std::max(
			maximumResolution,
			computeMaximumCameraResolution(camera)
		);
	}

	return maximumResolution;
}

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
			{ {referenceCamera, i} }
		);
	}

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		if (c == referenceCamera) {
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

	return setsOfRays;
}


/**
 * \brief Match each ray to a 3D point based on the re-projected error
 * \param points3D A set of points in 3D
 * \param camera A camera
 * \param points2D A set of 2D points projected on the camera
 * \param rays A set of rays associated to each 2D point projected on the camera
 * \return The assignment between the 3D points and the rays
 */
std::vector<long> pointsRaysMatching(
	const std::vector<glm::vec3>& points3D,
	const Camera& camera,
	const std::vector<glm::vec2>& points2D,
	const std::vector<Ray>& rays
	)
{
	assert(points3D.size() == points2D.size());
	assert(points2D.size() == rays.size());

	// Compute an upper bound on the maximum distance between two pixel with the camera
	const auto maximumDistancePixels = computeMaximumCameraResolution(camera) * std::sqrt(2.f);

	// Multiplier used to convert a floating point value to an integer value
	// Make sure that this distance is about 1G for the maximum distance in the image
	const double realToLongMultiplier = 1e9 / maximumDistancePixels;

	const int costSize = points3D.size();
	
	dlib::matrix<long> cost(costSize, costSize);
	for (int i = 0; i < costSize; i++)
	{
		for (int j = 0; j < costSize; j++)
		{
			// Project the 3D point i on the current camera
			const auto projectedPoint = glm::vec2(camera.project(points3D[i]));

			// Compare to the 2D point j
			const auto pointsDistance = static_cast<double>(glm::distance(projectedPoint, points2D[j]));
			// The distance is the ratio between the point distance and the longest distance in the image
			const auto dist = pointsDistance / maximumDistancePixels;

			// Make the distance integer
			const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
			cost(i, j) = -integerDist;
		}
	}

	// Compute best assignment between pairs of points
	return dlib::max_cost_assignment(cost);
}

std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> solveDP(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<bool>& activeCameraSet
	)
{
	assert(points2D.size() == cameras.size());
	assert(rays.size() == cameras.size());
	assert(activeCameraSet.size() == cameras.size());

	// ---------------------------------------------------------
	std::cout << "Solve: ";
	for (unsigned int i = 0; i < activeCameraSet.size(); i++) {
		std::cout << activeCameraSet[i];
	}
	std::cout << std::endl;
	// ---------------------------------------------------------
	
	// TODO: lookup in the cache if the solution is already computed
	// TODO: the cache is a map with the input active set converted as an integer and the output of the function
	
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

		return { triangulatedPoints3D, setsOfRays };
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
				std::tie(subPoints, setsOfRays) = solveDP(cameras, points2D, rays, subActiveCameraSet);

				// Match rays to subPoints and add the rays to the setsOfRays
				const auto raysAssignment = pointsRaysMatching(subPoints, cameras[c], points2D[c], rays[c]);

				// Add the assigned rays to the setOfRays
				for (unsigned int p = 0; p < raysAssignment.size(); p++)
				{
					setsOfRays[p].emplace_back(c, raysAssignment[p]);
				}
				
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

		// Return the best solution
		return { bestTriangulatedPoints, bestSetsOfRays };
	}
	
	// If there are less than 2 cameras, we can't triangulate anything
	return {};
}

void matchingTriangulatedPointsWithGroundTruth(
	const std::vector<glm::vec3>& points3D,
	const std::vector<glm::vec3>& triangulatedPoints3D
)
{
	assert(points3D.size() == triangulatedPoints3D.size());

	// Multiplier used to convert a floating point value to an integer value
	const float realToLongMultiplier = 1000.f;

	dlib::matrix<long> cost(points3D.size(), points3D.size());
	dlib::matrix<float> realCost(points3D.size(), points3D.size());
	
	for (int i = 0; i < points3D.size(); i++)
	{
		for (int j = 0; j < triangulatedPoints3D.size(); j++)
		{
			const auto dist = glm::distance(points3D[i], triangulatedPoints3D[j]);
			realCost(i, j) = dist;
			
			const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
			cost(i, j) = -integerDist;
		}
	}

	// Compute best assignment between pairs of points
	const auto assignment = dlib::max_cost_assignment(cost);

	// Compute statistics on the assignment
	float maximumDistance = 0.0f;
	for (unsigned int i = 0; i < assignment.size(); i++)
	{
		const auto groundTruthIndex = i;
		const auto triangulatedIndex = assignment[i];

		const auto dist = glm::distance(points3D[groundTruthIndex], triangulatedPoints3D[triangulatedIndex]);
		maximumDistance = std::max(maximumDistance, dist);
	}

	std::cout << "Maximum distance from triangulated to ground truth: " << maximumDistance << "\n"
		      << "Assignment cost between the ground truth and triangulation: "
	          << dlib::assignment_cost(realCost, assignment) << std::endl;
}

void testWithSyntheticData()
{
	const int numberPoints3D = 1;
	const float spherePointsRadius = 1.f;
	const int numberCameras = 2;
	const float sphereCamerasRadius = 5.f;

	const auto points3D = generatePointsInSphere(numberPoints3D, spherePointsRadius);
	const auto cameras = generateCamerasOnSphere(numberCameras, sphereCamerasRadius);
	const auto projectedPoints2D = projectPoints(points3D, cameras);
	// Add noise and occlusion and shuffle points
	const auto points2D = removePoints(addNoise(projectedPoints2D, cameras, 2.f), 1.0f);
	
	const auto rays = computeRays(cameras, points2D);
	checkUnProject(points3D, rays);

	// Draw the scene in OBJ for Debugging
	exportSceneAsOBJ(points3D, rays, "scene.obj");

	// Compute similarity between all points between the two cameras
	// Matching of points using the Hungarian algorithm
	const auto setsOfRays = findSetsOfRays(cameras, points2D, rays);

	// Triangulation and bundle adjustment of sets of rays
	float triangulationError;
	std::vector<glm::vec3> triangulatedPoints3D;
	std::tie(triangulationError, triangulatedPoints3D) = triangulatePoints(cameras, points2D, setsOfRays);
	std::cout << "Triangulation re-projection error: " << triangulationError << std::endl;

	// Match the two sets of points and check the distance
	matchingTriangulatedPointsWithGroundTruth(points3D, triangulatedPoints3D);
}

void runOnRealData()
{
	const float imageWidth = 2454.0;
	const float imageHeight = 2056.0;

	const auto cameras = loadCamerasFromFiles({
		"camera_0.txt",
		"camera_72.txt",
		// "camera_144.txt",
		// "camera_216.txt",
		"camera_288.txt",
		// "camera_top.txt"
		}, glm::vec2(imageWidth, imageHeight));

	// X axis is from left to right
	// Y axis is from bottom to top
	const std::vector<std::vector<glm::vec2>> points2D = {
		// Camera 0
		{
			// {1009, 560},
			// {813, 866},
			{883, 1268},
			{909, 1258},
			// {1319, 1308},
			// {1445, 1056},
			// {1473, 716}
		},
		// Camera 72
		{
			// {803, 884},
			// {877, 576},
			{931, 1260},
			{1195, 1258},
			// {1285, 1094},
			// {1481, 1356},
			// {1609, 1116},
			// {1627, 686}
		},
		// Camera 144
		/*
		{
			// {1348, 888},
			// {1228, 1112},
			// {1294, 1366},
			{1364, 1252},
			{1514, 1258}
		},
		// Camera 216
		{
			// {815, 696},
			// {841, 1112},
			// {1189, 1096},
			// {1007, 1364},
			{1449, 1280},
			{1627, 1266},
			// {1731, 880},
			// {1575, 584}
		},
		*/
		// Camera 288
		{
			// {999, 711},
			// {993, 1099},
			// {1027, 1349},
			{1051, 1281},
			{1327, 1277},
			// {1435, 865},
			// {1463, 549}
		},
		// Camera top
		/*
		{
			{776, 1357},
			{750, 1327},
			{764, 1137},
			{1142, 981},
			{1630, 693},
			{1690, 461},
			{1536, 455},
			{1118, 459}
		}
		*/
	};

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(cameras, points2D);

	// Compute similarity between all points between the two cameras
	// Matching of points using the Hungarian algorithm
	const auto setsOfRays = findSetsOfRays(cameras, points2D, rays);

	// Triangulation and bundle adjustment of sets of rays
	float triangulationError;
	std::vector<glm::vec3> triangulatedPoints3D;
	std::tie(triangulationError, triangulatedPoints3D) = triangulatePoints(cameras, points2D, setsOfRays);
	std::cout << "Triangulation re-projection error: " << triangulationError << std::endl;

	exportSplitSceneAsOBJ(rays, setsOfRays, triangulatedPoints3D);
}

void experiment()
{
	const float imageWidth = 2454.0;
	const float imageHeight = 2056.0;

	const auto cameras = loadCamerasFromFiles({
		"camera_0.txt",
		"camera_72.txt",
		"camera_144.txt",
		"camera_216.txt",
		"camera_288.txt",
		"camera_top.txt"
		}, glm::vec2(imageWidth, imageHeight));

	// X axis is from left to right
	// Y axis is from bottom to top
	const std::vector<std::vector<glm::vec2>> points2D = {
		// Camera 0
		{
			{883, 1268},
			{909, 1258}
		},
		// Camera 72
		{
			{931, 1260},
			{1195, 1258}
		},
		// Camera 144
		{
			{1364, 1252},
			{1514, 1258}
		},
		// Camera 216
		{
			{1449, 1280},
			{1627, 1266}
		},
		// Camera 288
		{
			{1051, 1281},
			{1327, 1277}
		},
		// Camera top
		{
			{697, 1171},
			{671, 683}
		}
	};

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(cameras, points2D);
	
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	std::tie(triangulatedPoints3D, setsOfRays) = solveDP(cameras, points2D, rays, { true, true, true, true, true, true });

	exportSceneAsOBJ(triangulatedPoints3D, rays, "scene.obj");
}

int main(int argc, char *argv[])
{
	experiment();
	
    return 0;
}
