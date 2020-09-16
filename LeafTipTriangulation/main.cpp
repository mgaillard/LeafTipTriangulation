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

std::vector<std::vector<std::pair<int, int>>> findSetsOfRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays)
{
	// Multiplier used to convert a floating point value to an integer value
	const float realToLongMultiplier = 1000.f;

	// A list of 3D points defined by a list of index of rays
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	
	// TODO: Find the view with the maximum number of points in 2D
	const unsigned int referenceCamera = 0;

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

		assert(rays[referenceCamera].size() == rays[c].size());

		// Compare the referenceCamera to this camera
		dlib::matrix<long> cost(rays[referenceCamera].size(), rays[c].size());
		for (int i = 0; i < rays[referenceCamera].size(); i++)
		{
			std::vector<float> row(rays[c].size(), 0.f);
			for (int j = 0; j < rays[c].size(); j++)
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
		}

		// Compute best assignment between pairs of points
		// TODO: Handle the case with non square cost matrix
		const auto assignment = dlib::max_cost_assignment(cost);

		for (unsigned int i = 0; i < assignment.size(); i++)
		{
			// Ray i from the reference camera is in setsOfRays[i]
			// We add the corresponding ray assignment[i] from camera c
			setsOfRays[i].emplace_back(c, assignment[i]);
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
	const auto triangulatedPoints3D = triangulatePoints(cameras, points2D, setsOfRays);

	// Match the two sets of points and check the distance
	matchingTriangulatedPointsWithGroundTruth(points3D, triangulatedPoints3D);
}

int main(int argc, char *argv[])
{
	const float imageWidth = 2454.0;
	const float imageHeight = 2056.0;
	
	const auto cameras = loadCamerasFromFiles({
		"camera_0.txt",
		"camera_72.txt",
		// "camera_144.txt",
		"camera_216.txt",
		// "camera_288.txt",
		// "camera_top.txt"
	}, glm::vec2(imageWidth, imageHeight));

	// X axis is from left to right
	// Y axis is from bottom to top
	const std::vector<std::vector<glm::vec2>> points2D = {
		// Camera 0
		{
			{1095.0, (imageHeight - 1.0) - 1499.0},
			{1452.0, (imageHeight - 1.0) - 1352.0}
		},
		// Camera 72
		{
			{855.0, (imageHeight - 1.0) - 1479.0},
			{1639.0, (imageHeight - 1.0) - 1373.0},
		},
		// Camera 216
		{
			{1579.0, (imageHeight - 1.0) - 1471.0},
			{819.0, (imageHeight - 1.0) - 1362.0},
		}
	};

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(cameras, points2D);

	// Compute similarity between all points between the two cameras
	// Matching of points using the Hungarian algorithm
	const auto setsOfRays = findSetsOfRays(cameras, points2D, rays);

	// Triangulation and bundle adjustment of sets of rays
	const auto triangulatedPoints3D = triangulatePoints(cameras, points2D, setsOfRays);

	// Draw the scene in OBJ for Debugging
	exportSceneAsOBJ(triangulatedPoints3D, rays, "scene.obj");
	
    return 0;
}
