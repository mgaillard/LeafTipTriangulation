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
 * \return A vector of camera as glm::mat4
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
	
	// TODO: Support more than 2 views
	
	dlib::matrix<long> cost(rays[0].size(), rays[1].size());
	for (int i = 0; i < rays[0].size(); i++)
	{
		std::vector<float> row(rays[1].size(), 0.f);
		for (int j = 0; j < rays[1].size(); j++)
		{
			const auto dist = similarity(cameras[0],
				                         rays[0][i],
				                         points2D[0][i],
				                         cameras[1],
				                         rays[1][j],
				                         points2D[1][j]);

			const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
			cost(i, j) = -integerDist;
		}
	}

	// Compute best assignment between pairs of points
	const auto assignment = dlib::max_cost_assignment(cost);

	// A list of 3D points defined by a list of index of rays
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	
	for (unsigned int i = 0; i < assignment.size(); i++)
	{
		// Add a 3D point defined by the intersection of 2 rays
		// Ray i from camera 0
		// Ray assignment[i] from camera 1
		setsOfRays.push_back(
			{{0, i}, {1, assignment[i]}}
		);
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

int main(int argc, char *argv[])
{
	const int numberPoints3D = 1;
	const float spherePointsRadius = 1.f;
	const int numberCameras = 2;
	const float sphereCamerasRadius = 5.f;

	const auto points3D = generatePointsInSphere(numberPoints3D, spherePointsRadius);
	const auto cameras = generateCamerasOnSphere(numberCameras, sphereCamerasRadius);
	const auto points2D = projectPoints(points3D, cameras);
	// TODO: Add noise and occlusion and shuffle points 
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
	
    return 0;
}
