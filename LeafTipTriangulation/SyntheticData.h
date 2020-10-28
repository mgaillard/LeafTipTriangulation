#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "Camera.h"
#include "Ray.h"

/**
 * \brief Generate 3D points in a sphere centered around the origin
 * \param n The number of points to generate
 * \param radius The radius of the sphere
 * \return A vector of 3D points
 */
std::vector<glm::vec3> generatePointsInSphere(int n, float radius);

/**
 * \brief Generate cameras pointing to the origin on a sphere
 * \param n Number of cameras
 * \param radius Radius of the sphere
 * \return A vector of camera
 */
std::vector<Camera> generateCamerasOnSphere(int n, float radius);

/**
 * \brief Project 3D points on cameras. The viewport is 1000*1000 px.
 * \param points A list of 3D points
 * \param cameras A list of cameras
 * \return 2D points projected on cameras
 */
std::vector<std::vector<glm::vec2>> projectPoints(const std::vector<glm::vec3>& points,
												  const std::vector<Camera>& cameras);

/**
 * \brief Add gaussian noise to 2D points
 * \param points A list of 2D points
 * \param cameras The list of cameras projecting the two points
 * \param noiseStd Standard deviation of the gaussian noise added to 2D points
 * \return The list of 2D points, with added noise
 */
std::vector<std::vector<glm::vec2>> addNoise(const std::vector<std::vector<glm::vec2>>& points,
	                                         const std::vector<Camera>& cameras,
	                                         float noiseStd);

/**
 * \brief Randomly remove points from the list to simulation occlusion
 * \param points A list of 2D points
 * \param probabilityKeep A probability to keep a 2D point
 * \return The list of 2D points minus some points that are removed
 */
std::vector<std::vector<glm::vec2>> removePoints(const std::vector<std::vector<glm::vec2>>& points,
											     float probabilityKeep);

/**
 * \brief Check that each 3D point is close to a it's associated ray.
 *        Use this function to test the output of the function computeRays() when ground truth is available.
 * \param points A list of 3D points
 * \param rays A list of Rays associated to 3D points
 * \return True if all 3D points are within 1e-3 of all their rays, false otherwise
 */
bool checkUnProject(const std::vector<glm::vec3>& points,
	                const std::vector<std::vector<Ray>>& rays);

/**
 * \brief Return the cost of matching triangulated 3D points to ground truth 3D points.
 *        Display the maximum distance between a 3D point and it's matching triangulated 3D point.
 *        Display the average distance between the ground truth and the triangulation.
 *        Use this function to check that the triangulation without correspondences was successful.
 * \param points3D A list of 3D points
 * \param triangulatedPoints3D A list of triangulated 3D points
 */
void matchingTriangulatedPointsWithGroundTruth(const std::vector<glm::vec3>& points3D,
	                                           const std::vector<glm::vec3>& triangulatedPoints3D);

/**
 * \brief Check the correspondence of rays to make sure the matching is perfect
 * \param setsOfRays The matching of rays that best triangulates 2D points projected by cameras
 */
void checkCorrespondenceSetsOfRays(const std::vector<std::vector<std::pair<int, int>>>& setsOfRays);
