#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "Camera.h"

/**
 * \brief Find a matching of rays between multiple cameras.
 *		  Warning: This function is an approximation and does not work in the general case.
 * \param cameras A list of cameras
 * \param points2D A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \return The matching of rays that best triangulates 2D points projected by cameras
 */
std::vector<std::vector<std::pair<int, int>>> findSetsOfRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays);

/**
 * \brief Find a matching of rays between multiple cameras and triangulate 3D points based on the matching
 * \param cameras A list of cameras
 * \param points2D A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \return Triangulated 3D points and the matching of rays that triangulates 2D points projected by cameras
 */
std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulate(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays);
