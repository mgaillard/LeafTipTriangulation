#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "Camera.h"

/**
 * \brief Remove points defined by a single ray, since they can't be triangulated
 * \param setsOfRays A matching of rays
 */
void removeSingleRays(std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Remove points that are triangulated from only one ray
 * \param points A set of 3D points
 * \param setsOfRays A matching of rays
 */
void removePointsFromSingleRays(std::vector<glm::vec3>& points,
	                            std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Sort each set of rays
 * \param setsOfRays A matching of rays
 */
void sortSetsOfRays(std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Find a matching of rays between multiple cameras and triangulate 3D points based on the matching
 * \param cameras A list of cameras
 * \param points2D A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param thresholdNoPair Threshold in px above which two rays/points can't be paired together
 * \return Triangulated 3D points and the matching of rays that triangulates 2D points projected by cameras
 */
std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulate(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<Ray>>& rays,
	float thresholdNoPair = std::numeric_limits<float>::max());
