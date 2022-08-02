#pragma once

#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

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
void removePointsFromSingleRays(std::vector<glm::dvec3>& points,
	                            std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Sort each set of rays
 * \param setsOfRays A matching of rays
 */
void sortSetsOfRays(std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Compute two distribution of similarities from a matching.
 *        1) the distribution of similarities between a ray and all rays that are matched with it
 *		  2) the distribution of similarities between a ray and all rays that are NOT matched with it
 * \param filename The path to the file in which to save the similarities
 * \param cameras A list of cameras
 * \param points2d A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param setsOfRays The matching of rays
 * \return True if the similarities could be saved in the file
 */
bool computeDistributionOfSimilarities(
	const std::string& filename,
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points2d,
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays);

/**
 * \brief Find a matching of rays between multiple cameras and triangulate 3D points based on the matching
 * \param cameras A list of cameras
 * \param points2d A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param thresholdNoPair Threshold in px above which two rays/points can't be paired together
 * \return Triangulated 3D points and the matching of rays that triangulates 2D points projected by cameras
 */
std::tuple<std::vector<glm::dvec3>, std::vector<std::vector<std::pair<int, int>>>> matchRaysAndTriangulate(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points2d,
	const std::vector<std::vector<Ray>>& rays,
	double thresholdNoPair = std::numeric_limits<double>::max());
