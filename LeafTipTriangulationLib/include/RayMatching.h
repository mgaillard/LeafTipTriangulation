#pragma once

#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Camera.h"
#include "Types.h"

/**
 * \brief Remove points defined by a single ray, since they can't be triangulated
 * \param setsOfCorrespondences A matching of rays
 */
void removeSingleRays(SetsOfCorrespondences& setsOfCorrespondences);

/**
 * \brief Remove points that are triangulated from only one ray
 * \param points A set of 3D points
 * \param setsOfCorrespondences A matching of rays
 */
void removePointsFromSingleRays(SetOfVec3& points,
                                SetsOfCorrespondences& setsOfCorrespondences);

/**
 * \brief Sort each set of rays
 * \param setsOfCorrespondences A matching of rays
 */
void sortSetsOfCorrespondences(SetsOfCorrespondences& setsOfCorrespondences);

/**
 * \brief Compute two distribution of similarities from a matching.
 *        1) the distribution of similarities between a ray and all rays that are matched with it
 *		  2) the distribution of similarities between a ray and all rays that are NOT matched with it
 * \param filename The path to the file in which to save the similarities
 * \param cameras A list of cameras
 * \param points2d A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param setsOfCorrespondences The matching of rays
 * \return True if the similarities could be saved in the file
 */
bool computeDistributionOfSimilarities(
    const std::string& filename,
    const std::vector<Camera>& cameras,
    const SetsOfVec2& points2d,
    const SetsOfRays& rays,
    const SetsOfCorrespondences& setsOfCorrespondences);

/**
 * \brief Find a matching of rays between multiple cameras and triangulate 3D points based on the matching
 * \param cameras A list of cameras
 * \param points2d A list of 2D points per camera
 * \param rays A list of 3D rays associated to 2D points per camera
 * \param thresholdNoPair Threshold in px above which two rays/points can't be paired together
 * \return Triangulated 3D points, the matching of rays that triangulates 2D points projected by cameras, and the view order
 */
std::tuple<SetOfVec3, SetsOfCorrespondences, std::vector<int>> matchRaysAndTriangulate(
    const std::vector<Camera>& cameras,
    const SetsOfVec2& points2d,
    const SetsOfRays& rays,
    double thresholdNoPair = std::numeric_limits<double>::max());
