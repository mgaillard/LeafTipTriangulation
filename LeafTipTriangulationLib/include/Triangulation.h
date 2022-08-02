#pragma once

#include <utils/warnoff.h>
#include <glm/glm.hpp>

#include <opencv2/core/core.hpp>
#include <utils/warnon.h>

#include "Camera.h"

/**
 * \brief Project a 3D point in homogeneous coordinates to 2D.
 * \param projectionMatrix A 3x4 projection matrix.
 * \param point A 3D point in homogeneous coordinates
 * \return A 2D point
 */
glm::dvec2 projectPoint(const glm::dmat4x3& projectionMatrix, const glm::dvec4& point);

/**
 * \brief Project a 3D point in homogeneous coordinates to 2D.
 * \param projectionMatrix A 4x4 projection matrix.
 * \param point A 3D point in homogeneous coordinates
 * \return A 2D point
 */
glm::dvec2 projectPoint(const glm::dmat4& projectionMatrix, const glm::dvec4& point);

/**
 * \brief Compute the reprojection error when triangulating a 3D point from multiple views
 * \param projectionMatrices The projection matrices of all views, projecting 3D points to 2D points.
 * \param points2d The 2D points projected from all views
 * \param point3d The 3D point that is supposed to be a triangulation of the 2D points
 * \return The total reprojection error from all views
 */
double reprojectionErrorFromMultipleViews(const std::vector<glm::dmat4>& projectionMatrices,
                                          const std::vector<glm::dvec2>& points2d,
                                          const glm::dvec3& point3d);

/**
 * \brief Triangulate a point in 3D from multiple views
 * \param projectionMatrices The projection matrices of all views, projecting 3D points to 2D points.
 *                           Instead of 3x4 projection matrices, the input is the full 4x4 projection matrix.
 *							 The algorithm simply ignore the third row, which corresponds to the Z coordinate.
 * \param points2d The 2D points projected from all views
 * \return The triangulated 3D point and its reprojection error
 */
std::tuple<double, glm::dvec3> triangulatePointFromMultipleViews(
	const std::vector<glm::dmat4>& projectionMatrices,
	const std::vector<glm::dvec2>& points2d
);

/**
 * \brief Compute the reprojection error when triangulating a 3D point from multiple views
 * \param cameras The list of cameras on which the point is projected
 * \param points2d The 2D points projected from all views
 * \param setOfRays Correspondences of the point in the 2D views
 * \param point3d The 3D point that is supposed to be a triangulation of the 2D points
 * \return The total reprojection error from all views
 */
double reprojectionErrorFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points2d,
	const std::vector<std::pair<int, int>>& setOfRays,
	const glm::dvec3& point3d
);

/**
 * \brief Triangulate a point in 3D from multiple views
 * \param cameras The list of cameras on which the point is projected
 * \param points2d The 2D points projected from all views
 * \param setOfRays Correspondences of the point in the 2D views
 * \return The reprojection error and the 3D triangulated points
 */
std::tuple<double, glm::dvec3> triangulatePointFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points2d,
	const std::vector<std::pair<int, int>>& setOfRays
);

/**
 * \brief Compute the total reprojection error for many points seen from multiple views.
 * \param cameras The list of cameras on which points are projected
 * \param points2d The 2D points projected from all views
 * \param setsOfRays Correspondences of the points in the 2D views
 * \param points3d The coordinates of the 3D points
 * \return The total reprojection error
 */
double reprojectionErrorManyPointsFromMultipleViews(const std::vector<Camera>& cameras,
                                                    const std::vector<std::vector<glm::dvec2>>& points2d,
                                                    const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
                                                    const std::vector<glm::dvec3>& points3d);

/**
 * \brief Triangulate many points in 3D from multiple 2D views
 * \param cameras The list of cameras on which points are projected
 * \param points2d The 2D points projected from all views
 * \param setsOfRays Correspondences of the points in the 2D views
 * \return The total reprojection error and all 3D triangulated points
 */
std::tuple<double, std::vector<glm::dvec3>> triangulateManyPointsFromMultipleViews(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points2d,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays
);
