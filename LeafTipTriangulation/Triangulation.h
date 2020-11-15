#pragma once

#include <glm/glm.hpp>

#include <opencv2/core/core.hpp>

#include "Camera.h"

/**
 * \brief Return the input matrix without the third row (Z axis)
 * \param m The input 4x4 matrix
 * \return A 3x4 matrix
 */
glm::mat4x3 removeZRow(const glm::mat4& m);


/**
 * \brief Convert a 3x4 glm matrix to OpenCV matrix
 * \param m A glm::mat4x3 matrix
 * \return An OpenCV matrix
 */
cv::Mat1f convertToOpenCV(const glm::mat4x3& m);

/**
 * \brief Convert a 2D glm vector to OpenCV vector
 * \param v A glm::vec2 vector
 * \return An OpenCV vector
 */
cv::Vec2f convertToOpenCV(const glm::vec2& v);

/**
 * \brief Convert a 3D OpenCV vector to glm vector
 * \param v A cv::Vec3f vector
 * \return A glm vector
 */
glm::vec3 convertToGlm(const cv::Vec3f& v);

/**
 * \brief Project a 3D point in homogeneous coordinates to 2D.
 * \param H A 3x4 projection matrix.
 * \param m A 3D point in homogeneous coordinates
 * \return A 2D point
 */
cv::Vec2f projectPoint(const cv::Mat1f& H, const cv::Vec4f& m);

/**
 * \brief Triangulate a point in 3D 
 * \param homographies The homography matrices, projecting 3D points to 2D points
 * \param points The 2D points
 * \return The triangulated 3D point and its reprojection error 
 */
std::tuple<float, cv::Vec3f> reconstructPointFromViews(
	const std::vector<cv::Mat1f>& homographies,
	const std::vector<cv::Vec2f>& points
);

float reprojectionAdjustment(
	const std::vector<cv::Mat1f>& homographies,
	const std::vector<cv::Vec2f>& points,
	cv::Vec3f& pointToAdjust
);

float reprojectionError(
	const std::vector<cv::Mat1f>& homographies,
	const std::vector<cv::Vec2f>& points,
	cv::Vec3f& pointToAdjust
);

std::tuple<float, std::vector<glm::vec3>> triangulatePoints(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays);
