#pragma once

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

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
 * \brief Convert a 3D glm vector to OpenCV vector
 * \param v A glm::vec3 vector
 * \return An OpenCV vector
 */
cv::Vec3f convertToOpenCV(const glm::vec3& v);

/**
 * \brief Convert a 3D OpenCV vector to glm vector
 * \param v A cv::Vec3f vector
 * \return A glm vector
 */
glm::vec3 convertToGlm(const cv::Vec3f& v);

/**
 * \brief Convert a 3D OpenCV vector to glm vector
 * \param v A cv::Vec3d vector
 * \return A glm vector
 */
glm::vec3 convertToGlm(const cv::Vec3d& v);

/**
 * \brief Convert a 4D OpenCV vector to glm vector
 * \param v A cv::Vec4f vector
 * \return A glm vector
 */
glm::vec4 convertToGlm(const cv::Vec4f& v);

/**
 * \brief Convert a 3-by-4 OpenCV matrix to glm matrix
 * \param v A cv::Mat1f matrix of size (3, 4)
 * \return A glm matrix
 */
glm::mat4x3 convertToGlm(const cv::Mat1f& v);

/**
 * \brief Project a 3D point in homogeneous coordinates to 2D.
 * \param H A 3x4 projection matrix.
 * \param m A 3D point in homogeneous coordinates
 * \return A 2D point
 */
cv::Vec2f projectPoint(const cv::Mat1f& H, const cv::Vec4f& m);

/**
 * \brief Project a 3D point in homogeneous coordinates to 2D.
 * \param H A 3x4 projection matrix.
 * \param m A 3D point in homogeneous coordinates
 * \return A 2D point
 */
glm::vec2 projectPoint(const glm::mat4x3& H, const glm::vec4& m);

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
	const cv::Vec3f& point3d
);

std::tuple<float, std::vector<glm::vec3>> triangulatePoints(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::vec2>>& points2D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays);
