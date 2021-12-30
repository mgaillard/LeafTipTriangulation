#pragma once

#include <utils/warnoff.h>
#include <glm/glm.hpp>

#include <opencv2/core/core.hpp>
#include <utils/warnon.h>

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
