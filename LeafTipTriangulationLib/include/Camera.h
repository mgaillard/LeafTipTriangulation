#pragma once

#include <string>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Ray.h"

class Camera
{
public:
	Camera(const glm::vec3& eye, const glm::vec3& at, const glm::vec3& up);

	Camera(const glm::vec3& eye,
		   const glm::vec3& at,
		   const glm::vec3& up,
		   float fovy,
		   float aspectRatio,
		   const glm::vec2& viewportSize);

	Camera(const glm::vec3& eye,
		   const glm::vec3& at,
		   const glm::vec3& up,
		   const glm::mat4& matV,
		   const glm::mat4& matP,
		   const glm::vec2& viewportSize);

	const glm::vec3& eye() const { return m_eye; }
	const glm::vec3& at() const { return m_at; }
	const glm::vec3& up() const { return m_up; }

	const glm::mat4& matV() const { return m_matV; }
	const glm::mat4& matP() const { return m_matP; }
	const glm::mat4& mat() const { return m_mat; }

	const glm::vec4& viewport() const { return m_viewport; }

	glm::vec3 project(const glm::vec3& point) const;

	glm::vec3 unProject(const glm::vec3& point) const;

	glm::vec2 windowToViewport(const glm::vec2& point) const;

private:
	glm::vec3 m_eye;
	glm::vec3 m_at;
	glm::vec3 m_up;

	glm::mat4 m_matV;
	glm::mat4 m_matP;
	glm::mat4 m_mat;

	glm::vec4 m_viewport;
};

/**
 * \brief Return the maximum resolution on X axis or Y axis of a camera
 * \param camera A camera
 * \return The maximum resolution of the camera 
 */
float computeMaximumCameraResolution(const Camera& camera);

/**
 * \brief Return the maximum resolution of a list of cameras
 * \param cameras A list of cameras
 * \return The maximum resolution of the list of cameras
 */
float computeMaximumCameraResolution(const std::vector<Camera>& cameras);

/**
 * \brief Load camera from a list of files
 * \param files A list of files containing the camera information
 * \param viewportSize Size of the viewport of images in pixels
 * \return A vector of camera
 */
std::vector<Camera> loadCamerasFromFiles(const std::vector<std::string>& files,
								         const glm::vec2& viewportSize);

/**
 * \brief Compute rays going from camera to the 3D space
 * \param cameras A list of cameras
 * \param points A list of 2D points per camera
 * \return A list of rays from cameras
 */
std::vector<std::vector<Ray>> computeRays(const std::vector<Camera>& cameras,
							              const std::vector<std::vector<glm::vec2>>& points);
