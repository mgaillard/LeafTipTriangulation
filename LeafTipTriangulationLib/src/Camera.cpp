#include "Camera.h"

#include <fstream>
#include <iostream>

#include <utils/warnoff.h>
#include <glm/gtc/matrix_transform.hpp>
#include <utils/warnon.h>

Camera::Camera(const glm::dvec3& eye, const glm::dvec3& at, const glm::dvec3& up):
	m_eye(eye),
	m_at(at),
	m_up(up),
	m_matV(glm::lookAt(eye, at, up)), // Generate the view matrix
	m_matP(glm::perspective(45.0, 1.0, 0.1, 10.0)), // Generate the projection matrix
	m_mat(m_matP * m_matV),
	m_viewport(0.0, 0.0, 1000.0, 1000.0)
{
	
}

Camera::Camera(const glm::dvec3& eye,
               const glm::dvec3& at,
               const glm::dvec3& up,
               double fovy,
               double aspectRatio,
               const glm::dvec2& viewportSize) :
	m_eye(eye),
	m_at(at),
	m_up(up),
	m_matV(glm::lookAt(eye, at, up)), // Generate the view matrix
	m_matP(glm::perspective(fovy, aspectRatio, 0.001, 10.0)), // Generate the projection matrix
	m_mat(m_matP * m_matV),
	m_viewport(0.0, 0.0, viewportSize.x, viewportSize.y)
{
	
}

Camera::Camera(const glm::dvec3& eye,
               const glm::dvec3& at,
               const glm::dvec3& up,
               const glm::dmat4& matV,
               const glm::dmat4& matP,
               const glm::dvec2& viewportSize) :
	m_eye(eye),
	m_at(at),
	m_up(up),
	m_matV(matV),
	m_matP(matP),
	m_mat(m_matP * m_matV),
	m_viewport(0.0, 0.0, viewportSize.x, viewportSize.y)
{
}

glm::dvec3 Camera::project(const glm::dvec3& point) const
{
	return glm::projectNO(point,
	                      glm::identity<glm::dmat4>(),
	                      m_mat,
	                      m_viewport);
}

glm::dvec3 Camera::unProject(const glm::dvec3& point) const
{
	return glm::unProjectNO(point,
	                        glm::identity<glm::dmat4>(),
	                        m_mat,
	                        m_viewport);
}

glm::dvec2 Camera::windowToViewport(const glm::dvec2& point) const
{
	// Remap the viewport coordinates in pixels to [-1, 1]
	return {
		-1.0 + 2.0 * ((point.x - m_viewport[0]) / (m_viewport[2] - m_viewport[0])),
		-1.0 + 2.0 * ((point.y - m_viewport[1]) / (m_viewport[3] - m_viewport[1])),
	};
}

double computeMaximumCameraResolution(const Camera& camera)
{
	return std::max(
		camera.viewport().z - camera.viewport().x,
		camera.viewport().w - camera.viewport().y
	);
}

double computeMaximumCameraResolution(const std::vector<Camera>& cameras)
{
	double maximumResolution = 0.0;

	for (const auto& camera : cameras)
	{
		maximumResolution = std::max(
			maximumResolution,
			computeMaximumCameraResolution(camera)
		);
	}

	return maximumResolution;
}

std::vector<Camera> loadCamerasFromFiles(const std::vector<std::string>& files)
{
	std::vector<Camera> cameras;

	cameras.reserve(files.size());

	for (const auto& filename : files)
	{
		std::ifstream file(filename);

		if (file.is_open())
		{
			int imageWidth, imageHeight;
			glm::dvec3 eye, at, up;
			// These are not used by our camera model, but are still part of the file
			double fovy, aspectRatio, nearPlane, farPlane;
			glm::dmat4 matV, matP;

			file >> imageWidth >> imageHeight;

			file >> eye.x >> eye.y >> eye.z;
			file >> at.x >> at.y >> at.z;
			file >> up.x >> up.y >> up.z;

			file >> fovy;
			file >> aspectRatio;
			file >> nearPlane;
			file >> farPlane;

			file >> matV[0][0] >> matV[1][0] >> matV[2][0] >> matV[3][0];
			file >> matV[0][1] >> matV[1][1] >> matV[2][1] >> matV[3][1];
			file >> matV[0][2] >> matV[1][2] >> matV[2][2] >> matV[3][2];
			file >> matV[0][3] >> matV[1][3] >> matV[2][3] >> matV[3][3];

			file >> matP[0][0] >> matP[1][0] >> matP[2][0] >> matP[3][0];
			file >> matP[0][1] >> matP[1][1] >> matP[2][1] >> matP[3][1];
			file >> matP[0][2] >> matP[1][2] >> matP[2][2] >> matP[3][2];
			file >> matP[0][3] >> matP[1][3] >> matP[2][3] >> matP[3][3];

			file.close();

			cameras.emplace_back(eye, at, up, matV, matP, glm::dvec2(imageWidth, imageHeight));
		}
		else
		{
			std::cerr << "Could not open file: " << filename << std::endl;
		}
	}

	return cameras;
}

std::vector<std::vector<Ray>> computeRays(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points
)
{
	std::vector<std::vector<Ray>> rays(cameras.size());

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		for (const auto& point : points[c])
		{
			const glm::dvec3 points3d(point.x, point.y, 1.f);

			const auto unProjected = cameras[c].unProject(points3d);

			const auto origin = cameras[c].eye();
			const auto direction = glm::normalize(unProjected - origin);

			rays[c].emplace_back(origin, direction);
		}
	}

	return rays;
}
