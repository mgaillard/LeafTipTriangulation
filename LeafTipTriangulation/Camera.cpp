#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>

Camera::Camera(const glm::vec3& eye, const glm::vec3& at, const glm::vec3& up):
	m_eye(eye),
	m_at(at),
	m_up(up),
	m_matV(glm::lookAt(eye, at, up)), // Generate the view matrix
	m_matP(glm::perspective(45.f, 1.f, 0.1f, 10.f)), // Generate the projection matrix
	m_mat(m_matP * m_matV),
	m_viewport(0, 0, 1000, 1000)
{
	
}

Camera::Camera(const glm::vec3& eye,
	           const glm::vec3& at,
	           const glm::vec3& up,
	           const glm::mat4& matV,
	           const glm::mat4& matP,
	           const glm::vec2& viewportSize) :
	m_eye(eye),
	m_at(at),
	m_up(up),
	m_matV(matV),
	m_matP(matP),
	m_mat(m_matP * m_matV),
	m_viewport(0, 0, viewportSize.x, viewportSize.y)
{
}

glm::vec3 Camera::project(const glm::vec3& point) const
{
	return glm::projectNO(point,
	                      glm::identity<glm::mat4>(),
	                      m_mat,
	                      m_viewport);
}

glm::vec3 Camera::unProject(const glm::vec3& point) const
{
	return glm::unProjectNO(point,
	                        glm::identity<glm::mat4>(),
	                        m_mat,
	                        m_viewport);
}

glm::vec2 Camera::windowToViewport(const glm::vec2& point) const
{
	// Remap the viewport coordinates in pixels to [-1, 1]
	return {
		-1.f + 2.0f * ((point.x - m_viewport[0]) / (m_viewport[2] - m_viewport[0])),
		-1.f + 2.0f * ((point.y - m_viewport[1]) / (m_viewport[3] - m_viewport[1])),
	};
}
