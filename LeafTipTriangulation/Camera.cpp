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
