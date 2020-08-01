#pragma once

#include <glm/glm.hpp>

class Camera
{
public:
	Camera(const glm::vec3& eye, const glm::vec3& at, const glm::vec3& up);

	const glm::vec3& eye() const { return m_eye; }
	const glm::vec3& at() const { return m_at; }
	const glm::vec3& up() const { return m_up; }

	const glm::mat4& matV() const { return m_matV; }
	const glm::mat4& matP() const { return m_matP; }
	const glm::mat4& mat() const { return m_mat; }

	const glm::vec4& viewport() const { return m_viewport; }

	glm::vec3 project(const glm::vec3& point) const;

	glm::vec3 unProject(const glm::vec3& point) const;

private:
	glm::vec3 m_eye;
	glm::vec3 m_at;
	glm::vec3 m_up;

	glm::mat4 m_matV;
	glm::mat4 m_matP;
	glm::mat4 m_mat;

	glm::vec4 m_viewport;
};
