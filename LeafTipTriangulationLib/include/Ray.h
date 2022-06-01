#pragma once

#include <algorithm>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

class Ray
{
public:
	// Origin of the ray
	glm::vec3 origin;
	// Direction of the ray (normalized vector)
	glm::vec3 direction;

	/**
	 * \brief Construct a line ray
	 * \param origin Origin of the ray
	 * \param direction Direction of the ray (normalized vector)
	 */
	Ray(glm::vec3 origin, glm::vec3 direction) :
		origin(std::move(origin)),
		direction(std::move(direction)),
		m_clampRay(false),
		m_start(0.f),
		m_end(1.f)
	{
		
	}

	/**
	 * \brief Construct a line segment ray (clamped)
	 * \param origin Origin of the ray
	 * \param direction Direction of the ray (normalized vector)
	 * \param start Parametric coordinate of the start of the line segment
	 * \param end Parametric coordinate of the end of the line segment
	 */
	Ray(glm::vec3 origin, glm::vec3 direction, float start, float end) :
		origin(std::move(origin)),
		direction(std::move(direction)),
		m_clampRay(true),
		m_start(start),
		m_end(end)
	{

	}

	bool isClamped() const
	{
		return m_clampRay;
	}

	float start() const
	{
		return m_start;
	}

	float end() const
	{
		return m_end;
	}

	glm::vec3 at(float t) const
	{
		if (m_clampRay)
		{
			t = std::clamp(t, m_start, m_end);
		}

		return origin + t * direction;
	}

private:
	// Boolean to clamp the ray to a line segment instead of a line
	const bool m_clampRay;
	// If clampRay is true, this is the starting parametric coordinate of the ray line segment
	const float m_start;
	// If clampRay is false, this is the starting parametric coordinate of the ray line segment
	const float m_end;
};

/**
 * \brief Compute the pseudo-intersection of two rays in 3D space
 *        If rays are parallel return false
 * \param ray0 First ray
 * \param ray1 Second ray
 * \param c If the pseudo-intersection exists, this parameter is set with the result
 * \return True if the pseudo-intersection exist, false otherwise
 */
bool raysPseudoIntersection(const Ray& ray0, const Ray& ray1, glm::vec3& c);
