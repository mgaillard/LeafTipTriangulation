#pragma once

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

struct Ray
{
	glm::vec3 origin;
	glm::vec3 direction;

	glm::vec3 at(float t) const
	{
		return origin + t * direction;
	}
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
