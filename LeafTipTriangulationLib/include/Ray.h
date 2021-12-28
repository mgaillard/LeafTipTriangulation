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
