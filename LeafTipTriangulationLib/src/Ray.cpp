#include "Ray.h"

#include <array>

#include <utils/warnoff.h>
#include <glm/ext/scalar_constants.hpp>
#include <utils/warnon.h>

bool raysPseudoIntersection(const Ray& ray0, const Ray& ray1, glm::vec3& c)
{
	const std::array<glm::vec3, 2> u = { {ray0.direction, ray1.direction} };

	// Failure case 1: ray0 and ray1 are parallel
	// Epsilon is scaled to the magnitude of u0 + u1, which is 2.0f
	if (glm::length(glm::cross(u[0], u[1])) <= 2.f * glm::epsilon<float>())
	{
		// We can't solve the equation system
		return false;
	}

	std::array<glm::mat3, 2> A = { glm::mat3(0.f), glm::mat3(0.f) };
	for (unsigned int i = 0; i < A.size(); i++)
	{
		// glm::mat3 is column major
		auto& Ai = A[i];
		const auto& ui = u[i];

		// First row
		Ai[0][0] = (ui.y * ui.y) + (ui.z * ui.z);
		Ai[1][0] = -ui.x * ui.y;
		Ai[2][0] = -ui.x * ui.z;

		// Second row
		Ai[0][1] = -ui.x * ui.y;
		Ai[1][1] = (ui.x * ui.x) + (ui.z * ui.z);
		Ai[2][1] = -ui.y * ui.z;

		// Third row
		Ai[0][2] = -ui.x * ui.z;
		Ai[1][2] = -ui.y * ui.z;
		Ai[2][2] = (ui.x * ui.x) + (ui.y * ui.y);
	}

	// Closest point between the two lines
	c = glm::inverse(A[0] + A[1]) * ((A[0] * ray0.origin) + (A[1] * ray1.origin));

	return true;
}
