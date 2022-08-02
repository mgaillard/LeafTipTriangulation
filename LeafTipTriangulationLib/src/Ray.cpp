#include "Ray.h"

#include <algorithm>
#include <array>

#include <utils/warnoff.h>
#include <glm/ext/scalar_constants.hpp>
#include <glm/gtx/norm.hpp>

#include <spdlog/spdlog.h>
#include <utils/warnon.h>

// ------------------------------------------
// Private functions
// ------------------------------------------

/**
 * \brief Compute the pseudo intersection of two lines
 * \param a0 Start of line 0
 * \param b0 End of line 0
 * \param a1 Start of line 1
 * \param b1 End of line 1
 * \param c Output the coordinates of the pseudo intersection of the two lines if they are not parallel
 * \return True if lines are not parallel and the pseudo-intersection is defined, false otherwise
 */
bool linesPseudoIntersection(
	const glm::dvec3& a0,
	const glm::dvec3& b0,
	const glm::dvec3& a1,
	const glm::dvec3& b1,
	glm::dvec3& c)
{
	const std::array<glm::dvec3, 2> u = {{
		glm::normalize(b0 - a0),
		glm::normalize(b1 - a1)
	}};
	
	// Failure case: the lines are parallel and the pseudo-intersection does not exist
	// Epsilon is scaled to the magnitude of u0 + u1, which is 2.0f
	if (glm::length(glm::cross(u[0], u[1])) <= 2.0 * glm::epsilon<double>())
	{
		// We can't solve the equation system
		return false;
	}

	std::array<glm::dmat3, 2> A = { glm::dmat3(0.0), glm::dmat3(0.0) };
	for (unsigned int i = 0; i < A.size(); i++)
	{
		// glm::dmat3 is column major
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
	c = glm::inverse(A[0] + A[1]) * ((A[0] * a0) + (A[1] * a1));

	return true;
}

/**
 * \brief Projection of the point P on the line defined by (AB)
 * \param p The point to project on the line
 * \param a The starting point that defines the line
 * \param b The ending point that defines the line
 * \return The parametric coordinate of the projection of P on (AB)
 */
double pointLineProjection(const glm::dvec3& p, const glm::dvec3& a, const glm::dvec3& b)
{
	const auto ap = p - a;
	const auto ab = b - a;

	// Segment is only a point and has no length
	if (glm::length2(ab) <= 0.0)
	{
		// The nearest point on the segment is A (or B)
		return 0.0;
	}

	// Segment has a length greater than 0
	// Projection of the point p on the line (AB)
	return glm::dot(ap, ab) / glm::length2(ab);
}

/**
 * \brief Projection of the point P on the line segment defined by [AB]
 * \param p The point to project on the line segment
 * \param a The starting point of the line segment
 * \param b The ending point of the line segment
 * \return The parametric coordinate of the projection of P on [AB]
 */
double pointLineSegmentProjection(const glm::dvec3& p, const glm::dvec3& a, const glm::dvec3& b)
{
	const auto u = pointLineProjection(p, a, b);
	return std::clamp(u, 0.0, 1.0);
}

/**
 * \brief Computes the distance from the point P to the line defined by (AB(])
 * \param p The point from which to compute the distance to the line segment
 * \param a The starting point of the line
 * \param b The ending point of the line
 * \param c The projection of point P on the line
 * \return The distance from the point P to the line (AB)
 */
double distToLine(const glm::dvec3& p, const glm::dvec3& a, const glm::dvec3& b, glm::dvec3& c)
{
	const auto ab = b - a;
	const auto u = pointLineProjection(p, a, b);

	// Projection of P is on the line (AB)
	c = a + ab * u;

	return glm::distance(p, c);
}

/**
 * \brief Computes the distance from the point P to the line segment defined by [AB]
 * \param p The point from which to compute the distance to the line segment
 * \param a The starting point of the line segment
 * \param b The ending point of the line segment
 * \param c The projection of point P on the line segment
 * \return The distance from the point P to the line segment [AB]
 */
double distToLineSegment(const glm::dvec3& p, const glm::dvec3& a, const glm::dvec3& b, glm::dvec3& c)
{
	const auto ab = b - a;
	const auto u = pointLineSegmentProjection(p, a, b);

	// Projection of P is between A and B
	c = a + ab * u;
	
	return glm::distance(p, c);
}

/**
 * \brief Compute the pseudo intersection of a line segment with a line
 *        Warning: the two directions cannot be parallel, it should be tested before calling this function.
 * \param lineA Start of line 0
 * \param lineB End of line segment 0
 * \param lineSegmentA Start of line segment 1
 * \param lineSegmentB End of line segment 1
 * \param c The pseudo intersection of the two line segments, if it exists
 * \return True if the pseudo-intersection exists, false otherwise
 */
bool lineAndLineSegmentPseudoIntersection(
	const glm::dvec3& lineA,
	const glm::dvec3& lineB,
	const glm::dvec3& lineSegmentA,
	const glm::dvec3& lineSegmentB,
	glm::dvec3& c)
{
	// Check the pseudo intersection of the two lines
	glm::dvec3 pseudoIntersection;
	const auto isNotParallel = linesPseudoIntersection(lineA, lineB, lineSegmentA, lineSegmentB, pseudoIntersection);

	// If the line segment is parallel to the line, the pseudo-intersection is not defined
	if (!isNotParallel)
	{
		return false;
	}

	// Project back to the line segment and check that it is between A and B
	const auto u = pointLineProjection(pseudoIntersection, lineSegmentA, lineSegmentB);

	if (u >= 0.0 && u <= 1.0)
	{
		// Keep this pseudo intersection because it is equivalent to having two lines
		c = pseudoIntersection;
		return true;
	}
	else if (u < 0.0)
	{
		// Clamp at A
		glm::dvec3 projectionOnLine;
		distToLine(lineSegmentA, lineA, lineB, projectionOnLine);
		c = (lineSegmentA + projectionOnLine) / 2.0;
		return true;
	}
	else if (u > 1.0)
	{
		// Clamp at B
		glm::dvec3 projectionOnLine;
		distToLine(lineSegmentB, lineA, lineB, projectionOnLine);
		c = (lineSegmentB + projectionOnLine) / 2.0;
		return true;
	}
	
	return false;
}

/**
 * \brief Compute the pseudo intersection of two line segments
 * \param a0 Start of line segment 0
 * \param b0 End of line segment 0
 * \param a1 Start of line segment 1
 * \param b1 End of line segment 1
 * \param c The pseudo intersection of the two line segments, if it exists
 * \return True if the pseudo-intersection exists, false otherwise
 */
bool lineSegmentsPseudoIntersection(
	const glm::dvec3& a0,
	const glm::dvec3& b0,
	const glm::dvec3& a1,
	const glm::dvec3& b1,
	glm::dvec3& c)
{
	// Source: https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments

	// Check the pseudo intersection of the two line segments as if they were lines
	glm::dvec3 pseudoIntersection;
	const auto isNotParallel = linesPseudoIntersection(a0, b0, a1, b1, pseudoIntersection);

	// If the lines are parallel, the pseudo-intersection may not be defined
	if (!isNotParallel)
	{
		// Project all the end points A and B on the line 0
		// The projection of A on the line (AB) has parametric coordinate 0.0
		const auto ua0 = 0.0;
		// The projection of B on the line (AB) has parametric coordinate 1.0
		const auto ub0 = 1.0;
		const auto ua1 = pointLineProjection(a1, a0, b0);
		const auto ub1 = pointLineProjection(b1, a0, b0);

		// Sort the line segments to make mirror cases easier to handle
		// By default, line segment 1 is in the same direction as line segment 0
		double ustart1 = ua1;
		double uend1 = ub1;
		glm::dvec3 start1 = a1;
		glm::dvec3 end1 = b1;
		// In case the line segment 1 is in the opposite direction as line segment 0
		if (ub1 < ua1)
		{
			// Swap the end points
			std::swap(ustart1, uend1);
			std::swap(start1, end1);
		}

		// For the rest of the block, we can assume that the points are in this order
		assert(ua0 < ub0);
		assert(ustart1 < uend1);

		// Handle different cases based on the ordering of points
		// The line segment 1 is below the line segment 0
		if (uend1 <= ua0)
		{
			c = (a0 + end1) / 2.0;
			return true;
		}
		// The line segment 1 is above the line segment 0
		if (ustart1 >= ub0)
		{
			c = (b0 + start1) / 2.0;
			return true;
		}

		// In all other cases, there is no unique pseudo-intersection
		return false;
	}

	// Project back to the line segment and check that it is between A and B
	const auto u = pointLineProjection(pseudoIntersection, a0, b0);
	const auto v = pointLineProjection(pseudoIntersection, a1, b1);

	// u and v can be in 3 different zones w.r.t. the line segment
	// - Before the start of the line segment (u < 0.0)
	// - In the line segment (u >= 0.0 && u <= 1.0)
	// - After the end of the line segment (u > 1.0)
	// Therefore, there are a total of 9 cases to handle differently
	if (u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0)
	{
		// Keep this pseudo intersection because it is equivalent to having two lines
		c = pseudoIntersection;
		return true;
	}
	else if (u < 0.0 && v >= 0.0 && v <= 1.0)
	{
		// Clamp line segment 0 at A
		glm::dvec3 projectionOnLine;
		distToLine(a0, a1, b1, projectionOnLine);
		c = (a0 + projectionOnLine) / 2.0;
		return true;
	}
	else if (u > 1.0 && v >= 0.0 && v <= 1.0)
	{
		// Clamp line segment 0 at B
		glm::dvec3 projectionOnLine;
		distToLine(b0, a1, b1, projectionOnLine);
		c = (b0 + projectionOnLine) / 2.0;
		return true;
	}
	else if (u >= 0.0 && u <= 1.0 && v < 0.0)
	{
		// Clamp line segment 1 at A
		glm::dvec3 projectionOnLine;
		distToLine(a1, a0, b0, projectionOnLine);
		c = (a1 + projectionOnLine) / 2.0;
		return true;
	}
	else if (u >= 0.0 && u <= 1.0 && v > 1.0)
	{
		// Clamp line segment 1 at B
		glm::dvec3 projectionOnLine;
		distToLine(b1, a0, b0, projectionOnLine);
		c = (b1 + projectionOnLine) / 2.0;
		return true;
	}
	else if (u < 0.0 && v < 0.0)
	{
		// Clamp both line segments at A
		c = (a0 + a1) / 2.0;
		return true;
	}
	else if (u < 0.0 && v > 1.0)
	{
		// Clamp line segment 0 at A and line segment 1 at B
		c = (a0 + b1) / 2.0;
		return true;
	}
	else if (u > 1.0 && v < 0.0)
	{
		// Clamp line segment 0 at B and line segment 1 at A
		c = (a1 + b0) / 2.0;
		return true;
	}
	else if (u > 1.0 && v > 1.0)
	{
		// Clamp both line segments at B
		c = (b0 + b1) / 2.0;
		return true;
	}

	return false;
}

// ------------------------------------------
// Public functions
// ------------------------------------------

bool raysPseudoIntersection(const Ray& ray0, const Ray& ray1, glm::dvec3& c)
{
	// If rays are lines
	if (!ray0.isClamped() && !ray1.isClamped())
	{
		const auto a0 = ray0.origin;
		const auto b0 = ray0.origin + ray0.direction;
		const auto a1 = ray1.origin;
		const auto b1 = ray1.origin + ray1.direction;
		
		glm::dvec3 pseudoIntersection;
		const auto success = linesPseudoIntersection(a0, b0, a1, b1, pseudoIntersection);

		if (success)
		{
			c = pseudoIntersection;
		}
		
		return success;
	}
	// If rays are line segments
	else if (ray0.isClamped() && ray1.isClamped())
	{
		const auto a0 = ray0.at(ray0.start());
		const auto b0 = ray0.at(ray0.end());
		const auto a1 = ray1.at(ray1.start());
		const auto b1 = ray1.at(ray1.end());

		glm::dvec3 pseudoIntersection;
		const auto success = lineSegmentsPseudoIntersection(a0, b0, a1, b1, pseudoIntersection);
		
		if (success)
		{
			c = pseudoIntersection;
		}

		return success;
	}
	// If the two rays are mixed: line and line segment
	else if (ray0.isClamped() && !ray1.isClamped())
	{
		const auto lineSegmentA = ray0.at(ray0.start());
		const auto lineSegmentB = ray0.at(ray0.end());
		const auto lineA = ray1.origin;
		const auto lineB = ray1.origin + ray1.direction;

		glm::dvec3 pseudoIntersection;
		const auto success = lineAndLineSegmentPseudoIntersection(lineA, lineB, lineSegmentA, lineSegmentB, pseudoIntersection);

		if (success)
		{
			c = pseudoIntersection;
		}

		return success;
	}
	else if (!ray0.isClamped() && ray1.isClamped())
	{
		const auto lineA = ray0.origin;
		const auto lineB = ray0.origin + ray0.direction;
		const auto lineSegmentA = ray1.at(ray1.start());
		const auto lineSegmentB = ray1.at(ray1.end());

		glm::dvec3 pseudoIntersection;
		const auto success = lineAndLineSegmentPseudoIntersection(lineA, lineB, lineSegmentA, lineSegmentB, pseudoIntersection);

		if (success)
		{
			c = pseudoIntersection;
		}

		return success;
	}

	return false;
}

double pointToRayDistance(const glm::dvec3& point, const Ray& ray)
{
	double dist = 0.0;

	// If the ray is a line
	if (!ray.isClamped())
	{
		const auto lineA = ray.origin;
		const auto lineB = ray.origin + ray.direction;
		glm::dvec3 c;
		dist = distToLine(point, lineA, lineB, c);
	}
	// If the ray is a line segment
	else
	{
		const auto lineSegmentA = ray.at(ray.start());
		const auto lineSegmentB = ray.at(ray.end());
		glm::dvec3 c;
		dist = distToLineSegment(point, lineSegmentA, lineSegmentB, c);
	}

	return dist;
}
