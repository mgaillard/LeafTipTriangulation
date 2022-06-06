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
	// TODO: If line segments are parallel it's possible that they have a unique pseudo intersection if one segment is "before" the other
	// Source: https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments

	// Failure case 1: ray0 and ray1 are parallel
	// Epsilon is scaled to the magnitude of u0 + u1, which is 2.0f
	if (glm::length(glm::cross(b0 - a0, b1 - a1)) <= 2.0 * glm::epsilon<double>())
	{
		// We can't solve the equation system
		return false;
	}

	// Brute force approach
	// TODO: Implement better approach

	double minDist = std::numeric_limits<double>::max();

	constexpr int N = 10000;
	for (int i = 0; i < N; i++)
	{
		const auto u = static_cast<double>(i) / (N - 1);
		const auto point = a1 + u * (b1 - a1);

		glm::dvec3 closestPoint;
		const auto dist = distToLineSegment(point, a0, b0, closestPoint);

		if (dist < minDist)
		{
			minDist = dist;
			c = (closestPoint + point) / 2.0;
		}
	}

	return true;
}


// ------------------------------------------
// Public functions
// ------------------------------------------

bool raysPseudoIntersection(const Ray& ray0, const Ray& ray1, glm::vec3& c)
{
	// If rays are lines
	if (!ray0.isClamped() && !ray1.isClamped())
	{
		const glm::dvec3 a0 = ray0.origin;
		const glm::dvec3 b0 = ray0.origin + ray0.direction;
		const glm::dvec3 a1 = ray1.origin;
		const glm::dvec3 b1 = ray1.origin + ray1.direction;
		
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
		const glm::dvec3 a0 = ray0.at(ray0.start());
		const glm::dvec3 b0 = ray0.at(ray0.end());
		const glm::dvec3 a1 = ray1.at(ray1.start());
		const glm::dvec3 b1 = ray1.at(ray1.end());

		glm::dvec3 pseudoIntersection;
		lineSegmentsPseudoIntersection(a0, b0, a1, b1, pseudoIntersection);
		c = pseudoIntersection;

		return true;
	}
	// If the two rays are mixed: line and line segment
	else if (ray0.isClamped() && !ray1.isClamped())
	{
		const glm::dvec3 lineSegmentA = ray0.at(ray0.start());
		const glm::dvec3 lineSegmentB = ray0.at(ray0.end());
		const glm::dvec3 lineA = ray1.origin;
		const glm::dvec3 lineB = ray1.origin + ray1.direction;

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
		const glm::dvec3 lineA = ray0.origin;
		const glm::dvec3 lineB = ray0.origin + ray0.direction;
		const glm::dvec3 lineSegmentA = ray1.at(ray1.start());
		const glm::dvec3 lineSegmentB = ray1.at(ray1.end());

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


