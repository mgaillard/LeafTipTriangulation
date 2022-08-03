#pragma once

#include <algorithm>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

class Ray
{
public:
    // Origin of the ray
    glm::dvec3 origin;
    // Direction of the ray (normalized vector)
    glm::dvec3 direction;

    /**
     * \brief Construct a line ray
     * \param origin Origin of the ray
     * \param direction Direction of the ray (normalized vector)
     */
    Ray(const glm::dvec3& origin, const glm::dvec3& direction) :
        origin(origin),
        direction(direction),
        m_clampRay(false),
        m_start(0.0),
        m_end(1.0)
    {
        
    }

    /**
     * \brief Construct a line segment ray (clamped)
     * \param origin Origin of the ray
     * \param direction Direction of the ray (normalized vector)
     * \param start Parametric coordinate of the start of the line segment
     * \param end Parametric coordinate of the end of the line segment
     */
    Ray(const glm::dvec3& origin, const glm::dvec3& direction, double start, double end) :
        origin(origin),
        direction(direction),
        m_clampRay(true),
        m_start(start),
        m_end(end)
    {

    }

    void clampRay(double start, double end)
    {
        m_start = start;
        m_end = end;
        m_clampRay = true;
    }

    bool isClamped() const
    {
        return m_clampRay;
    }

    double start() const
    {
        return m_start;
    }

    double end() const
    {
        return m_end;
    }

    glm::dvec3 at(double t) const
    {
        if (m_clampRay)
        {
            t = std::clamp(t, m_start, m_end);
        }

        return origin + t * direction;
    }

private:
    // Boolean to clamp the ray to a line segment instead of a line
    bool m_clampRay;
    // If clampRay is true, this is the starting parametric coordinate of the ray line segment
    double m_start;
    // If clampRay is false, this is the starting parametric coordinate of the ray line segment
    double m_end;
};

/**
 * \brief Compute the pseudo-intersection of two rays in 3D space
 *        If rays are parallel return false
 * \param ray0 First ray
 * \param ray1 Second ray
 * \param c If the pseudo-intersection exists, this parameter is set with the result
 * \return True if the pseudo-intersection exist, false otherwise
 */
bool raysPseudoIntersection(const Ray& ray0, const Ray& ray1, glm::dvec3& c);

/**
 * \brief Compute the distance from a 3D point to a ray in 3D space
 * \param point The 3D point
 * \param ray The ray in 3D space
 * \return The shortest distance from the point to the ray
 */
double pointToRayDistance(const glm::dvec3& point, const Ray& ray);
