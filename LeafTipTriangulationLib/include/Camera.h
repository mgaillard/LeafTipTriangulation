#pragma once

#include <string>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Ray.h"
#include "Types.h"

class Camera
{
public:
    Camera(const glm::dvec3& eye, const glm::dvec3& at, const glm::dvec3& up);

    Camera(const glm::dvec3& eye,
           const glm::dvec3& at,
           const glm::dvec3& up,
           double fovy,
           double aspectRatio,
           const glm::dvec2& viewportSize);

    Camera(const glm::dvec3& eye,
           const glm::dvec3& at,
           const glm::dvec3& up,
           const glm::dmat4& matV,
           const glm::dmat4& matP,
           const glm::dvec2& viewportSize);

    const glm::dvec3& eye() const { return m_eye; }
    const glm::dvec3& at() const { return m_at; }
    const glm::dvec3& up() const { return m_up; }

    const glm::dmat4& matV() const { return m_matV; }
    const glm::dmat4& matP() const { return m_matP; }
    const glm::dmat4& mat() const { return m_mat; }

    const glm::dvec4& viewport() const { return m_viewport; }

    glm::dvec3 project(const glm::dvec3& point) const;

    glm::dvec3 unProject(const glm::dvec3& point) const;

    glm::dvec2 windowToViewport(const glm::dvec2& point) const;

private:
    glm::dvec3 m_eye;
    glm::dvec3 m_at;
    glm::dvec3 m_up;

    glm::dmat4 m_matV;
    glm::dmat4 m_matP;
    glm::dmat4 m_mat;

    glm::dvec4 m_viewport;
};

/**
 * \brief Return the maximum resolution on X axis or Y axis of a camera
 * \param camera A camera
 * \return The maximum resolution of the camera 
 */
double computeMaximumCameraResolution(const Camera& camera);

/**
 * \brief Return the maximum resolution of a list of cameras
 * \param cameras A list of cameras
 * \return The maximum resolution of the list of cameras
 */
double computeMaximumCameraResolution(const std::vector<Camera>& cameras);

/**
 * \brief Load camera from a list of files
 * \param files A list of files containing the camera information
 * \return A vector of camera
 */
std::vector<Camera> loadCamerasFromFiles(const std::vector<std::string>& files);

/**
 * \brief Project 3D points on cameras. The viewport is 1000*1000 px.
 * \param points3d A list of 3D points
 * \param cameras A list of cameras
 * \return 2D points projected on cameras
 */
SetsOfVec2 projectPoints(const SetOfVec3& points3d,
                         const std::vector<Camera>& cameras);

/**
 * \brief Compute rays going from camera to the 3D space
 * \param cameras A list of cameras
 * \param points A list of 2D points per camera
 * \return A list of rays from cameras
 */
SetsOfRays computeRays(const std::vector<Camera>& cameras,
                       const SetsOfVec2& points);
