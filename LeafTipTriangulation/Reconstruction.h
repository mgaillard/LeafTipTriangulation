#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

/**
 * \brief Measure the length between two points triangulated without correspondences
 * \param inputPoints2D 2D coordinates of points
 * \return The distance between the two points
 */
float measureTwoPointsCharuco(const std::vector<std::vector<glm::vec2>>& inputPoints2D);
