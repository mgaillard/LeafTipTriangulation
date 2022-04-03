#pragma once

#include <string>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Camera.h"

struct PhenotypingSetup
{
	double imageWidth;
	double imageHeight;

	std::vector<Camera> cameras;
};

struct PlantLeafTips
{
	std::string plantName;

	// For each view, a list of 2D points corresponding to leaf tips
	std::vector<std::vector<glm::vec2>> tips;
};

/**
 * \brief Load a phenotyping setup
 * \return A phenotyping setup
 */
PhenotypingSetup loadPhenotypingSetup();

/**
 * \brief Read leaf tips data from a CSV file
 * \param filename Path to the CSV file
 * \return A list of plants with their leaf tips
 */
std::vector<PlantLeafTips> readLeafTipsFromCSV(const std::string& filename);

// TODO: Read translation for each plant in another file
// TODO: Apply translations to a list of plants

/**
 * \brief Find the 3D position of leaf tips in a plant
 * \param setup The phenotyping setup used to image the plant
 * \param plantLeafTips The leaf tips of the plant
 * \return A list of 3D points that correspond to the leaf tips
 */
std::vector<glm::vec3> triangulateLeafTips(const PhenotypingSetup& setup, const PlantLeafTips& plantLeafTips);
