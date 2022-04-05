#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Camera.h"

class PhenotypingSetup
{
public:
	PhenotypingSetup(double imageWidth,
	                 double imageHeight,
	                 std::vector<std::string> views,
	                 std::vector<Camera> cameras);

	double imageWidth() const;

	double imageHeight() const;

	const std::vector<std::string>& views() const;

	const std::vector<Camera>& cameras() const;

private:
	double m_imageWidth;
	double m_imageHeight;

	std::vector<std::string> m_views;

	std::vector<Camera> m_cameras;
};

class PlantLeafTips
{
public:
	PlantLeafTips() = default;
	explicit PlantLeafTips(std::string plantName);

	/**
	 * \brief Return the name of the plant
	 * \return The name of the plant
	 */
	const std::string& plantName() const;

	/**
	 * \brief Return the list of all views that have points
	 * \return The list of all views that have points
	 */
	std::vector<std::string> getAllViews() const;

	/**
	 * \brief Check if the plant has all views in a list
	 * \param viewNames A list of view names
	 * \return True if the plant has all views, false otherwise
	 */
	bool hasAllViews(const std::vector<std::string>& viewNames) const;

	/**
	 * \brief Add 2D points seen from a view
	 * \param viewName The name of the view
	 * \param points The list of 2D points seen from this view
	 */
	void addPointsFromView(const std::string& viewName, const std::vector<glm::vec2>& points);

	/**
	 * \brief Flip the coordinates of points on the Y axis
	 * \param imageHeight Resolution of the image on the Y axis
	 */
	void flipYAxis(double imageHeight);

	/**
	 * \brief Return a list of lists of points ordered by view
	 * \param viewNames The list of views where points are required, for example: {SV_0, SV_36, SV_72}
	 * \return A list of lists of 2D points
	 */
	std::vector<std::vector<glm::vec2>> pointsFromViews(const std::vector<std::string>& viewNames) const;

private:

	std::string m_plantName;

	// For each view, a list of 2D points corresponding to leaf tips
	// Views can be named: SV_0, SV_36, SV_72, ..., TV_90
	std::unordered_map<std::string, std::vector<glm::vec2>> m_points;

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

/**
 * \brief Flip the Y axis coordinates of a list of plants according to a phenotyping setup
 * \param setup The phenotyping setup used to image the plants
 * \param plants The list of plants to modify
 */
void flipYAxisOnAllPlants(const PhenotypingSetup& setup, std::vector<PlantLeafTips>& plants);

// TODO: Get a subset of cameras from a setup
// TODO: Read and apply translations to a list of plants

/**
 * \brief Only keep plants in the list with at least two views (required for 3D triangulation)
 * \param plants The list of plants to filter
 */
void keepOnlyPlantsWithMultipleViews(std::vector<PlantLeafTips>& plants);

/**
 * \brief Only keep plants in the list with all views
 * \param viewNames A list of name of views
 * \param plants The list of plants to filter
 */
void keepOnlyPlantsWithAllViews(const std::vector<std::string>& viewNames, std::vector<PlantLeafTips>& plants);

/**
 * \brief Find the 3D position of leaf tips in a plant
 * \param setup The phenotyping setup used to image the plant
 * \param plantLeafTips The leaf tips of the plant
 * \return A list of 3D points that correspond to the leaf tips
 */
std::vector<glm::vec3> triangulateLeafTips(const PhenotypingSetup& setup, const PlantLeafTips& plantLeafTips);
