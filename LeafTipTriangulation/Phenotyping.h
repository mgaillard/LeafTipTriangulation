#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <random>

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

	/**
	 * \brief Return a subset of cameras from a list of view names
	 *        If one of the views is not available, it will not be added to the output list of cameras
	 * \return A subset of cameras from a list of view names
	 */
	[[nodiscard]] std::vector<Camera> camerasFromViews(const std::vector<std::string>& viewNames) const;

	/**
	 * \brief Remove a view from the phenotyping setup
	 * \param viewName The name of the view to remove
	 * \return True if the view was found and removed
	 */
	bool removeView(const std::string& viewName);

private:
	double m_imageWidth;
	double m_imageHeight;

	std::vector<std::string> m_views;

	std::vector<Camera> m_cameras;
};

class PlantPhenotypePoints
{
public:
	PlantPhenotypePoints() = default;
	explicit PlantPhenotypePoints(std::string plantName);

	/**
	 * \brief Return the name of the plant
	 * \return The name of the plant
	 */
	[[nodiscard]] const std::string& plantName() const;

	/**
	 * \brief Return the list of all views that have points
	 * \return The list of all views that have points
	 */
	[[nodiscard]] std::vector<std::string> getAllViews() const;

	/**
	 * \brief Check if the plant has all views in a list
	 * \param viewNames A list of view names
	 * \return True if the plant has all views, false otherwise
	 */
	[[nodiscard]] bool hasAllViews(const std::vector<std::string>& viewNames) const;

	/**
	 * \brief Add 2D points seen from a view
	 * \param viewName The name of the view
	 * \param points The list of 2D points seen from this view
	 */
	void addPointsFromView(const std::string& viewName, const std::vector<glm::vec2>& points);

	/**
	 * \brief Apply a translation to all points from a view
	 * \param viewName The name of the view
	 * \param translation The 2D translation to apply
	 */
	void applyTranslationToView(const std::string& viewName, const glm::vec2& translation);

	/**
	 * \brief Apply a 90 degrees clockwise rotation to all points from a view
	 *        The center of the rotation is the center of the image, therefore we need the resolution
	 *		  The image starts with resolution (width, height)
	 *		  After the rotation the resolution is still (width, height)
	 * \param viewName The name of the view
	 * \param imageWidth Resolution of the image on the X axis
	 * \param imageHeight Resolution of the image on the Y axis
	 */
	void apply90DegreesClockwiseRotationToView(const std::string& viewName, double imageWidth, double imageHeight);

	/**
	 * \brief Apply a 90 degrees counterclockwise rotation to all points from a view
	 *        The center of the rotation is the center of the image, therefore we need the resolution
	 *		  The image starts with resolution (height, width)
	 *		  After the rotation the resolution is still (width, height)
	 * \param viewName The name of the view
	 * \param imageWidth Resolution of the image on the X axis
	 * \param imageHeight Resolution of the image on the Y axis
	 */
	void apply90DegreesCounterClockwiseRotationToView(const std::string& viewName,
	                                                  double imageWidth,
	                                                  double imageHeight);

	/**
	 * \brief Flip the coordinates of points on the Y axis
	 * \param imageHeight Resolution of the image on the Y axis
	 */
	void flipYAxis(double imageHeight);

	/**
	 * \brief Randomly discard leaf tips in the plant
	 * \tparam RandomGenerator A std random generator initialized with a seed
	 * \param generator A random generator
	 * \param probability A probability to discard a leaf tip
	 */
	template<class RandomGenerator>
	void discardPointsRandomly(RandomGenerator& generator, double probability)
	{
		std::uniform_real_distribution<double> dist(0.0, 1.0);

		for (auto& [viewName, points] : m_points)
		{
			for (auto itPoint = points.begin(); itPoint != points.end();)
			{
				// Generate a random number between 0 and 1
				const auto randomNumber = dist(generator);
				// If the random number is higher than the probability, we keep
				if (randomNumber > probability)
				{
					// Keep the point
					++itPoint;
				}
				else
				{
					// Discard the point
					itPoint = points.erase(itPoint);
				}
			}
		}
	}

	/**
	 * \brief Return a list of points from a view
	 * \param viewName The list of the view, for example: {SV_0, SV_36, SV_72}
	 * \return A list of 2D points
	 */
	std::vector<glm::vec2> pointsFromView(const std::string& viewName) const;

	/**
	 * \brief Return a list of lists of points ordered by view
	 * \param viewNames The list of views where points are required, for example: {SV_0, SV_36, SV_72}
	 * \return A list of lists of 2D points
	 */
	[[nodiscard]] std::vector<std::vector<glm::vec2>> pointsFromViews(const std::vector<std::string>& viewNames) const;

private:

	std::string m_plantName;

	// For each view, a list of 2D points corresponding to leaf tips
	// Views can be named: SV_0, SV_36, SV_72, ..., TV_90
	std::unordered_map<std::string, std::vector<glm::vec2>> m_points;

};

enum class PlantPhenotypePointType
{
	LeafTip,
	LeafJunction
};

enum class RotationDirection
{
	Unknown,
	Clockwise,
	Counterclockwise
};

/**
 * \brief Load the rotation direction for the top view
 * \param rotationFile Path to the file containing the rotation direction
 * \return The rotation 
 */
RotationDirection loadTopViewRotationDirection(const std::string& rotationFile);

/**
 * \brief Read the type of phenotype from a string
 *        By default, return PlantPhenotypePointType::LeafTip
 * \param phenotype The string of the phenotype ("tips", "junctions")
 * \return The corresponding value in the enum class PlantPhenotypePointType
 */
PlantPhenotypePointType readPhenotypingPointTypeFromString(const std::string& phenotype);

/**
 * \brief Translate a view name like "SV_0" to a filename like "0_0_0.png"
 * \param viewName The view name. For example: "SV_0", "SV_72", "TV_90"
 * \param extension Extension for the filename (jpg, png) 
 * \return The filename corresponding to the view name
 */
std::string translateViewNameToFilename(const std::string& viewName, const std::string& extension);

/**
 * \brief Load a phenotyping setup
 * \param cameraFolder Path to the folder containing the camera calibration files
 * \return A phenotyping setup
 */
PhenotypingSetup loadPhenotypingSetup(const std::string& cameraFolder);

/**
 * \brief Read leaf tips data from a CSV file
 * \param filename Path to the CSV file
 * \param type Type of phenotype to read from the CSV file
 * \return A list of plants with their leaf tips
 */
std::vector<PlantPhenotypePoints> readPhenotypePointsFromCsv(const std::string& filename, PlantPhenotypePointType type);

/**
 * \brief Flip the Y axis coordinates of a list of plants according to a phenotyping setup
 * \param setup The phenotyping setup used to image the plants
 * \param plants The list of plants to modify
 */
void flipYAxisOnAllPlants(const PhenotypingSetup& setup, std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Apply a 90 degrees clockwise rotation to points in a view for a list of plants
 * \param viewName The name of the view
 * \param setup The phenotyping setup used to image the plants
 * \param rotationDirection The direction of the rotation
 * \param plants The list of plants to modify
 */
void apply90DegreesRotationToViews(const std::string& viewName,
                                   const PhenotypingSetup& setup,
                                   const RotationDirection& rotationDirection,
                                   std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Convert the output of the calibration script to a CSV file
 *        that can be read with the function readAndApplyTranslationsFromCsv()
 * \param inputFilename Path to the input TXT file
 * \param outputFilename Path to the output CSV file
 */
void convertCalibrationOutputToCsv(const std::string& inputFilename, const std::string& outputFilename);

/**
 * \brief Read and apply translations used for calibration of views from a CSV file
 * \param filename Path to the CSV file
 * \param plants List of plants on which to apply the transformations
 */
void readAndApplyTranslationsFromCsv(const std::string& filename, std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Only keep plants in the list with at least two views (required for 3D triangulation)
 * \param plants The list of plants to filter
 */
void keepOnlyPlantsWithMultipleViews(std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Only keep plants in the list with all views
 * \param viewNames A list of name of views
 * \param plants The list of plants to filter
 */
void keepOnlyPlantsWithAllViews(const std::vector<std::string>& viewNames, std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Discard some annotated points randomly in the plant
 * \param seed Initialization value for the random generator
 * \param probability Probability to discard a point
 * \param plants The list of plants to process
 */
void discardPointsRandomly(unsigned int seed, double probability, std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Find the 3D position of leaf tips in a plant
 * \param setup The phenotyping setup used to image the plant
 * \param plantLeafTips The leaf tips of the plant
 * \return A list of 3D points that correspond to the leaf tips
 */
std::vector<glm::vec3> triangulateLeafTips(const PhenotypingSetup& setup, const PlantPhenotypePoints& plantLeafTips);

/**
 * \brief Draw a list of points in 2D image
 * \param filename The filename of the image
 * \param backgroundImage Image to load as background
 * \param points A list of 2D points to draw in the image
 * \return True if the image was successfully saved, false otherwise
 */
bool drawPointsInImage(const std::string& filename,
					   const std::string& backgroundImage,
                       const std::vector<glm::vec2>& points);

/**
 * \brief Export a table of the statistics of loaded plants
 * \param filename The path to the CSV file
 * \param setup The phenotyping setup (for the list of views)
 * \param plants The list of plants
 * \return True if the list was saved successfully
 */
bool exportPlantStatsToCsv(const std::string& filename,
	                       const PhenotypingSetup& setup,
	                       const std::vector<PlantPhenotypePoints>& plants);

/**
 * \brief Save the number of leaves per plant in a CSV file
 * \param filename The path to the CSV file
 * \param numberLeafTips A list of name of plants with the number of leaves
 * \return True if the list was saved successfully
 */
bool exportNumberLeavesToCsv(const std::string& filename,
	                         const std::vector<std::pair<std::string, int>>& numberLeafTips);
