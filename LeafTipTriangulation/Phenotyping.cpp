#include "Phenotyping.h"

#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <tuple>

#include <utils/warnoff.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <utils/warnon.h>

#include "RayMatching.h"

namespace fs = std::filesystem;

/**
 * \brief Convert a vector of view names and a vector of indices of views to a string that can be displayed
 * \param viewNames A vector of view names
 * \param viewOrder A vector of integer values that gives the order of views, for example: "{3, 5, 1, 6, 4, 7, 0, 2}"
 * \return A string representing the ordered view names
 */
std::string orderedViewNamesToString(
	const std::vector<std::string>& viewNames,
	const std::vector<int>& viewOrder,
	const SetsOfVec2& points2d)
{
	std::stringstream out;

	out << "{";

	for (unsigned int i = 0; i < viewOrder.size(); i++)
	{
		if (i > 0)
		{
			out << ", ";
		}

		out << viewNames.at(viewOrder[i]) << "(" << points2d.at(viewOrder[i]).size() << ")";
	}

	out << "}";

	return out.str();
}

PhenotypingSetup::PhenotypingSetup(
	double imageWidth,
	double imageHeight,
	std::vector<std::string> views,
	std::vector<Camera> cameras,
	double distanceToPlant,
	double radiusPlant,
	RotationDirection topViewRotation) :
	m_imageWidth(imageWidth),
	m_imageHeight(imageHeight),
	m_views(std::move(views)),
	m_cameras(std::move(cameras)),
	m_distanceToPlant(distanceToPlant),
	m_radiusPlant(radiusPlant),
	m_topViewRotation(topViewRotation)
{

}

double PhenotypingSetup::imageWidth() const
{
	return m_imageWidth;
}

double PhenotypingSetup::imageHeight() const
{
	return m_imageHeight;
}

const std::vector<std::string>& PhenotypingSetup::views() const
{
	return m_views;
}

const std::vector<Camera>& PhenotypingSetup::cameras() const
{
	return m_cameras;
}

double PhenotypingSetup::distanceToPlant() const
{
	return m_distanceToPlant;
}

double PhenotypingSetup::radiusPlant() const
{
	return m_radiusPlant;
}

RotationDirection PhenotypingSetup::topViewRotation() const
{
	return m_topViewRotation;
}

std::vector<Camera> PhenotypingSetup::camerasFromViews(const std::vector<std::string>& viewNames) const
{
	std::vector<Camera> cameras;

	for (const auto& viewName : viewNames)
	{
		// Find the name in the list of names
		const auto viewIt = std::find(m_views.begin(), m_views.end(), viewName);

		if (viewIt != m_views.end())
		{
			// Add the corresponding camera
			const auto index = std::distance(m_views.begin(), viewIt);
			cameras.push_back(m_cameras[index]);
		}
	}

	return cameras;
}

bool PhenotypingSetup::removeView(const std::string& viewName)
{
	bool success = false;

	auto itViews = m_views.begin();
	auto itCameras = m_cameras.begin();

	while (itViews != m_views.end() && itCameras != m_cameras.end())
	{
		if (*itViews == viewName)
		{
			// Erase from both vectors
			itViews = m_views.erase(itViews);
			itCameras = m_cameras.erase(itCameras);

			success = true;
		}
		else
		{
			++itViews;
			++itCameras;
		}
	}

	return success;
}

PlantPhenotypePoints::PlantPhenotypePoints(std::string plantName) : m_plantName(std::move(plantName))
{
	
}

const std::string& PlantPhenotypePoints::plantName() const
{
	return m_plantName;
}

std::vector<std::string> PlantPhenotypePoints::getAllViews() const
{
	std::vector<std::string> viewNames;

	for (const auto& [viewName, viewPoints] : m_points)
	{
		viewNames.push_back(viewName);
	}

	// Sort the name of views so that when executed in different contexts, the results are the same
	std::sort(viewNames.begin(), viewNames.end());

	return viewNames;
}

bool PlantPhenotypePoints::hasAllViews(const std::vector<std::string>& viewNames) const
{
	for (const auto& viewName : viewNames)
	{
		if (m_points.count(viewName) == 0)
		{
			return false;
		}
	}

	return true;
}

void PlantPhenotypePoints::addPointsFromView(const std::string& viewName, const SetOfVec2& points)
{
	if (m_points.count(viewName) > 0)
	{
		std::cerr << "Found a duplicate view for a plant" << std::endl;
		// If already exists, add points to already existing points
		m_points[viewName].insert(m_points[viewName].end(), points.begin(), points.end());
	}
	else
	{
		m_points[viewName] = points;
	}	
}

void PlantPhenotypePoints::applyTranslationToView(const std::string& viewName, const glm::dvec2& translation)
{
	if (m_points.count(viewName) > 0)
	{
		for (auto& point : m_points[viewName])
		{
			point += translation;
		}
	}
}

void PlantPhenotypePoints::apply90DegreesRotationToView(
	const std::string& viewName,
	const RotationDirection& rotationDirection,
	double imageWidth,
	double imageHeight)
{
	if (m_points.count(viewName) > 0)
	{
		apply90DegreesRotationToPoints(rotationDirection, imageWidth, imageHeight, m_points[viewName]);
	}
}

void PlantPhenotypePoints::flipYAxis(double imageHeight)
{
	for (auto& points : m_points)
	{
		for (auto& point : points.second)
		{
			point.y = imageHeight - point.y;
		}
	}
}

void PlantPhenotypePoints::discardView(const std::string& viewName)
{
	const auto itView = m_points.find(viewName);

	if (itView != m_points.end())
	{
		m_points.erase(itView);
	}
}

std::string PlantPhenotypePoints::exportToCsv() const
{
	std::stringstream out;

	for (const auto& [viewName, points] : m_points)
	{
		out << m_plantName << "_Vis_" << viewName << ".png" << "\t";

		out << "[";
		for (unsigned int i = 0; i < points.size(); i++)
		{
			const auto x = static_cast<int>(points[i].x);
			const auto y = static_cast<int>(points[i].y);

			out << "[" << x << ", " << y << "]";

			if (i + 1 < points.size())
			{
				out << ", ";
			}
		}
		out << "]\t[]\n";
	}

	return out.str();
}

SetOfVec2 PlantPhenotypePoints::pointsFromView(const std::string& viewName) const
{
	if (m_points.count(viewName) > 0)
	{
		return m_points.at(viewName);
	}

	// An empty array because there are no points visible from this view
	return {};
}

SetsOfVec2 PlantPhenotypePoints::pointsFromViews(const std::vector<std::string>& viewNames) const
{
	SetsOfVec2 points;

	points.reserve(viewNames.size());
	for (const auto& viewName : viewNames)
	{
		points.push_back(pointsFromView(viewName));
	}

	return points;
}

void PlantImageTranslations::loadFromCsv(const std::string& filename)
{
	// Regex to match the name of the plant
	const std::string matchPlantName = R"((.+)_Vis_(\w{2}_\d+)\.(?:png|jpg))";
	// Regex to match a float number
	const std::string matchFloatNumber = R"(([-+]?[0-9]*\.?[0-9]+))";
	// Regex to match a space between sequences
	const std::string matchSpace = R"(\s)";
	// Regex to match a full line
	const std::regex matchTranslationLine(matchPlantName + matchSpace +
		matchFloatNumber + matchSpace + matchFloatNumber);

	std::ifstream file(filename);

	if (file.is_open())
	{
		std::string line;
		while (std::getline(file, line))
		{
			// Match the current line and extract information
			std::smatch matchesInLine;
			if (std::regex_match(line, matchesInLine, matchTranslationLine) && matchesInLine.size() == 5)
			{
				const std::string plantName = matchesInLine[1];
				const std::string plantView = matchesInLine[2];
				const std::string translationStrX = matchesInLine[3];
				const std::string translationStrY = matchesInLine[4];

				const auto x = std::stod(translationStrX);
				const auto y = std::stod(translationStrY);

				m_viewTranslations.emplace_back(plantName, plantView, x, y);
			}
		}

		file.close();
	}
}

glm::dvec2 PlantImageTranslations::getTranslationForPlantAndView(
	const std::string& plantName,
	const std::string& viewName) const
{
	// Search in the list of available translations
	for (const auto& viewTranslation : m_viewTranslations)
	{
		if (viewTranslation.plantName == plantName && viewTranslation.viewName == viewName)
		{
			return viewTranslation.translation;
		}
	}

	// By default no translation is a translation of (0, 0)
	return { 0.0, 0.0 };
}

void PlantImageTranslations::applyTranslationToPlants(std::vector<PlantPhenotypePoints>& plants) const
{
	for (auto& plant : plants)
	{
		const auto viewNames = plant.getAllViews();
		for (const auto& viewName : viewNames)
		{
			const auto translation = getTranslationForPlantAndView(plant.plantName(), viewName);
			plant.applyTranslationToView(viewName, translation);
		}
	}
}

void apply90DegreesRotationToPoints(
	const RotationDirection& rotationDirection,
	double imageWidth,
	double imageHeight,
	SetOfVec2& points)
{
	for (auto& point : points)
	{
		if (rotationDirection == RotationDirection::CounterclockwiseWithCanvas
			|| rotationDirection == RotationDirection::ClockwiseWithCanvas)
		{
			// Inverse translate the points from the center of rotation
			point.x -= imageHeight / 2.0;
			point.y -= imageWidth / 2.0;
		}
		else if (rotationDirection == RotationDirection::CounterclockwiseWithoutCanvas
			|| rotationDirection == RotationDirection::ClockwiseWithoutCanvas)
		{
			// Inverse translate the points from the center of rotation
			point.x -= imageWidth / 2.0;
			point.y -= imageHeight / 2.0;
		}

		if (rotationDirection == RotationDirection::CounterclockwiseWithCanvas
			|| rotationDirection == RotationDirection::CounterclockwiseWithoutCanvas)
		{
			// Apply the following transformation (rotation 90 deg counter-clockwise)
			// x' = y
			// y' = -x
			std::swap(point.x, point.y);
			point.y = -point.y;
		}
		else if (rotationDirection == RotationDirection::ClockwiseWithCanvas
			|| rotationDirection == RotationDirection::ClockwiseWithoutCanvas)
		{
			// Apply the following transformation (rotation 90 deg clockwise)
			// x' = -y
			// y' = x
			std::swap(point.x, point.y);
			point.x = -point.x;
		}

		if (rotationDirection != RotationDirection::Unknown)
		{
			// Translate the points back to the center of rotation
			// We apply the opposite transformation
			//  - width/2 is added to the X axis
			//  - height/2 is added to the Y axis
			// We do this because the image is rotated, but the canvas stays the same
			point.x += imageWidth / 2.0;
			point.y += imageHeight / 2.0;
		}
	}
}

std::tuple<RotationDirection, double, double> loadSetupConfiguration(const std::filesystem::path& configFile)
{
	auto direction = RotationDirection::Unknown;
	double distanceToPlant = 0.0;
	double radiusPlant = 0.0;

	std::ifstream file(configFile);

	if (file.is_open())
	{
		std::string directionString;
		file >> directionString;

		if (directionString == "ClockwiseWithCanvas")
		{
			direction = RotationDirection::ClockwiseWithCanvas;
		}
		else if (directionString == "ClockwiseWithoutCanvas")
		{
			direction = RotationDirection::ClockwiseWithoutCanvas;
		}
		else if (directionString == "CounterclockwiseWithCanvas")
		{
			direction = RotationDirection::CounterclockwiseWithCanvas;
		}
		else if (directionString == "CounterclockwiseWithoutCanvas")
		{
			direction = RotationDirection::CounterclockwiseWithoutCanvas;
		}

		file >> distanceToPlant >> radiusPlant;

		file.close();
	}

	return { direction, distanceToPlant, radiusPlant };
}

PlantPhenotypePointType readPhenotypingPointTypeFromString(const std::string& phenotype)
{
	// By default, triangulate leaf tips
	auto type = PlantPhenotypePointType::LeafTip;

	if (phenotype == "tips")
	{
		type = PlantPhenotypePointType::LeafTip;
	}
	else if (phenotype == "junctions")
	{
		type = PlantPhenotypePointType::LeafJunction;
	}

	return type;
}

std::string translateViewNameToFilename(const std::string& viewName, const std::string& extension)
{
	std::string filename;

	if (viewName == ViewSv0)
	{
		filename = ViewFileSv0;
	}
	else if (viewName == ViewSv36)
	{
		filename = ViewFileSv36;
	}
	else if (viewName == ViewSv72)
	{
		filename = ViewFileSv72;
	}
	else if (viewName == ViewSv108)
	{
		filename = ViewFileSv108;
	}
	else if (viewName == ViewSv144)
	{
		filename = ViewFileSv144;
	}
	else if (viewName == ViewSv216)
	{
		filename = ViewFileSv216;
	}
	else if (viewName == ViewSv252)
	{
		filename = ViewFileSv252;
	}
	else if (viewName == ViewSv288)
	{
		filename = ViewFileSv288;
	}
	else if (viewName == ViewSv324)
	{
		filename = ViewFileSv324;
	}
	else if (viewName == ViewTv90)
	{
		filename = ViewFileTv90;
	}
	else
	{
		// By default the view name is "default", which will most likely not exist and trigger an error.
		filename = "default";
	}

	return filename + '.' + extension;
}

PhenotypingSetup loadPhenotypingSetup(const fs::path& setupFolder)
{
	const auto cameraFolder = setupFolder / "cameras";

	const std::array<std::pair<std::string, fs::path>, 10> viewCameraNames = {{
		{ViewSv0, cameraFolder / ("camera_" + ViewFileSv0 + ".txt")},
		{ViewSv36, cameraFolder / ("camera_" + ViewFileSv36 + ".txt")},
		{ViewSv72, cameraFolder / ("camera_" + ViewFileSv72 + ".txt")},
		{ViewSv108, cameraFolder / ("camera_" + ViewFileSv108 + ".txt")},
		{ViewSv144, cameraFolder / ("camera_" + ViewFileSv144 + ".txt")},
		{ViewSv216, cameraFolder / ("camera_" + ViewFileSv216 + ".txt")},
		{ViewSv252, cameraFolder / ("camera_" + ViewFileSv252 + ".txt")},
		{ViewSv288, cameraFolder / ("camera_" + ViewFileSv288 + ".txt")},
		{ViewSv324, cameraFolder / ("camera_" + ViewFileSv324 + ".txt")},
		{ViewTv90, cameraFolder / ("camera_" + ViewFileTv90 + ".txt")}
	}};

	std::vector<std::string> views;
	std::vector<std::string> cameraFiles;

	// List views that are defined
	// and discard views that could not be loaded because the txt file does not exist
	for (const auto& viewCameraName : viewCameraNames)
	{
		if (std::filesystem::exists(viewCameraName.second))
		{
			views.push_back(viewCameraName.first);
			cameraFiles.push_back(viewCameraName.second.string());
		}
	}

	const auto cameras = loadCamerasFromFiles(cameraFiles);

	// Read the resolution from the first image
	const auto imageWidth = cameras[0].viewport().z;
	const auto imageHeight = cameras[0].viewport().w;

	// Read the rotation direction for the top view
	const auto configFile = setupFolder / "setup_config.txt";
	const auto [rotationDirection, distanceToPlant, radiusPlant] = loadSetupConfiguration(configFile);

	return {
		imageWidth,
		imageHeight,
		views,
		cameras,
		distanceToPlant,
		radiusPlant,
		rotationDirection
	};
}

std::vector<PlantPhenotypePoints> readPhenotypePointsFromCsv(const std::string& filename, PlantPhenotypePointType type)
{
	// Regex to match the name of the plant
	const std::string matchPlantName = R"((.+)_Vis_(\w{2}_\d+)\.(?:png|jpg))";
	// Regex to match the list of leaf tips of a plant
	const std::string matchLeafTips = R"(\[((?:(?:\[(?:\d+,\s\d+)\])(?:,\s)?)+)\])";
	// Regex to match the list of junctions of a plant
	const std::string matchJunctions = R"(\[((?:(?:\[(?:\d+,\s\d+)\])(?:,\s)?)*)\])";
	// Regex to match a space between sequences
	const std::string matchSpace = R"(\s)";
	// Regex to match a full line
	const std::regex matchLeafTipLine(matchPlantName + matchSpace +
		                              matchLeafTips + matchSpace + matchJunctions);
	// Regex to match a 2D point
	const std::regex matchPoint(R"(\[(\d+),\s(\d+)\])");

	// Plants indexed by name
	std::unordered_map<std::string, PlantPhenotypePoints> plants;

	std::ifstream file(filename);

	if (file.is_open())
	{
		std::string line;
		while (std::getline(file, line))
		{
			// Match the current line and extract information
			std::smatch matchesInLine;
			if (std::regex_match(line, matchesInLine, matchLeafTipLine) && matchesInLine.size() == 5)
			{
				const std::string& plantName = matchesInLine[1];
				const std::string& plantView = matchesInLine[2];
				const std::string& leafTipPoints = matchesInLine[3];
				const std::string& junctionPoints = matchesInLine[4];
				SetOfVec2 points;

				// Based on which phenotype we want to read, read leaf tips or junctions
				std::string pointsString = (type == PlantPhenotypePointType::LeafTip) ? leafTipPoints : junctionPoints;
				std::smatch matchesInPoints;
				while (std::regex_search(pointsString, matchesInPoints, matchPoint) && matchesInPoints.size() == 3)
				{
					// Split the point match into two integers
					const int x = std::stoi(matchesInPoints[1]);
					const int y = std::stoi(matchesInPoints[2]);
					points.emplace_back(x, y);
					// Analyze the rest of the points string
					pointsString = matchesInPoints.suffix().str();
				}

				// If points were found, add them to the corresponding plant
				if (!points.empty())
				{
					// Find the plant, add the view
					auto itPlant = plants.find(plantName);

					if (itPlant == plants.end())
					{
						// The plant does not already exist, we add it
						std::tie(itPlant, std::ignore) = plants.emplace(plantName, PlantPhenotypePoints(plantName));
					}

					// The plant already exists, we only add the points from the view
					itPlant->second.addPointsFromView(plantView, points);
				}
			}
		}

		file.close();
	}

	// Convert the map of plants to a list of plants
	std::vector<PlantPhenotypePoints> plantList(plants.size());
	std::transform(plants.begin(), plants.end(), plantList.begin(),
		[](auto pair)
		{
			return pair.second;
		});

	return plantList;
}

void flipYAxisOnAllPlants(const PhenotypingSetup& setup, std::vector<PlantPhenotypePoints>& plants)
{
	for (auto& plant : plants)
	{
		plant.flipYAxis(setup.imageHeight());
	}
}

void discardViewOnAllPlants(const std::string& viewName, std::vector<PlantPhenotypePoints>& plants)
{
	for (auto& plant : plants)
	{
		plant.discardView(viewName);
	}
}

void keepOnlyPlantsWhoseNameContains(const std::string& plantName, std::vector<PlantPhenotypePoints>& plants)
{
	auto itPlants = plants.begin();

	while (itPlants != plants.end())
	{
		// If the plant name does not contain the string "plantName"
		if (itPlants->plantName().find(plantName) == std::string::npos)
		{
			// Erase from both vectors
			itPlants = plants.erase(itPlants);
		}
		else
		{
			++itPlants;
		}
	}
}

void apply90DegreesRotationToViews(const std::string& viewName,
                                   const PhenotypingSetup& setup,
                                   std::vector<PlantPhenotypePoints>& plants)
{
	for (auto& plant : plants)
	{
		plant.apply90DegreesRotationToView(viewName, setup.topViewRotation(), setup.imageWidth(), setup.imageHeight());
	}
}

void convertCalibrationOutputToCsv(const std::string& inputFilename, const std::string& outputFilename)
{
	// Conversion between the name of the view in the calibration output and the name of the view in the CSV file
	const std::unordered_map<std::string, std::string> viewName = {
		{"0_0_0", "_Vis_" + ViewSv0 + ".png"},
		{"0_36_0", "_Vis_" + ViewSv36 + ".png"},
		{"0_72_0", "_Vis_" + ViewSv72 + ".png"},
		{"0_108_0", "_Vis_" + ViewSv108 + ".png"},
		{"0_144_0", "_Vis_" + ViewSv144 + ".png"},
		{"0_216_0", "_Vis_" + ViewSv216 + ".png"},
		{"0_288_0", "_Vis_" + ViewSv288 + ".png"},
		{"0_324_0", "_Vis_" + ViewSv324 + ".png"},
		{"top_0_90_0", "_Vis_" + ViewTv90 + ".png"},
	};

	// Prefix to the name of the plant
	const std::string plantPrefix = "4-9-18_Schnable_";
	// Regex to match the name of the plant in the format of the output of the calibration script
	const std::string matchPlantName = R"((.+)_\d{2}-\d{2}-\d{2}_\d+_((?:top_)?0_\d+_0))";
	// Regex to match a float number
	const std::string matchFloatNumber = R"(([-+]?[0-9]*\.?[0-9]+))";
	// Regex to match a space between sequences
	const std::string matchSpace = R"(\s)";
	// Regex to match a full line
	const std::regex matchTranslationLine(plantPrefix + matchPlantName + matchSpace +
		                                  matchFloatNumber + matchSpace + matchFloatNumber);

	std::ifstream inputFile(inputFilename);
	std::ofstream outputFile(outputFilename);

	if (!inputFile.is_open() || !outputFile.is_open())
	{
		return;
	}

	std::string line;
	while (std::getline(inputFile, line))
	{
		// Match the current line and extract information
		std::smatch matchesInLine;
		if (std::regex_match(line, matchesInLine, matchTranslationLine) && matchesInLine.size() == 5)
		{
			const std::string plantName = matchesInLine[1];
			std::string plantView = matchesInLine[2];
			const std::string translationX = matchesInLine[3];
			const std::string translationY = matchesInLine[4];

			// Convert the plant view to the right format if a translation rule exists
			if (viewName.count(plantView) > 0)
			{
				plantView = viewName.at(plantView);
			}

			outputFile << plantName << plantView << "\t" << translationX << "\t" << translationY << "\n";
		}
	}

	inputFile.close();
	outputFile.close();
}

void keepOnlyPlantsWithMultipleViews(std::vector<PlantPhenotypePoints>& plants)
{
	// Send plants without all views at the end of the list
	const auto endIt = std::remove_if(plants.begin(),
	                                  plants.end(),
	                                  [](const PlantPhenotypePoints& p) -> bool
	                                  {
		                                  return p.getAllViews().size() <= 1;
	                                  });

	// Remove plants at the end of the list
	plants.erase(endIt, plants.end());
}

void keepOnlyPlantsWithAllViews(const std::vector<std::string>& viewNames, std::vector<PlantPhenotypePoints>& plants)
{
	// Send plants without all views at the end of the list
	const auto endIt = std::remove_if(plants.begin(),
	                                  plants.end(),
	                                  [&viewNames](const PlantPhenotypePoints& p) -> bool
	                                  {
		                                  return !p.hasAllViews(viewNames);
	                                  });

	// Remove plants at the end of the list
	plants.erase(endIt, plants.end());
}

void discardPointsRandomly(unsigned int seed, double probability, std::vector<PlantPhenotypePoints>& plants)
{
	std::default_random_engine generator(seed);

	for (auto& plant : plants)
	{
		plant.discardPointsRandomly(generator, probability);
	}
}

void addPointsRandomly(const PhenotypingSetup& setup,
                       unsigned int seed,
                       double probability,
                       std::vector<PlantPhenotypePoints>& plants)
{
	std::default_random_engine generator(seed);

	for (auto& plant : plants)
	{
		plant.addPointsRandomly(setup, generator, probability);
	}
}

void clampRaysWithPhenotypingSetup(
	const PhenotypingSetup& setup,
	const std::vector<std::string>& viewNames,
	SetsOfRays& rays)
{
	for (unsigned int c = 0; c < viewNames.size(); c++)
	{
		// If it is not a top view, clamp the ray
		if (viewNames[c] != ViewTv90)
		{
			for (auto& ray : rays[c])
			{
				ray.clampRay(setup.distanceToPlant() - setup.radiusPlant(),
				             setup.distanceToPlant() + setup.radiusPlant());
			}
		}
	}
}

std::tuple<SetOfVec3, SetsOfCorrespondences>
triangulatePhenotypePoints(
	const PhenotypingSetup& setup,
	const PlantPhenotypePoints& plantPoints,
	double thresholdNoPair)
{
	// Get the list of views from which the plant was annotated, order does matter
	const auto viewNames = plantPoints.getAllViews();
	// Get the 2D points in the same order as the views
	const auto points = plantPoints.pointsFromViews(viewNames);
	// Get the cameras in the same order as the views
	const auto cameras = setup.camerasFromViews(viewNames);
	// Un-project annotated 2D points and get 3D rays
	auto rays = computeRays(cameras, points);
	// Clamp all rays according to the phenotyping setup
	clampRaysWithPhenotypingSetup(setup, viewNames, rays);
	// Triangulate the 3D points
	auto [triangulatedPoints3D, setsOfCorrespondences, viewOrder] = matchRaysAndTriangulate(cameras, points, rays, thresholdNoPair);
	// If some of the points are triangulated with only one ray, we remove them because it's probably a failed match
	removePointsFromSingleRays(triangulatedPoints3D, setsOfCorrespondences);
	// Sort the set of rays to make it uniquely identifiable even if it has been permuted
	sortSetsOfCorrespondences(setsOfCorrespondences);

	spdlog::trace("View order for plant {}: {}", plantPoints.plantName(), orderedViewNamesToString(viewNames, viewOrder, points));

	return { triangulatedPoints3D, setsOfCorrespondences };
}

std::tuple<SetsOfVec2, std::vector<std::vector<int>>>
projectPhenotypePointsAndRetainMatches(
	const PhenotypingSetup& setup,
	const PlantPhenotypePoints& plantPoints,
	const SetOfVec3& triangulatedPoints3D,
	const SetsOfCorrespondences& setsOfCorrespondences)
{
	SetsOfVec2 triangulatedPoints;
	std::vector<std::vector<int>> triangulatedPointsMatches;

	// Re-project 3D triangulated points on views
	const auto viewNames = plantPoints.getAllViews();
	const auto cameras = setup.camerasFromViews(viewNames);
	for (int c = 0; c < static_cast<int>(cameras.size()); c++)
	{
		const auto& camera = cameras[c];
		// Project 3D triangulated points 
		SetOfVec2 viewTriangulatedPoints;
		viewTriangulatedPoints.reserve(triangulatedPoints3D.size());
		for (const auto& point : triangulatedPoints3D)
		{
			const auto point2D = camera.project(point);
			// point2D is a dvec3 but adding to the vector of glm::dvec2 removes the Z coordinate
			viewTriangulatedPoints.emplace_back(point2D);
		}
		// Add the projected points to the list of all projected points
		triangulatedPoints.emplace_back(std::move(viewTriangulatedPoints));

		// List matches between points
		std::vector<int> viewTriangulatedPointsMatches;
		viewTriangulatedPointsMatches.reserve(setsOfCorrespondences.size());
		for (const auto& setOfCorrespondences : setsOfCorrespondences)
		{
			// Look for the annotated point in the current view that corresponds to the current triangulated point
			int annotatedPointIndex = -1;
			// setOfCorrespondences is a list of (cameraIndex, pointIndex) for a triangulated point
			for (const auto& [cameraIndex, pointIndex] : setOfCorrespondences)
			{
				if (cameraIndex == c)
				{
					annotatedPointIndex = pointIndex;
					break;
				}
			}
			viewTriangulatedPointsMatches.push_back(annotatedPointIndex);
		}
		// Add the projected points to the list of all projected points
		triangulatedPointsMatches.emplace_back(std::move(viewTriangulatedPointsMatches));
	}

	return { triangulatedPoints, triangulatedPointsMatches };
}

void applyInverseTranslationsOnPhenotypePoints(
	const PlantImageTranslations& translations,
	const std::string& plantName,
	const std::vector<std::string>& viewNames,
	SetsOfVec2& plantPoints)
{
	for (unsigned int v = 0; v < viewNames.size(); v++)
	{
		auto translation = translations.getTranslationForPlantAndView(plantName, viewNames[v]);

		// Apply a translation of minus Y because the axis has been flipped previously
		translation.y = -translation.y;

		for (auto& viewPoint : plantPoints[v])
		{
			viewPoint -= translation;
		}
	}
}

bool drawPointsInImage(const std::string& filename,
                       const std::string& backgroundImage,
                       const SetOfVec2& points)
{
	auto image = cv::imread(backgroundImage);

	// Could not open the background image
	if (image.data == nullptr)
	{
		return false;
	}

	// The radius of points is 0.5% of the largest dimension of the image
	const auto radius = std::max(image.rows, image.cols) / 200;
	const auto thickness = radius / 2;

	for (const auto& point : points)
	{
		cv::circle(image,
		           cv::Point2d(point.x, static_cast<double>(image.rows) - point.y),
		           radius,
		           cv::Scalar(255, 0, 0),
		           thickness);
	}

	return cv::imwrite(filename, image);
}

bool drawPointsInImageWithMatches(
	const std::string& filename,
	const std::string& backgroundImage,
	const SetOfVec2& annotationPoints,
	const SetOfVec2& projectionPoints,
	const std::vector<int>& projectionMatches)
{
	auto image = cv::imread(backgroundImage);

	// Could not open the background image
	if (image.data == nullptr)
	{
		return false;
	}

	// The radius of points is 0.5% of the largest dimension of the image
	const auto radius = std::max(image.rows, image.cols) / 200;
	const auto thicknessPrediction = radius / 2;
	// The radius of the annotation is slightly larger than the prediction
	// to make sure it is visible when both are at the same location
	const auto thicknessAnnotation = thicknessPrediction + 1;

	for (const auto& point : annotationPoints)
	{
		cv::circle(image,
		           cv::Point2d(point.x, static_cast<double>(image.rows) - point.y),
		           radius,
		           cv::Scalar(0, 0, 255),
		           thicknessAnnotation);
	}

	for (const auto& point : projectionPoints)
	{
		cv::circle(image,
		           cv::Point2d(point.x, static_cast<double>(image.rows) - point.y),
		           radius,
		           cv::Scalar(255, 0, 0),
		           thicknessPrediction);
	}

	// Display matches with lines between points
	for (unsigned int i = 0; i < projectionMatches.size(); i++)
	{
		// Draw a line between projection i and annotation match[i]
		const auto indexMatch = projectionMatches[i];

		if (indexMatch >= 0 && indexMatch < static_cast<int>(annotationPoints.size()))
		{
			cv::Point2d pt1(projectionPoints[i].x, static_cast<double>(image.rows) - projectionPoints[i].y);
			cv::Point2d pt2(annotationPoints[indexMatch].x, static_cast<double>(image.rows) - annotationPoints[indexMatch].y);

			cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), thicknessPrediction);
		}
	}

	return cv::imwrite(filename, image);
}

bool exportPlantStatsToCsv(
	const std::string& filename,
	const PhenotypingSetup& setup,
	const std::vector<PlantPhenotypePoints>& plants)
{
	std::ofstream file(filename, std::ofstream::out);

	if (!file.is_open())
	{
		return false;
	}

	const auto& views = setup.views();

	// Header
	for (const auto& view : views)
	{
		file << "\t" << view;
	}
	file << "\n";

	// Plants
	for (const auto& plant : plants)
	{
		file << plant.plantName();

		for (const auto& view : views)
		{
			file << "\t" << plant.pointsFromView(view).size();
		}

		file << "\n";
	}

	file.close();

	return true;
}

bool exportPlantPointsToCsv(const std::string& filename, const std::vector<PlantPhenotypePoints>& plants)
{
	std::ofstream file(filename, std::ofstream::out);

	if (!file.is_open())
	{
		return false;
	}

	for (const auto& plant : plants)
	{
		file << plant.exportToCsv();
	}

	file.close();

	return true;
}

bool exportNumberLeavesToCsv(const std::string& filename, const std::vector<std::pair<std::string, int>>& numberLeafTips)
{
	std::ofstream file(filename, std::ofstream::out);

	if (!file.is_open())
	{
		return false;
	}

	for (const auto& [plantName, number] : numberLeafTips)
	{
		file << plantName << "\t" << number << "\n";
	}

	file.close();

	return true;
}

bool exportPositionLeavesToTxt(
	const std::string& filename,
	const std::vector<PlantPhenotypePoints>& plants,
	const SetsOfVec3& points3d)
{
	assert(plants.size() == points3d.size());

	std::ofstream file(filename, std::ofstream::out);

	if (!file.is_open())
	{
		return false;
	}

	// Use full precision when writing floats
	file << std::setprecision(std::numeric_limits<double>::digits10 + 1);

	for (unsigned int i = 0; i < plants.size(); i++)
	{
		file << plants[i].plantName() << "\t" << points3d[i].size() << "\n";

		for (const auto& point3d : points3d[i])
		{
			file << point3d.x << " " << point3d.y << " " << point3d.z << std::endl;
		}
	}

	file.close();

	return true;
}
