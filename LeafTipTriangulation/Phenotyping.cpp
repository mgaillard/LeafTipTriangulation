#include "Phenotyping.h"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <regex>
#include <tuple>

#include <utils/warnoff.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <utils/warnon.h>

#include "RayMatching.h"

PhenotypingSetup::PhenotypingSetup(
	double imageWidth,
	double imageHeight,
	std::vector<std::string> views,
	std::vector<Camera> cameras) :
	m_imageWidth(imageWidth),
	m_imageHeight(imageHeight),
	m_views(std::move(views)),
	m_cameras(std::move(cameras))
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

void PlantPhenotypePoints::addPointsFromView(const std::string& viewName, const std::vector<glm::vec2>& points)
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

void PlantPhenotypePoints::applyTranslationToView(const std::string& viewName, const glm::vec2& translation)
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
			point.y = static_cast<float>(imageHeight) - point.y;
		}
	}
}

std::vector<glm::vec2> PlantPhenotypePoints::pointsFromView(const std::string& viewName) const
{
	if (m_points.count(viewName) > 0)
	{
		return m_points.at(viewName);
	}

	// An empty array because there are no points visible from this view
	return {};
}

std::vector<std::vector<glm::vec2>> PlantPhenotypePoints::pointsFromViews(const std::vector<std::string>& viewNames) const
{
	std::vector<std::vector<glm::vec2>> points;

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

glm::vec2 PlantImageTranslations::getTranslationForPlantAndView(
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
	std::vector<glm::vec2>& points)
{
	for (auto& point : points)
	{
		if (rotationDirection == RotationDirection::CounterclockwiseWithCanvas
			|| rotationDirection == RotationDirection::ClockwiseWithCanvas)
		{
			// Inverse translate the points from the center of rotation
			point.x -= static_cast<float>(imageHeight) / 2.f;
			point.y -= static_cast<float>(imageWidth) / 2.f;
		}
		else if (rotationDirection == RotationDirection::CounterclockwiseWithoutCanvas
			|| rotationDirection == RotationDirection::ClockwiseWithoutCanvas)
		{
			// Inverse translate the points from the center of rotation
			point.x -= static_cast<float>(imageWidth) / 2.f;
			point.y -= static_cast<float>(imageHeight) / 2.f;
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
			point.x += static_cast<float>(imageWidth) / 2.f;
			point.y += static_cast<float>(imageHeight) / 2.f;
		}
	}
}

RotationDirection loadTopViewRotationDirection(const std::string& rotationFile)
{
	auto direction = RotationDirection::Unknown;

	std::ifstream file(rotationFile);

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

		file.close();
	}

	return direction;
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

	if (viewName == "SV_0")
	{
		filename = "0_0_0";
	}
	else if (viewName == "SV_36")
	{
		filename = "0_36_0";
	}
	else if (viewName == "SV_72")
	{
		filename = "0_72_0";
	}
	else if (viewName == "SV_108")
	{
		filename = "0_108_0";
	}
	else if (viewName == "SV_144")
	{
		filename = "0_144_0";
	}
	else if (viewName == "SV_216")
	{
		filename = "0_216_0";
	}
	else if (viewName == "SV_252")
	{
		filename = "0_252_0";
	}
	else if (viewName == "SV_288")
	{
		filename = "0_288_0";
	}
	else if (viewName == "SV_324")
	{
		filename = "0_324_0";
	}
	else if (viewName == "TV_90")
	{
		filename = "top_0_90_0";
	}
	else
	{
		// By default the view name is "default", which will most likely not exist and trigger an error.
		filename = "default";
	}

	return filename + '.' + extension;
}

PhenotypingSetup loadPhenotypingSetup(const std::string& cameraFolder)
{
	const std::array<std::pair<std::string, std::string>, 10> viewCameraNames = {{
		{"SV_0", cameraFolder + "camera_0_0_0.txt"},
		{"SV_36", cameraFolder + "camera_0_36_0.txt"},
		{"SV_72", cameraFolder + "camera_0_72_0.txt"},
		{"SV_108", cameraFolder + "camera_0_108_0.txt"},
		{"SV_144", cameraFolder + "camera_0_144_0.txt"},
		{"SV_216", cameraFolder + "camera_0_216_0.txt"},
		{"SV_252", cameraFolder + "camera_0_252_0.txt"},
		{"SV_288", cameraFolder + "camera_0_288_0.txt"},
		{"SV_324", cameraFolder + "camera_0_324_0.txt"},
		{"TV_90", cameraFolder + "camera_top_0_90_0.txt"}
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
			cameraFiles.push_back(viewCameraName.second);
		}
	}

	const auto cameras = loadCamerasFromFiles(cameraFiles);

	// Read the resolution from the first image
	const auto imageWidth = static_cast<double>(cameras[0].viewport().z);
	const auto imageHeight = static_cast<double>(cameras[0].viewport().w);

	return {
		imageWidth,
		imageHeight,
		views,
		cameras
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
				std::vector<glm::vec2> points;

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

void apply90DegreesRotationToViews(const std::string& viewName,
                                   const PhenotypingSetup& setup,
                                   const RotationDirection& rotationDirection,
                                   std::vector<PlantPhenotypePoints>& plants)
{
	for (auto& plant : plants)
	{
		plant.apply90DegreesRotationToView(viewName, rotationDirection, setup.imageWidth(), setup.imageHeight());
	}
}

void convertCalibrationOutputToCsv(const std::string& inputFilename, const std::string& outputFilename)
{
	// Conversion between the name of the view in the calibration output and the name of the view in the CSV file
	const std::unordered_map<std::string, std::string> viewName = {
		{"0_0_0", "_Vis_SV_0.png"},
		{"0_36_0", "_Vis_SV_36.png"},
		{"0_72_0", "_Vis_SV_72.png"},
		{"0_108_0", "_Vis_SV_108.png"},
		{"0_144_0", "_Vis_SV_144.png"},
		{"0_216_0", "_Vis_SV_216.png"},
		{"0_288_0", "_Vis_SV_288.png"},
		{"0_324_0", "_Vis_SV_324.png"},
		{"top_0_90_0", "_Vis_TV_90.png"},
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

std::tuple<std::vector<glm::vec3>, std::vector<std::vector<std::pair<int, int>>>>
triangulatePhenotypePoints(const PhenotypingSetup& setup, const PlantPhenotypePoints& plantPoints)
{
	// Get the list of views from which the plant was annotated, order does matter
	const auto viewNames = plantPoints.getAllViews();
	// Get the 2D points in the same order as the views
	const auto points = plantPoints.pointsFromViews(viewNames);
	// Get the cameras in the same order as the views
	const auto cameras = setup.camerasFromViews(viewNames);
	// Un-project annotated 2D points and get 3D rays
	const auto rays = computeRays(cameras, points);
	// Triangulate the 3D points
	return matchRaysAndTriangulate(cameras, points, rays);
}

std::tuple<std::vector<std::vector<glm::vec2>>, std::vector<std::vector<int>>>
projectPhenotypePointsAndRetainMatches(
	const PhenotypingSetup& setup,
	const PlantPhenotypePoints& plantPoints,
	const std::vector<glm::vec3>& triangulatedPoints3D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	std::vector<std::vector<glm::vec2>> triangulatedPoints;
	std::vector<std::vector<int>> triangulatedPointsMatches;

	// Re-project 3D triangulated points on views
	const auto viewNames = plantPoints.getAllViews();
	const auto cameras = setup.camerasFromViews(viewNames);
	for (int c = 0; c < static_cast<int>(cameras.size()); c++)
	{
		const auto& camera = cameras[c];
		// Project 3D triangulated points 
		std::vector<glm::vec2> viewTriangulatedPoints;
		viewTriangulatedPoints.reserve(triangulatedPoints3D.size());
		for (const auto& point : triangulatedPoints3D)
		{
			const auto point2D = camera.project(point);
			// point2D is a vec3 but adding to the vector of glm::vec2 removes the Z coordinate
			viewTriangulatedPoints.emplace_back(point2D);
		}
		// Add the projected points to the list of all projected points
		triangulatedPoints.emplace_back(std::move(viewTriangulatedPoints));

		// List matches between points
		std::vector<int> viewTriangulatedPointsMatches;
		viewTriangulatedPointsMatches.reserve(setsOfRays.size());
		for (const auto& setOfRays : setsOfRays)
		{
			// Look for the annotated point in the current view that corresponds to the current triangulated point
			int annotatedPointIndex = -1;
			// setOfRays is a list of (cameraIndex, pointIndex) for a triangulated point
			for (const auto& [cameraIndex, pointIndex] : setOfRays)
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
	std::vector<std::vector<glm::vec2>>& plantPoints)
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
                       const std::vector<glm::vec2>& points)
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
		           cv::Point2f(point.x, static_cast<float>(image.rows) - point.y),
		           radius,
		           cv::Scalar(255, 0, 0),
		           thickness);
	}

	return cv::imwrite(filename, image);
}

bool drawPointsInImageWithMatches(
	const std::string& filename,
	const std::string& backgroundImage,
	const std::vector<glm::vec2>& annotationPoints,
	const std::vector<glm::vec2>& projectionPoints,
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
	const auto thickness = radius / 2;

	for (const auto& point : annotationPoints)
	{
		cv::circle(image,
		           cv::Point2f(point.x, static_cast<float>(image.rows) - point.y),
		           radius,
		           cv::Scalar(0, 0, 255),
		           thickness);
	}

	for (const auto& point : projectionPoints)
	{
		cv::circle(image,
		           cv::Point2f(point.x, static_cast<float>(image.rows) - point.y),
		           radius,
		           cv::Scalar(255, 0, 0),
		           thickness);
	}

	// Display matches with lines between points
	for (unsigned int i = 0; i < projectionMatches.size(); i++)
	{
		// Draw a line between projection i and annotation match[i]
		const auto indexMatch = projectionMatches[i];

		if (indexMatch >= 0 && indexMatch < annotationPoints.size())
		{
			cv::Point2f pt1(projectionPoints[i].x, static_cast<float>(image.rows) - projectionPoints[i].y);
			cv::Point2f pt2(annotationPoints[indexMatch].x, static_cast<float>(image.rows) - annotationPoints[indexMatch].y);

			cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), thickness);
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
