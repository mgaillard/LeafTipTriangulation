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

PlantLeafTips::PlantLeafTips(std::string plantName) : m_plantName(std::move(plantName))
{
	
}

const std::string& PlantLeafTips::plantName() const
{
	return m_plantName;
}

std::vector<std::string> PlantLeafTips::getAllViews() const
{
	std::vector<std::string> viewNames;

	for (const auto& [viewName, viewPoints] : m_points)
	{
		viewNames.push_back(viewName);
	}

	return viewNames;
}

bool PlantLeafTips::hasAllViews(const std::vector<std::string>& viewNames) const
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

void PlantLeafTips::addPointsFromView(const std::string& viewName, const std::vector<glm::vec2>& points)
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

void PlantLeafTips::applyTranslationToView(const std::string& viewName, const glm::vec2& translation)
{
	if (m_points.count(viewName) > 0)
	{
		for (auto& point : m_points[viewName])
		{
			point += translation;
		}
	}
}

void PlantLeafTips::apply90DegreesClockwiseRotationToView(
	const std::string& viewName,
	double imageWidth,
	double imageHeight)
{
	if (m_points.count(viewName) > 0)
	{
		for (auto& point : m_points[viewName])
		{
			// Inverse translate the points from the center of rotation
			point.x -= static_cast<float>(imageWidth) / 2.f;
			point.y -= static_cast<float>(imageHeight) / 2.f;

			// Apply the following transformation (rotation 90 deg clockwise)
			// x' = -y
			// y' = x
			std::swap(point.x, point.y);
			point.x = -point.x;

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

void PlantLeafTips::apply90DegreesCounterClockwiseRotationToView(
	const std::string& viewName,
	double imageWidth,
	double imageHeight)
{
	if (m_points.count(viewName) > 0)
	{
		for (auto& point : m_points[viewName])
		{
			// Inverse translate the points from the center of rotation
			point.x -= static_cast<float>(imageHeight) / 2.f;
			point.y -= static_cast<float>(imageWidth) / 2.f;
			
			// Apply the following transformation (rotation 90 deg counter-clockwise)
			// x' = y
			// y' = -x
			std::swap(point.x, point.y);
			point.y = -point.y;

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

void PlantLeafTips::flipYAxis(double imageHeight)
{
	for (auto& points : m_points)
	{
		for (auto& point : points.second)
		{
			point.y = static_cast<float>(imageHeight) - point.y;
		}
	}
}

std::vector<glm::vec2> PlantLeafTips::pointsFromView(const std::string& viewName) const
{
	if (m_points.count(viewName) > 0)
	{
		return m_points.at(viewName);
	}

	// An empty array because there are no points visible from this view
	return {};
}

std::vector<std::vector<glm::vec2>> PlantLeafTips::pointsFromViews(const std::vector<std::string>& viewNames) const
{
	std::vector<std::vector<glm::vec2>> points;

	for (const auto& viewName : viewNames)
	{
		points.push_back(pointsFromView(viewName));
	}

	return points;
}

RotationDirection loadTopViewRotationDirection(const std::string& rotationFile)
{
	auto direction = RotationDirection::Unknown;

	std::ifstream file(rotationFile);

	if (file.is_open())
	{
		std::string directionString;
		file >> directionString;

		if (directionString == "CW")
		{
			direction = RotationDirection::Clockwise;
		}
		else if (directionString == "CCW")
		{
			direction = RotationDirection::Counterclockwise;
		}

		file.close();
	}

	return direction;
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

std::vector<PlantLeafTips> readLeafTipsFromCSV(const std::string& filename)
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
	std::unordered_map<std::string, PlantLeafTips> plants;

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
				const std::string plantName = matchesInLine[1];
				const std::string plantView = matchesInLine[2];
				std::vector<glm::vec2> points;

				std::string pointsString = matchesInLine[3];
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
						std::tie(itPlant, std::ignore) = plants.emplace(plantName, PlantLeafTips(plantName));
					}

					// The plant already exists, we only add the points from the view
					itPlant->second.addPointsFromView(plantView, points);
				}
			}
		}

		file.close();
	}

	// Convert the map of plants to a list of plants
	std::vector<PlantLeafTips> plantList(plants.size());
	std::transform(plants.begin(), plants.end(), plantList.begin(),
		[](auto pair)
		{
			return pair.second;
		});

	return plantList;
}

void flipYAxisOnAllPlants(const PhenotypingSetup& setup, std::vector<PlantLeafTips>& plants)
{
	for (auto& plant : plants)
	{
		plant.flipYAxis(setup.imageHeight());
	}
}

void apply90DegreesRotationToViews(const std::string& viewName,
                                   const PhenotypingSetup& setup,
                                   const RotationDirection& rotationDirection,
                                   std::vector<PlantLeafTips>& plants)
{
	for (auto& plant : plants)
	{
		if (rotationDirection == RotationDirection::Clockwise)
		{
			plant.apply90DegreesClockwiseRotationToView(viewName, setup.imageWidth(), setup.imageHeight());
		}
		else if (rotationDirection == RotationDirection::Counterclockwise)
		{
			plant.apply90DegreesCounterClockwiseRotationToView(viewName, setup.imageWidth(), setup.imageHeight());
		}
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

void readAndApplyTranslationsFromCsv(const std::string& filename, std::vector<PlantLeafTips>& plants)
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

				// Find the plant whose name is plantName in the list
				auto plantIt = std::find_if(plants.begin(), plants.end(),
					[&plantName](const PlantLeafTips& plant)
					{
						return plant.plantName() == plantName;
					});

				// If the plant is found
				if (plantIt != plants.end())
				{
					plantIt->applyTranslationToView(plantView, { x, y });
				}
			}
		}

		file.close();
	}
}

void keepOnlyPlantsWithMultipleViews(std::vector<PlantLeafTips>& plants)
{
	// Send plants without all views at the end of the list
	const auto endIt = std::remove_if(plants.begin(),
	                                  plants.end(),
	                                  [](const PlantLeafTips& p) -> bool
	                                  {
		                                  return p.getAllViews().size() <= 1;
	                                  });

	// Remove plants at the end of the list
	plants.erase(endIt, plants.end());
}

void keepOnlyPlantsWithAllViews(const std::vector<std::string>& viewNames, std::vector<PlantLeafTips>& plants)
{
	// Send plants without all views at the end of the list
	const auto endIt = std::remove_if(plants.begin(),
	                                  plants.end(),
	                                  [&viewNames](const PlantLeafTips& p) -> bool
	                                  {
		                                  return !p.hasAllViews(viewNames);
	                                  });

	// Remove plants at the end of the list
	plants.erase(endIt, plants.end());
}

void discardPointsRandomly(unsigned seed, double probability, std::vector<PlantLeafTips>& plants)
{
	std::default_random_engine generator(seed);

	for (auto& plant : plants)
	{
		plant.discardPointsRandomly(generator, probability);
	}
}

std::vector<glm::vec3> triangulateLeafTips(const PhenotypingSetup& setup, const PlantLeafTips& plantLeafTips)
{
	// Get the list of views from which the plant was annotated, order does matter
	const auto viewNames = plantLeafTips.getAllViews();
	// Get the 2D points in the same order as the views
	const auto points = plantLeafTips.pointsFromViews(viewNames);
	// Get the cameras in the same order as the views
	const auto cameras = setup.camerasFromViews(viewNames);
	// Un-project annotated 2D points and get 3D rays
	const auto rays = computeRays(cameras, points);
	// Triangulate the 3D points
	const auto [triangulatedPoints3D, setsOfRays] = matchRaysAndTriangulate(cameras, points, rays);

	return triangulatedPoints3D;
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

	for (const auto& point : points)
	{
		cv::circle(image,
		           cv::Point2f(point.x, static_cast<float>(image.rows) - point.y),
		           5,
		           cv::Scalar(255, 0, 0),
		           2);
	}

	return cv::imwrite(filename, image);
}

bool exportPlantStatsToCsv(
	const std::string& filename,
	const PhenotypingSetup& setup,
	const std::vector<PlantLeafTips>& plants)
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
