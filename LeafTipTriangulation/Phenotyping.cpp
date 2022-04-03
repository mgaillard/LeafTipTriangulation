#include "Phenotyping.h"

#include <fstream>
#include <iostream>
#include <regex>
#include <tuple>

#include "RayMatching.h"

PlantLeafTips::PlantLeafTips(std::string plantName) : m_plantName(std::move(plantName))
{
	
}

const std::string& PlantLeafTips::plantName() const
{
	return m_plantName;
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

std::vector<std::vector<glm::vec2>> PlantLeafTips::pointsFromViews(const std::vector<std::string>& viewNames) const
{
	std::vector<std::vector<glm::vec2>> points;

	for (const auto& viewName : viewNames)
	{
		if (m_points.count(viewName) > 0)
		{
			points.push_back(m_points.at(viewName));
		}
		else
		{
			// Add an empty array because there are no points visible from this view
			points.emplace_back();
		}
	}

	return points;
}

PhenotypingSetup loadPhenotypingSetup()
{
	constexpr double imageWidth = 2454.0;
	constexpr double imageHeight = 2056.0;

	const std::vector<std::string> views = {
		"SV_0",
		"SV_72",
		"SV_144",
		"SV_216",
		"SV_288",
		"TV_90",
	};

	const auto cameras = loadCamerasFromFiles({
		"cameras/camera_0.txt",
		"cameras/camera_72.txt",
		"cameras/camera_144.txt",
		"cameras/camera_216.txt",
		"cameras/camera_288.txt",
		"cameras/camera_top.txt"
		}, glm::vec2(imageWidth, imageHeight));

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
	const std::string matchJunctions = R"(\[((?:(?:\[(?:\d+,\s\d+)\])(?:,\s)?)+)\])";
	// Regex to match a space between sequences
	const std::string matchSpace = R"(\s)";
	// Regex to match a full line
	const std::regex matchLeafTipLine(matchPlantName + matchSpace + matchLeafTips + matchSpace + matchJunctions);
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

std::vector<glm::vec3> triangulateLeafTips(const PhenotypingSetup& setup, const PlantLeafTips& plantLeafTips)
{
	const auto points = plantLeafTips.pointsFromViews(setup.views);

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(setup.cameras, points);

	auto [triangulatedPoints3D, setsOfRays] = matchRaysAndTriangulate(setup.cameras, points, rays);

	return triangulatedPoints3D;
}
