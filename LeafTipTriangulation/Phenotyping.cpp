#include "Phenotyping.h"

#include <tuple>

#include "RayMatching.h"

PhenotypingSetup loadPhenotypingSetup()
{
	constexpr double imageWidth = 2454.0;
	constexpr double imageHeight = 2056.0;

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
		cameras
	};
}

std::vector<PlantLeafTips> readLeafTipsFromCSV(const std::string& filename)
{
	std::vector<PlantLeafTips> plants;

	(void)filename;
	// TODO: Read leaf tips from CSV file
	// TODO: Remember to flip Y axis

	return plants;
}

std::vector<glm::vec3> triangulateLeafTips(const PhenotypingSetup& setup, const PlantLeafTips& plantLeafTips)
{
	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(setup.cameras, plantLeafTips.tips);

	// Matching and triangulation of points
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	std::tie(triangulatedPoints3D, setsOfRays) = matchRaysAndTriangulate(setup.cameras, plantLeafTips.tips, rays);

	return triangulatedPoints3D;
}
