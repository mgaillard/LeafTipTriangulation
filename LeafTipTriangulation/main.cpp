#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "Camera.h"
#include "SyntheticData.h"
#include "ExportScene.h"
#include "RayMatching.h"
#include "Triangulation.h"

void testWithSyntheticData()
{
	const int numberPoints3D = 8;
	const float spherePointsRadius = 1.f;
	const int numberCameras = 6;
	const float sphereCamerasRadius = 5.f;

	const auto points3D = generatePointsInSphere(numberPoints3D, spherePointsRadius);
	const auto cameras = generateCamerasOnSphere(numberCameras, sphereCamerasRadius);
	const auto projectedPoints2D = projectPoints(points3D, cameras);
	// Add noise and occlusion and shuffle points
	const auto points2D = removePoints(addNoise(projectedPoints2D, cameras, 2.f), 0.8f);
	
	const auto rays = computeRays(cameras, points2D);
	checkUnProject(points3D, rays);

	// Matching and triangulation of points
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	std::tie(triangulatedPoints3D, setsOfRays) = matchRaysAndTriangulate(cameras, points2D, rays);

	// Match the two sets of points and check the distance
	matchingTriangulatedPointsWithGroundTruth(points3D, triangulatedPoints3D);

	// TODO: Check sets of rays to make sure the matching is exact
	
	// Draw the scene in OBJ for Debugging
	exportSceneAsOBJ(points3D, rays, "scene.obj");
	// exportSplitSceneAsOBJ(rays, setsOfRays, triangulatedPoints3D);
}

void runOnRealData()
{
	const float imageWidth = 2454.0;
	const float imageHeight = 2056.0;

	const auto cameras = loadCamerasFromFiles({
		"cameras/camera_0.txt",
		"cameras/camera_72.txt",
		"cameras/camera_144.txt",
		"cameras/camera_216.txt",
		"cameras/camera_288.txt",
		"cameras/camera_top.txt"
		}, glm::vec2(imageWidth, imageHeight));

	// X axis is from left to right
	// Y axis is from bottom to top
	const std::vector<std::vector<glm::vec2>> points2D = {
		// Camera 0
		{
			{1009, 560},
			{813, 866},
			{883, 1268},
			{909, 1258},
			{1319, 1308},
			{1445, 1056},
			{1473, 716}
		},
		// Camera 72
		{
			{803, 884},
			{877, 576},
			{931, 1260},
			{1195, 1258},
			{1285, 1094},
			{1481, 1356},
			{1609, 1116},
			{1627, 686}
		},
		// Camera 144
		{
			{1348, 888},
			{1228, 1112},
			{1294, 1366},
			{1364, 1252},
			{1514, 1258}
		},
		// Camera 216
		{
			{815, 696},
			{841, 1112},
			{1189, 1096},
			{1007, 1364},
			{1449, 1280},
			{1627, 1266},
			{1731, 880},
			{1575, 584}
		},
		// Camera 288
		{
			{999, 711},
			{993, 1099},
			{1027, 1349},
			{1051, 1281},
			{1327, 1277},
			{1435, 865},
			{1463, 549}
		},
		// Camera top
		{
			{1600, 1492},
			{1572, 1525},
			{1373, 1491},
			{1220, 1113},
			{698, 1171},
			{670, 683},
			{633, 596},
			{944, 628}
		}
	};

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(cameras, points2D);

	// Matching and triangulation of points
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	std::tie(triangulatedPoints3D, setsOfRays) = matchRaysAndTriangulate(cameras, points2D, rays);

	// Export the scene
	exportSplitSceneAsOBJ(rays, setsOfRays, triangulatedPoints3D);
}

int main(int argc, char *argv[])
{
	runOnRealData();
	
    return 0;
}
