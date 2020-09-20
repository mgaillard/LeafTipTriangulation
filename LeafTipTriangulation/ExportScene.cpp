#include "ExportScene.h"

#include "OBJWriter.h"

bool exportSceneAsOBJ(
	const std::vector<glm::vec3>& points,
	const std::vector<std::vector<Ray>>& rays,
	const std::string& filename)
{
	OBJWriter objWriter;

	// Add 3D points
	for (const auto& point : points)
	{
		objWriter.addVertex(point);
	}

	// Add rays
	for (const auto& cameraRays : rays)
	{
		for (const auto& ray : cameraRays)
		{
			objWriter.addLine(ray.origin, ray.at(6.f));
		}
	}

	return objWriter.save(filename);
}

void exportSplitSceneAsOBJ(
	const std::vector<std::vector<Ray>>& rays,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	const std::vector<glm::vec3>& triangulatedPoints3D)
{
	// Draw the scene in OBJ for Debugging
	exportSceneAsOBJ(triangulatedPoints3D, {}, "points.obj");
	// Draw rays from each camera separately
	for (unsigned int i = 0; i < rays.size(); i++)
	{
		exportSceneAsOBJ({}, { rays[i] }, "camera_" + std::to_string(i) + ".obj");
	}
	// Draw rays from each point separately
	for (unsigned int i = 0; i < setsOfRays.size(); i++)
	{
		std::vector<Ray> raysToDisplay;
		for (const auto& pointRays : setsOfRays[i])
		{
			// Camera
			const auto& c = pointRays.first;
			// Ray
			const auto& r = pointRays.second;

			raysToDisplay.push_back(rays[c][r]);
		}

		exportSceneAsOBJ({}, { raysToDisplay }, "rays_" + std::to_string(i) + ".obj");
	}
}