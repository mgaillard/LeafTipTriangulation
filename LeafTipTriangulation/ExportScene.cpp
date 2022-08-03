#include "ExportScene.h"

#include "OBJWriter.h"

bool exportSceneAsOBJ(
	const SetOfVec3& points,
	const SetsOfRays& rays,
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
			if (ray.isClamped())
			{
				objWriter.addLine(ray.at(ray.start()), ray.at(ray.end()));
			}
			else
			{
				objWriter.addLine(ray.origin, ray.at(6.f));
			}
		}
	}

	return objWriter.save(filename);
}

void exportSplitSceneAsOBJ(
	const SetsOfRays& rays,
	const SetsOfCorrespondences& setsOfCorrespondences,
	const SetOfVec3& triangulatedPoints3D)
{
	// Draw the scene in OBJ for Debugging
	exportSceneAsOBJ(triangulatedPoints3D, {}, "points.obj");
	// Draw rays from each camera separately
	for (unsigned int i = 0; i < rays.size(); i++)
	{
		exportSceneAsOBJ({}, { rays[i] }, "camera_" + std::to_string(i) + ".obj");
	}
	// Draw rays from each point separately
	for (unsigned int i = 0; i < setsOfCorrespondences.size(); i++)
	{
		std::vector<Ray> raysToDisplay;
		for (const auto& pointRays : setsOfCorrespondences[i])
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