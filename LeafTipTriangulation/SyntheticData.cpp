#include "SyntheticData.h"

#include <iostream>

#include <glm/gtc/random.hpp>
#include <glm/gtx/closest_point.hpp>

#include <dlib/optimization/max_cost_assignment.h>

std::vector<glm::vec3> generatePointsInSphere(int n, float radius)
{
	std::vector<glm::vec3> points3D;

	points3D.reserve(n);
	for (int i = 0; i < n; i++)
	{
		points3D.push_back(glm::ballRand(radius));
	}

	return points3D;
}

std::vector<Camera> generateCamerasOnSphere(int n, float radius)
{
	std::vector<Camera> cameras;

	cameras.reserve(n);
	for (int i = 0; i < n; i++)
	{
		// Generate the location of the camera
		const auto eye = glm::sphericalRand(radius);

		// Generate the up vector (unit vector around the eye, orthogonal to atToEye)
		glm::vec3 up;
		do
		{
			up = glm::cross(glm::sphericalRand(1.f), eye);
		} while (glm::length(up) <= 0.f);
		up = glm::normalize(up);

		cameras.push_back(Camera(eye, glm::vec3(0.f, 0.f, 0.f), up));
	}

	return cameras;
}

std::vector<std::vector<glm::vec2>> projectPoints(
	const std::vector<glm::vec3>& points,
	const std::vector<Camera>& cameras
)
{
	std::vector<std::vector<glm::vec2>> projected(cameras.size());

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		for (auto point : points)
		{
			const auto point2D = cameras[c].project(point);

			// Check depth: the point must be visible from the camera and not behind it
			assert(point2D.z > 0.f);

			projected[c].emplace_back(point2D);
		}
	}

	return projected;
}

std::vector<std::vector<glm::vec2>> addNoise(
	const std::vector<std::vector<glm::vec2>>& points,
	const std::vector<Camera>& cameras,
	float noiseStd)
{
	std::vector<std::vector<glm::vec2>> newPoints;

	newPoints.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
	{
		std::vector<glm::vec2> newCameraPoints;

		newCameraPoints.reserve(points[i].size());
		for (const auto& point : points[i])
		{
			// Viewport for the current camera
			const auto& view = cameras[i].viewport();

			// 2D point + noise
			const auto x = glm::clamp(point.x + glm::gaussRand(0.f, noiseStd), view.x, view.z);
			const auto y = glm::clamp(point.y + glm::gaussRand(0.f, noiseStd), view.y, view.w);

			newCameraPoints.emplace_back(x, y);
		}

		newPoints.push_back(newCameraPoints);
	}

	return newPoints;
}

std::pair<std::vector<std::vector<glm::vec2>>, std::vector<std::vector<std::pair<int, int>>>> removePoints(
	const std::vector<std::vector<glm::vec2>>& points,
	float probabilityKeep)
{
	assert(!points.empty());
	assert(probabilityKeep >= 0.f && probabilityKeep <= 1.f);

	// Number of cameras on which each point is visible
	std::vector<int> visibility(points.front().size(), 0);

	// For each point retain the ground-truth correspondence
	std::vector<std::vector<std::pair<int, int>>> correspondences(points.front().size());

	// New array of camera points
	std::vector<std::vector<glm::vec2>> newPoints;

	newPoints.reserve(points.size());
	for (int c = 0; c < points.size(); c++)
	{
		const auto& cameraPoints = points[c];
		
		std::vector<glm::vec2> newCameraPoints;

		newCameraPoints.reserve(cameraPoints.size());
		for (unsigned int i = 0; i < cameraPoints.size(); i++)
		{
			const auto& point = cameraPoints[i];
			
			if (glm::linearRand(0.f, 1.f) <= probabilityKeep)
			{
				newCameraPoints.push_back(point);
				visibility[i]++;
				correspondences[i].emplace_back(c, newCameraPoints.size() - 1);
			}
		}

		newPoints.push_back(newCameraPoints);
	}

	// Check that each point is visible from at least two cameras
	for (unsigned int i = 0; i < visibility.size(); i++)
	{
		if (visibility[i] < 2)
		{
			std::cout << "Warning: point " << i
			          << " can't be reconstructed because it is visible from only one camera." << std::endl;
		}
	}

	// Maximum number of points seen by a camera
	const auto maximumVisibility = *std::max_element(visibility.begin(), visibility.end());
	if (maximumVisibility < points.size())
	{
		std::cout << "None of the cameras can see all points" << std::endl;
	}

	return { newPoints, correspondences };
}

bool checkUnProject(
	const std::vector<glm::vec3>& points,
	const std::vector<std::vector<Ray>>& rays)
{
	for (unsigned int i = 0; i < points.size(); i++)
	{
		for (unsigned int c = 0; c < rays.size(); c++)
		{
			const auto closestPoint = glm::closestPointOnLine(points[i],
				rays[c][i].origin,
				rays[c][i].at(10.f));

			if (glm::distance(points[i], closestPoint) >= 1e-3)
			{
				std::cout << "Warning, re-projection not accurate" << std::endl;
				return false;
			}
		}
	}

	return true;
}

void matchingTriangulatedPointsWithGroundTruth(
	const std::vector<glm::vec3>& triangulatedPoints3D,
	const std::vector<std::vector<std::pair<int, int>>>& setOfRays,
	const std::vector<glm::vec3>& points3D,
	const std::vector<std::vector<std::pair<int, int>>>& trueCorrespondences)
{
	assert(points3D.size() == triangulatedPoints3D.size());

	// TODO: handle the case where the number of points are not the same

	// Multiplier used to convert a floating point value to an integer value
	const float realToLongMultiplier = 1000.f;

	dlib::matrix<long> cost(points3D.size(), points3D.size());
	dlib::matrix<float> realCost(points3D.size(), points3D.size());

	for (int i = 0; i < points3D.size(); i++)
	{
		for (int j = 0; j < triangulatedPoints3D.size(); j++)
		{
			const auto dist = glm::distance(points3D[i], triangulatedPoints3D[j]);
			realCost(i, j) = dist;

			const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
			cost(i, j) = -integerDist;
		}
	}

	// Compute best assignment between pairs of points
	const auto assignment = dlib::max_cost_assignment(cost);

	// Compute statistics on the assignment
	int nbRightPoints = 0;
	int nbWrongPoints = 0;
	// TODO: Compute mean, standard-deviation
	float maximumDistance = 0.0f;
	float minimumDistance = std::numeric_limits<float>::max();
	for (unsigned int i = 0; i < assignment.size(); i++)
	{
		const auto groundTruthIndex = i;
		const auto triangulatedIndex = assignment[i];

		const auto dist = glm::distance(points3D[groundTruthIndex], triangulatedPoints3D[triangulatedIndex]);
		maximumDistance = std::max(maximumDistance, dist);
		minimumDistance = std::min(minimumDistance, dist);

		// Compare the correspondences
		// from trueCorrespondences[groundTruthIndex] with setOfRays[triangulatedIndex]
		if (trueCorrespondences[groundTruthIndex] == setOfRays[triangulatedIndex])
		{
			nbRightPoints++;
		}
		else
		{
			nbWrongPoints++;
		}

		// TODO: compute 
		
		// For each correspondence in setOfRays[triangulatedIndex]
		// If it is present in the ground truth => true positive
		// If it is not present in the ground truth => false positive

		// For each correspondence in trueCorrespondences[groundTruthIndex]
		// If it is not present in the ground truth => false negative
	}

	const auto rateWrongPoints = float(nbWrongPoints) / float(nbRightPoints + nbWrongPoints);

	// TODO: boolean for CSV output

	std::cout << "Minimum distance from triangulated to ground truth: " << minimumDistance << "\n"
			  << "Maximum distance from triangulated to ground truth: " << maximumDistance << "\n"
		      << "Assignment cost between the ground truth and triangulation: "
		      << dlib::assignment_cost(realCost, assignment) << "\n"
		      << "Number of incorrectly/correctly matched points : "
		      << nbWrongPoints << " / " << nbRightPoints << "\n"
		      << "Rate of incorrectly matched points : " << 100.f * rateWrongPoints << " %\n" << std::endl;
}

void checkCorrespondenceSetsOfRays(const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	int nbRightCorrespondences = 0;
	int nbWrongCorrespondences = 0;

	int nbRightPoints = 0;
	int nbWrongPoints = 0;
	
	// If rays have been generated, all rays of a point are associated to the same point on all cameras
	for (int i = 0; i < setsOfRays.size(); i++)
	{
		const auto& setOfRays = setsOfRays[i];

		bool pointIsCorrect = true;

		for (const auto& ray : setOfRays)
		{
			const auto& cameraIndex = ray.first;
			const auto& pointIndex = ray.second;

			if (pointIndex == i)
			{
				nbRightCorrespondences++;
			}
			else
			{
				nbWrongCorrespondences++;
				// If at least one wrong ray is associated to this point,
				// the point is wrongly triangulated
				pointIsCorrect = false;
			}
		}

		// Whether the point has successfully been triangulated or not
		if (pointIsCorrect)
		{
			nbRightPoints++;
		}
		else
		{
			nbWrongPoints++;
		}
	}

	// Compute rates of errors
	const auto totalCorrespondences = nbWrongCorrespondences + nbRightCorrespondences;
	const auto totalPoints = setsOfRays.size();
	
	const auto rateWrongCorrespondences = float(nbWrongCorrespondences) / float(totalCorrespondences);
	const auto rateWrongPoints = float(nbWrongPoints) / float(totalPoints);

	std::cout << "Number of incorrectly/correctly matched points : "
	          << nbWrongPoints << " / " << nbRightPoints << "\n"
			  << "Rate of incorrectly matched points : " << 100.f * rateWrongPoints << " %\n"
			  << "Number of incorrect/correct correspondences : "
	          << nbWrongCorrespondences << " / " << nbRightCorrespondences << "\n"
			  << "Rate of incorrect correspondences : " << 100.f * rateWrongCorrespondences << " %" << std::endl;
}
