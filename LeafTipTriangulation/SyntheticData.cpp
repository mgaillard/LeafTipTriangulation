#include "SyntheticData.h"

#include <iostream>
#include <numeric>

#include <utils/warnoff.h>
#include <glm/gtc/random.hpp>
#include <glm/gtx/closest_point.hpp>

#include <dlib/optimization/max_cost_assignment.h>
#include <utils/warnon.h>

#include "Triangulation.h"

std::vector<glm::dvec3> generatePointsInSphere(int n, double radius)
{
	std::vector<glm::dvec3> points3d;

	points3d.reserve(n);
	for (int i = 0; i < n; i++)
	{
		points3d.push_back(glm::ballRand(radius));
	}

	return points3d;
}

std::vector<Camera> generateCamerasOnSphere(int n, double radius)
{
	std::vector<Camera> cameras;

	cameras.reserve(n);
	for (int i = 0; i < n; i++)
	{
		// Generate the location of the camera
		const auto eye = glm::sphericalRand(radius);

		// Generate the up vector (unit vector around the eye, orthogonal to atToEye)
		glm::dvec3 up;
		do
		{
			up = glm::cross(glm::sphericalRand(1.0), eye);
		} while (glm::length(up) <= 0.0);
		up = glm::normalize(up);

		cameras.emplace_back(eye, glm::dvec3(0.f, 0.f, 0.f), up);
	}

	return cameras;
}

std::vector<std::vector<glm::dvec2>> projectPoints(
	const std::vector<glm::dvec3>& points3d,
	const std::vector<Camera>& cameras
)
{
	std::vector<std::vector<glm::dvec2>> projected(cameras.size());

	for (unsigned int c = 0; c < cameras.size(); c++)
	{
		for (const auto& point : points3d)
		{
			const auto point2d = cameras[c].project(point);

			// Check depth: the point must be visible from the camera and not behind it
			assert(point2d.z > 0.0);

			projected[c].emplace_back(point2d);
		}
	}

	return projected;
}

std::vector<std::vector<glm::dvec2>> addNoise(
	const std::vector<std::vector<glm::dvec2>>& points,
	const std::vector<Camera>& cameras,
	double noiseStd)
{
	std::vector<std::vector<glm::dvec2>> newPoints;

	newPoints.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
	{
		std::vector<glm::dvec2> newCameraPoints;

		newCameraPoints.reserve(points[i].size());
		for (const auto& point : points[i])
		{
			// Viewport for the current camera
			const auto& view = cameras[i].viewport();

			// 2D point + noise
			const auto x = glm::clamp(point.x + glm::gaussRand(0.0, noiseStd), view.x, view.z);
			const auto y = glm::clamp(point.y + glm::gaussRand(0.0, noiseStd), view.y, view.w);

			newCameraPoints.emplace_back(x, y);
		}

		newPoints.push_back(newCameraPoints);
	}

	return newPoints;
}

std::pair<std::vector<std::vector<glm::dvec2>>, std::vector<std::vector<std::pair<int, int>>>> removePoints(
	const std::vector<std::vector<glm::dvec2>>& points,
	double probabilityKeep,
	bool verbose)
{
	assert(!points.empty());
	assert(probabilityKeep >= 0.f && probabilityKeep <= 1.f);

	// Number of cameras on which each point is visible
	std::vector<int> visibility(points.front().size(), 0);

	// For each point retain the ground-truth correspondence
	std::vector<std::vector<std::pair<int, int>>> correspondences(points.front().size());

	// New array of camera points
	std::vector<std::vector<glm::dvec2>> newPoints;

	newPoints.reserve(points.size());
	for (unsigned int c = 0; c < points.size(); c++)
	{
		const auto& cameraPoints = points[c];
		
		std::vector<glm::dvec2> newCameraPoints;

		newCameraPoints.reserve(cameraPoints.size());
		for (unsigned int i = 0; i < cameraPoints.size(); i++)
		{
			const auto& point = cameraPoints[i];
			
			if (glm::linearRand(0.0, 1.0) <= probabilityKeep)
			{
				newCameraPoints.push_back(point);
				visibility[i]++;
				const auto lastCameraIndex = static_cast<int>(newCameraPoints.size()) - 1;
				correspondences[i].emplace_back(c, lastCameraIndex);
			}
		}

		newPoints.push_back(newCameraPoints);
	}

	// True if all points are seen from at least two views
	bool outputIsValid = true;

	// Maximum number of points seen by a camera
	const auto maximumVisibility = *std::max_element(visibility.begin(), visibility.end());
	if (maximumVisibility < static_cast<int>(points.size()))
	{
		// outputIsValid = false;
		if (verbose)
		{
			std::cout << "None of the cameras can see all points" << std::endl;
		}
	}

	// Check that each point is visible from at least two cameras
	for (unsigned int i = 0; i < visibility.size(); i++)
	{
		if (visibility[i] < 2)
		{
			outputIsValid = false;

			if (verbose)
			{
				std::cout << "Warning: point " << i
					      << " can't be reconstructed because it is visible from only one camera." << std::endl;
			}
		}
	}

	if (outputIsValid)
	{
		return { newPoints, correspondences };
	}
	else
	{
		// If the output is not valid, simply output empty arrays
		return { {}, {} };
	}
}

bool checkUnProject(
	const std::vector<glm::dvec3>& points,
	const std::vector<std::vector<Ray>>& rays)
{
	for (unsigned int i = 0; i < points.size(); i++)
	{
		for (unsigned int c = 0; c < rays.size(); c++)
		{
			const auto closestPoint = glm::closestPointOnLine(points[i],
			                                                  rays[c][i].origin,
			                                                  rays[c][i].at(10.0));

			if (glm::distance(points[i], closestPoint) >= 1e-3)
			{
				std::cout << "Warning, re-projection not accurate" << std::endl;
				return false;
			}
		}
	}

	return true;
}

GroundTruthMatchingResult matchingTriangulatedPointsWithGroundTruth(
	const std::vector<Camera>& cameras,
	const std::vector<std::vector<glm::dvec2>>& points2d,
	const std::vector<glm::dvec3>& points3d,
	const std::vector<std::vector<std::pair<int, int>>>& trueCorrespondences,
	const std::vector<glm::dvec3>& triangulatedPoints3D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	// Multiplier used to convert a floating point value to an integer value
	const double realToLongMultiplier = 1000.0;

	// TODO: better matching and see if it changes results in stats

	const int costRows = static_cast<int>(points3d.size());
	const int costCols = static_cast<int>(triangulatedPoints3D.size());
	const int costSize = std::max(costRows, costCols);

	dlib::matrix<long> cost(costSize, costSize);
	dlib::matrix<double> realCost(costSize, costSize);
	
	for (int i = 0; i < costSize; i++)
	{
		for (int j = 0; j < costSize; j++)
		{
			if (i < costRows && j < costCols)
			{
				const auto dist = glm::distance(points3d[i], triangulatedPoints3D[j]);
				realCost(i, j) = dist;

				const auto integerDist = static_cast<long>(std::round(dist * realToLongMultiplier));
				cost(i, j) = -integerDist;
			}
			else
			{
				realCost(i, j) = 0.0;
				// TODO: replace with a better constant
				cost(i, j) = -1000000;
			}
		}
	}

	// Compute best assignment between pairs of points
	const auto assignment = dlib::max_cost_assignment(cost);

	GroundTruthMatchingResult result;
	result.nbPointsTriangulated = static_cast<int>(triangulatedPoints3D.size());
	// Number of points that were missed by the algorithm
	result.nbPointsMissed = std::max(0, costRows - costCols);
	// Number of points that were added by the algorithm even if they should not be present
	result.nbPointsFalsePositive = std::max(0, costCols - costRows);

	// Compute the reprojection error with the true correspondences
	// This is a lower bound on the error if the algorithm managed to find the true correspondences
	// What can happen though is that the algorithm finds correspondences that have even a lower reprojection error
	// This happens when the noise added to the 3D points changes what corresponds to the optimum set of correspondences
	// In this case, the algorithm finds "better correspondences" even if they are not the true correspondences
	result.trueReprojectionError = reprojectionErrorManyPointsFromMultipleViews(cameras, points2d, trueCorrespondences, points3d);
	result.measuredReprojectionError = reprojectionErrorManyPointsFromMultipleViews(cameras, points2d, setsOfRays, triangulatedPoints3D);

	for (int i = 0; i < costRows; i++)
	{
		// If it is a true 3D point
		if (i < costRows)
		{
			const auto groundTruthIndex = i;

			// If the true point has been successfully matched with a triangulated point
			if (assignment[i] < costCols)
			{
				// The true point i does have an equivalent triangulated assignment[i]
				
				const auto triangulatedIndex = assignment[i];

				result.nbPointsSuccessful++;

				const auto dist = glm::distance(points3d[groundTruthIndex], triangulatedPoints3D[triangulatedIndex]);
				result.distances.push_back(dist);

				// Compare the correspondences
				// from trueCorrespondences[groundTruthIndex] with setsOfRays[triangulatedIndex]
				if (trueCorrespondences[groundTruthIndex] == setsOfRays[triangulatedIndex])
				{
					result.nbRightPointsCorrespondence++;
				}
				else
				{
					result.nbWrongPointsCorrespondence++;
				}

				// For each correspondence in setsOfRays[triangulatedIndex]
				for (const auto& setOfRays : setsOfRays[triangulatedIndex])
				{
					const auto it = std::find(trueCorrespondences[groundTruthIndex].begin(),
						trueCorrespondences[groundTruthIndex].end(),
						setOfRays);

					if (it != trueCorrespondences[groundTruthIndex].end())
					{
						// If it is present in the ground truth => true positive
						result.truePositiveCorrespondence++;
					}
					else
					{
						// If it is not present in the ground truth => false positive
						result.falsePositiveCorrespondence++;
					}
				}

				// For each correspondence in trueCorrespondences[groundTruthIndex]
				for (const auto& trueCorrespondence : trueCorrespondences[groundTruthIndex])
				{
					const auto it = std::find(setsOfRays[triangulatedIndex].begin(),
						setsOfRays[triangulatedIndex].end(),
						trueCorrespondence);

					if (it == setsOfRays[triangulatedIndex].end())
					{
						// If it is not present in the computed correspondence => false negative
						result.falseNegativeCorrespondence++;
					}
					// If it is present in the ground truth => true positive
					// But it has already be counted
				}
			}
			else
			{
				// The true point i does not have an equivalent triangulated
				
				// For each correspondence in trueCorrespondences[groundTruthIndex]
				// If it is not present in the computed correspondence => false negative
				result.falseNegativeCorrespondence += static_cast<int>(trueCorrespondences[groundTruthIndex].size());
			}
		}
		else
		{
			// In the case of a triangulate point that is a false positive
			if (assignment[i] < costCols)
			{
				const auto triangulatedIndex = assignment[i];
				
				// The true point does not have an equivalent triangulated
				// For each correspondence in trueCorrespondences[groundTruthIndex]
				// If it is not present in the ground truth => false positive
				result.falsePositiveCorrespondence += static_cast<int>(setsOfRays[triangulatedIndex].size());
			}
			else
			{
				// This should not happen
				assert(false);
			}
		}
	}

	return result;
}

template <typename T>
T computeMedian(const std::vector<T>& v)
{
	assert(!v.empty());
	
	if ((v.size() % 2) == 0)
	{
		return (v[v.size() / 2 - 1] + v[v.size() / 2]) / 2.0;
	}
	else
	{
		return v[v.size() / 2];
	}
}

template <typename T>
std::pair<T, T> computeMeanAndStd(const std::vector<T>& v)
{
    // Source: https://stackoverflow.com/a/7616783/12135531
	
	const auto sum = std::accumulate(v.begin(), v.end(), T(0.0));
	const auto mean = sum / static_cast<T>(v.size());

	std::vector<T> diff(v.size());
	std::transform(v.begin(), v.end(), diff.begin(), [mean](T x) { return x - mean; });
	const T squareSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), T(0.0));
	// Compute the sample standard deviation
	const T std = std::sqrt(squareSum / static_cast<T>(diff.size() - 1));

	return {mean, std};
}

AggregatedGroundTruthMatchingResult aggregateResults(const std::vector<GroundTruthMatchingResult>& results)
{	
	GroundTruthMatchingResult aggregationTotal;
	std::vector<double> runtimes;
	int nbRuns = 0;

	double totalAverageDifferenceInReprojectionError = 0.0;
	int totalProportionOfBetterReprojectionError = 0;

	for (const auto& result : results)
	{
		// If the result has -1 nbPointsTriangulated, it has been aborted
		if (result.nbPointsTriangulated >= 0)
		{
			nbRuns += 1;
			runtimes.push_back(result.runtime);
			aggregationTotal.nbPointsTriangulated += result.nbPointsTriangulated;
			aggregationTotal.nbPointsMissed += result.nbPointsMissed;
			aggregationTotal.nbPointsFalsePositive += result.nbPointsFalsePositive;
			aggregationTotal.nbPointsSuccessful += result.nbPointsSuccessful;
			aggregationTotal.nbRightPointsCorrespondence += result.nbRightPointsCorrespondence;
			aggregationTotal.nbWrongPointsCorrespondence += result.nbWrongPointsCorrespondence;
			aggregationTotal.truePositiveCorrespondence += result.truePositiveCorrespondence;
			aggregationTotal.falsePositiveCorrespondence += result.falsePositiveCorrespondence;
			aggregationTotal.falseNegativeCorrespondence += result.falseNegativeCorrespondence;
			totalAverageDifferenceInReprojectionError += (result.measuredReprojectionError - result.trueReprojectionError);
			
			if (result.measuredReprojectionError < result.trueReprojectionError)
			{
				totalProportionOfBetterReprojectionError += 1;
			}
			// Concat distances
			aggregationTotal.distances.insert(aggregationTotal.distances.end(),
				                              result.distances.begin(),
				                              result.distances.end());
		}
	}

	AggregatedGroundTruthMatchingResult aggregation;

	// Compute mean of parameters
	aggregation.nbRuns = nbRuns;
	std::tie(aggregation.meanRuntime, aggregation.stdRuntime) = computeMeanAndStd(runtimes);
	aggregation.nbPointsTriangulated = static_cast<double>(aggregationTotal.nbPointsTriangulated) / static_cast<double>(nbRuns);
	aggregation.nbPointsMissed = static_cast<double>(aggregationTotal.nbPointsMissed) / static_cast<double>(nbRuns);
	aggregation.nbPointsFalsePositive = static_cast<double>(aggregationTotal.nbPointsFalsePositive) / static_cast<double>(nbRuns);
	aggregation.nbPointsSuccessful = static_cast<double>(aggregationTotal.nbPointsSuccessful) / static_cast<double>(nbRuns);
	aggregation.nbRightPointsCorrespondence = static_cast<double>(aggregationTotal.nbRightPointsCorrespondence) / static_cast<double>(nbRuns);
	aggregation.nbWrongPointsCorrespondence = static_cast<double>(aggregationTotal.nbWrongPointsCorrespondence) / static_cast<double>(nbRuns);

	aggregation.precisionCorrespondence = static_cast<double>(aggregationTotal.truePositiveCorrespondence)
	                                    / static_cast<double>(aggregationTotal.truePositiveCorrespondence + aggregationTotal.falsePositiveCorrespondence);
	aggregation.recallCorrespondence = static_cast<double>(aggregationTotal.truePositiveCorrespondence)
	                                 / static_cast<double>(aggregationTotal.truePositiveCorrespondence + aggregationTotal.falseNegativeCorrespondence);
	aggregation.fMeasureCorrespondence = 2.0 * (aggregation.precisionCorrespondence * aggregation.recallCorrespondence)
	                                         / (aggregation.precisionCorrespondence + aggregation.recallCorrespondence);

	aggregation.averageDifferenceInReprojectionError = totalAverageDifferenceInReprojectionError / static_cast<double>(nbRuns);
	aggregation.proportionOfBetterReprojectionError = static_cast<double>(totalProportionOfBetterReprojectionError) / static_cast<double>(nbRuns);

	// Sort distances
	std::sort(aggregationTotal.distances.begin(), aggregationTotal.distances.end());
	// Compute the min, mean and max distances
	aggregation.minimumDistance = aggregationTotal.distances.front();
	aggregation.firstDecileDistance = aggregationTotal.distances[aggregationTotal.distances.size() / 10];
	aggregation.firstQuartileDistance = aggregationTotal.distances[aggregationTotal.distances.size() / 4];
	aggregation.medianDistance = computeMedian(aggregationTotal.distances);
	aggregation.thirdQuartileDistance = aggregationTotal.distances[3 * aggregationTotal.distances.size() / 4];
	aggregation.lastDecileDistance = aggregationTotal.distances[9 * aggregationTotal.distances.size() / 10];
	aggregation.maximumDistance = aggregationTotal.distances.back();
	// Compute mean and std
	std::tie(aggregation.meanDistance, aggregation.stdDistance) = computeMeanAndStd(aggregationTotal.distances);
	
	return aggregation;
}

void checkCorrespondenceSetsOfRays(const std::vector<std::vector<std::pair<int, int>>>& setsOfRays)
{
	int nbRightCorrespondences = 0;
	int nbWrongCorrespondences = 0;

	int nbRightPoints = 0;
	int nbWrongPoints = 0;
	
	// If rays have been generated, all rays of a point are associated to the same point on all cameras
	for (unsigned int i = 0; i < setsOfRays.size(); i++)
	{
		const auto& setOfRays = setsOfRays[i];

		bool pointIsCorrect = true;

		for (const auto& ray : setOfRays)
		{
			const auto& pointIndex = ray.second;

			if (pointIndex == static_cast<int>(i))
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
	
	const auto rateWrongCorrespondences = static_cast<double>(nbWrongCorrespondences) / static_cast<double>(totalCorrespondences);
	const auto rateWrongPoints = static_cast<double>(nbWrongPoints) / static_cast<double>(totalPoints);

	std::cout << "Number of incorrectly/correctly matched points : "
	          << nbWrongPoints << " / " << nbRightPoints << "\n"
			  << "Rate of incorrectly matched points : " << 100.f * rateWrongPoints << " %\n"
			  << "Number of incorrect/correct correspondences : "
	          << nbWrongCorrespondences << " / " << nbRightCorrespondences << "\n"
			  << "Rate of incorrect correspondences : " << 100.f * rateWrongCorrespondences << " %" << std::endl;
}
