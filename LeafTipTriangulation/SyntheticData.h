#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "Camera.h"
#include "Ray.h"

struct GroundTruthMatchingResult
{
	/**
	 * \brief Time needed to solve the problem
	 */
	double runtime;
	
	/**
	 * \brief Number of points output by the triangulation method
	 */
	int nbPointsTriangulated;

	/**
	 * \brief Number of points that were missed by the algorithm
	 */
	int nbPointsMissed;

	/**
	 * \brief Number of points that were added by the algorithm even if they should not be present
	 */
	int nbPointsFalsePositive;

	/**
	 * \brief Number of point reconstructed and matched with the ground truth
	 */
	int nbPointsSuccessful;

	/**
	 * \brief Number of successfully reconstructed points with perfect correspondences
	 */
	int nbRightPointsCorrespondence;

	/**
	 * \brief Number of successfully reconstructed points without perfect correspondences
	 */
	int nbWrongPointsCorrespondence;

	int truePositiveCorrespondence;
	int falsePositiveCorrespondence;
	int falseNegativeCorrespondence;
	
	/**
	 * \brief A list of all distances between triangulated points and true points
	 */
	std::vector<double> distances;

	GroundTruthMatchingResult() :
		nbPointsTriangulated(0),
		nbPointsMissed(0),
		nbPointsFalsePositive(0),
		nbPointsSuccessful(0),
		nbRightPointsCorrespondence(0),
		nbWrongPointsCorrespondence(0),
		truePositiveCorrespondence(0),
		falsePositiveCorrespondence(0),
		falseNegativeCorrespondence(0)
	{
		
	}
};

struct AggregatedGroundTruthMatchingResult
{
	/**
	 * \brief Time needed to solve the problem
	 */
	double meanRuntime;
	double stdRuntime;

	/**
	 * \brief The number of results aggregated
	 */
	int nbRuns;

	/**
	 * \brief Number of points output by the triangulation method
	 */
	double nbPointsTriangulated;

	/**
	 * \brief Number of points that were missed by the algorithm
	 */
	double nbPointsMissed;

	/**
	 * \brief Number of points that were added by the algorithm even if they should not be present
	 */
	double nbPointsFalsePositive;

	/**
	 * \brief Number of point reconstructed and matched with the ground truth
	 */
	double nbPointsSuccessful;

	/**
	 * \brief Number of successfully reconstructed points with perfect correspondences
	 */
	double nbRightPointsCorrespondence;

	/**
	 * \brief Number of successfully reconstructed points without perfect correspondences
	 */
	double nbWrongPointsCorrespondence;

	double precisionCorrespondence;
	double recallCorrespondence;
	double fMeasureCorrespondence;

	/**
	 * \brief The minimum distance from ground-truth to triangulation
	 */
	double minimumDistance;

	/**
	 * \brief The first decile distance from ground-truth to triangulation
	 */
	double firstDecileDistance;

	/**
	 * \brief The first quartile distance from ground-truth to triangulation
	 */
	double firstQuartileDistance;

	/**
	 * \brief The median distance from ground-truth to triangulation
	 */
	double medianDistance;

	/**
	 * \brief The third quartile distance from ground-truth to triangulation
	 */
	double thirdQuartileDistance;

	/**
	 * \brief The last decile distance from ground-truth to triangulation
	 */
	double lastDecileDistance;

	/**
	 * \brief The maximum distance from ground-truth to triangulation
	 */
	double maximumDistance;

	/**
	 * \brief The mean distance from ground-truth to triangulation
	 */
	double meanDistance;

	/**
	 * \brief The standard deviation of distance from ground-truth to triangulation
	 */
	double stdDistance;

	AggregatedGroundTruthMatchingResult() :
		nbRuns(0),
		nbPointsTriangulated(0.0),
		nbPointsMissed(0.0),
		nbPointsFalsePositive(0.0),
		nbPointsSuccessful(0.0),
		nbRightPointsCorrespondence(0.0),
		nbWrongPointsCorrespondence(0.0),
		minimumDistance(0.0),
		firstDecileDistance(0.0),
		firstQuartileDistance(0.0),
		medianDistance(0.0),
		thirdQuartileDistance(0.0),
		lastDecileDistance(0.0),
		maximumDistance(0.0),
		meanDistance(0.0),
		stdDistance(0.0)
	
	{

	}
};

/**
 * \brief Generate 3D points in a sphere centered around the origin
 * \param n The number of points to generate
 * \param radius The radius of the sphere
 * \return A vector of 3D points
 */
std::vector<glm::vec3> generatePointsInSphere(int n, float radius);

/**
 * \brief Generate cameras pointing to the origin on a sphere
 * \param n Number of cameras
 * \param radius Radius of the sphere
 * \return A vector of camera
 */
std::vector<Camera> generateCamerasOnSphere(int n, float radius);

/**
 * \brief Project 3D points on cameras. The viewport is 1000*1000 px.
 * \param points A list of 3D points
 * \param cameras A list of cameras
 * \return 2D points projected on cameras
 */
std::vector<std::vector<glm::vec2>> projectPoints(const std::vector<glm::vec3>& points,
												  const std::vector<Camera>& cameras);

/**
 * \brief Add gaussian noise to 2D points
 * \param points A list of 2D points
 * \param cameras The list of cameras projecting the two points
 * \param noiseStd Standard deviation of the gaussian noise added to 2D points
 * \return The list of 2D points, with added noise
 */
std::vector<std::vector<glm::vec2>> addNoise(const std::vector<std::vector<glm::vec2>>& points,
	                                         const std::vector<Camera>& cameras,
	                                         float noiseStd);

/**
 * \brief Randomly remove points from the list to simulation occlusion
 * \param points A list of 2D points
 * \param probabilityKeep A probability to keep a 2D point
 * \param verbose Whether to display information about the new point configuration or not
 * \return The list of 2D points minus some points that are removed and the set of ground truth correspondences
 */
std::pair<std::vector<std::vector<glm::vec2>>, std::vector<std::vector<std::pair<int, int>>>>
removePoints(const std::vector<std::vector<glm::vec2>>& points,
			 float probabilityKeep,
	         bool verbose);

/**
 * \brief Check that each 3D point is close to a it's associated ray.
 *        Use this function to test the output of the function computeRays() when ground truth is available.
 * \param points A list of 3D points
 * \param rays A list of Rays associated to 3D points
 * \return True if all 3D points are within 1e-3 of all their rays, false otherwise
 */
bool checkUnProject(const std::vector<glm::vec3>& points,
	                const std::vector<std::vector<Ray>>& rays);

/**
 * \brief Return the cost of matching triangulated 3D points to ground truth 3D points.
 *        Display the maximum distance between a 3D point and it's matching triangulated 3D point.
 *        Display the average distance between the ground truth and the triangulation.
 *        Use this function to check that the triangulation without correspondences was successful.
 * \param triangulatedPoints3D A list of triangulated 3D points
 * \param setsOfRays The list of correspondences for the triangulation
 * \param points3D A list of 3D points
 * \param trueCorrespondences The list of true correspondences
 * \param outputCsv Whether to output the results in CSV format or in human readable format
 * \return A struct that stores statistics
 */
GroundTruthMatchingResult matchingTriangulatedPointsWithGroundTruth(
	const std::vector<glm::vec3>& triangulatedPoints3D,
	const std::vector<std::vector<std::pair<int, int>>>& setsOfRays,
	const std::vector<glm::vec3>& points3D,
	const std::vector<std::vector<std::pair<int, int>>>& trueCorrespondences);

/**
 * \brief Aggregate and compute statistics on a list of results
 * \param results A list of results
 * \return Aggregated results
 */
AggregatedGroundTruthMatchingResult aggregateResults(const std::vector<GroundTruthMatchingResult>& results);

/**
 * \brief Check the correspondence of rays to make sure the matching is perfect
 * \param setsOfRays The matching of rays that best triangulates 2D points projected by cameras
 */
void checkCorrespondenceSetsOfRays(const std::vector<std::vector<std::pair<int, int>>>& setsOfRays);
