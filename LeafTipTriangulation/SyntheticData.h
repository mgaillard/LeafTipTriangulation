#pragma once

#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Camera.h"
#include "Ray.h"
#include "Types.h"

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
	 * \brief Reprojection error of the 3D points to triangulate with the true correspondences
	 */
	double trueReprojectionError;

	/**
	 * \brief Reprojection error of the triangulated 3D points with the matched correspondences
	 */
	double measuredReprojectionError;
	
	/**
	 * \brief A list of all distances between triangulated points and true points
	 */
	std::vector<double> distances;

	GroundTruthMatchingResult() : runtime(0.0),
	                              nbPointsTriangulated(0),
	                              nbPointsMissed(0),
	                              nbPointsFalsePositive(0),
	                              nbPointsSuccessful(0),
	                              nbRightPointsCorrespondence(0),
	                              nbWrongPointsCorrespondence(0),
	                              truePositiveCorrespondence(0),
	                              falsePositiveCorrespondence(0),
	                              falseNegativeCorrespondence(0),
	                              trueReprojectionError(0.0),
	                              measuredReprojectionError(0.0)
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
	 * \brief Average difference between the measured reprojection error and the true reprojection error
	 *        A negative value means that the measured reprojection error is better than the true reprojection error
	 */
	double averageDifferenceInReprojectionError;

	/**
	 * \brief Proportion of the solutions that give better reprojection error than 
	 */
	double proportionOfBetterReprojectionError;

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

	AggregatedGroundTruthMatchingResult() : meanRuntime(0.0),
	                                        stdRuntime(0.0),
	                                        nbRuns(0),
	                                        nbPointsTriangulated(0.0),
	                                        nbPointsMissed(0.0),
	                                        nbPointsFalsePositive(0.0),
	                                        nbPointsSuccessful(0.0),
	                                        nbRightPointsCorrespondence(0.0),
	                                        nbWrongPointsCorrespondence(0.0),
	                                        precisionCorrespondence(0.0),
	                                        recallCorrespondence(0.0),
	                                        fMeasureCorrespondence(0.0),
	                                        averageDifferenceInReprojectionError(0.0),
	                                        proportionOfBetterReprojectionError(0.0),
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
SetOfVec3 generatePointsInSphere(int n, double radius);

/**
 * \brief Generate cameras pointing to the origin on a sphere
 * \param n Number of cameras
 * \param radius Radius of the sphere
 * \return A vector of camera
 */
std::vector<Camera> generateCamerasOnSphere(int n, double radius);

/**
 * \brief Project 3D points on cameras. The viewport is 1000*1000 px.
 * \param points3d A list of 3D points
 * \param cameras A list of cameras
 * \return 2D points projected on cameras
 */
SetsOfVec2 projectPoints(const SetOfVec3& points3d,
												   const std::vector<Camera>& cameras);

/**
 * \brief Add gaussian noise to 2D points
 * \param points A list of 2D points
 * \param cameras The list of cameras projecting the two points
 * \param noiseStd Standard deviation of the gaussian noise added to 2D points
 * \return The list of 2D points, with added noise
 */
SetsOfVec2 addNoise(const SetsOfVec2& points,
	                                          const std::vector<Camera>& cameras,
	                                          double noiseStd);

/**
 * \brief Randomly remove points from the list to simulation occlusion
 * \param points A list of 2D points
 * \param probabilityKeep A probability to keep a 2D point
 * \param verbose Whether to display information about the new point configuration or not
 * \return The list of 2D points minus some points that are removed and the set of ground truth correspondences
 */
std::pair<SetsOfVec2, SetsOfCorrespondences>
removePoints(const SetsOfVec2& points,
			 double probabilityKeep,
	         bool verbose);

/**
 * \brief Check that each 3D point is close to a it's associated ray.
 *        Use this function to test the output of the function computeRays() when ground truth is available.
 * \param points A list of 3D points
 * \param rays A list of Rays associated to 3D points
 * \return True if all 3D points are within 1e-3 of all their rays, false otherwise
 */
bool checkUnProject(const SetOfVec3& points,
	                const SetsOfRays& rays);

/**
 * \brief Return the cost of matching triangulated 3D points to ground truth 3D points.
 *        Display the maximum distance between a 3D point and it's matching triangulated 3D point.
 *        Display the average distance between the ground truth and the triangulation.
 *        Use this function to check that the triangulation without correspondences was successful.
 * \param cameras Cameras used to project points
 * \param points2d The list of 2D points seen on the cameras
 * \param points3d The list of true 3D points
 * \param trueCorrespondences The list of true correspondences
 * \param triangulatedPoints3D A list of triangulated 3D points
 * \param setsOfCorrespondences The list of correspondences for the triangulation
 * \return A struct that stores statistics
 */
GroundTruthMatchingResult matchingTriangulatedPointsWithGroundTruth(
	const std::vector<Camera>& cameras,
	const SetsOfVec2& points2d,
	const SetOfVec3& points3d,
	const SetsOfCorrespondences& trueCorrespondences,
	const SetOfVec3& triangulatedPoints3D,
	const SetsOfCorrespondences& setsOfCorrespondences);

/**
 * \brief Aggregate and compute statistics on a list of results
 * \param results A list of results
 * \return Aggregated results
 */
AggregatedGroundTruthMatchingResult aggregateResults(const std::vector<GroundTruthMatchingResult>& results);

/**
 * \brief Check the correspondence of rays to make sure the matching is perfect
 * \param setsOfCorrespondences The matching of rays that best triangulates 2D points projected by cameras
 */
void checkCorrespondenceSetsOfCorrespondences(const SetsOfCorrespondences& setsOfCorrespondences);
