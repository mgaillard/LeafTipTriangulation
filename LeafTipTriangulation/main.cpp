#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "Camera.h"
#include "SyntheticData.h"
#include "ExportScene.h"
#include "RayMatching.h"
#include "Triangulation.h"

struct Parameters
{
	unsigned int seed;

	int numberPoints3D;
	int numberCameras;
	
	float noiseStd;
	float probabilityKeep;

	float thresholdNoPair;
};

GroundTruthMatchingResult testWithSyntheticData(const Parameters& parameters)
{
	// Set the random seed
	srand(parameters.seed);

	const bool outputCsv = true; // TODO: deprecated
	const float spherePointsRadius = 1.f;
	const float sphereCamerasRadius = 3.f;
	// TODO: give the threshold for not pairing a ray and a point as a parameter (and make it a function of the noise)

	const auto points3D = generatePointsInSphere(parameters.numberPoints3D, spherePointsRadius);
	const auto cameras = generateCamerasOnSphere(parameters.numberCameras, sphereCamerasRadius);
	const auto projectedPoints2D = projectPoints(points3D, cameras);
	
	// Add noise and occlusion and shuffle points
	const auto noisyPoints2D = addNoise(projectedPoints2D, cameras, parameters.noiseStd);
	// Check that the problem can be solved
	// One point must be visible from at least two cameras
	std::vector<std::vector<glm::vec2>> points2D;
	std::vector<std::vector<std::pair<int, int>>> trueCorrespondences;
	std::tie(points2D, trueCorrespondences) = removePoints(noisyPoints2D, parameters.probabilityKeep, !outputCsv);

	// TODO: maybe let the single ray points and check that they are not matched with something else
	
	// If too many points have been removed, cancel the reconstruction
	if (points2D.empty() || trueCorrespondences.empty())
	{
		// Return a negative number of point triangulated to say that it has been aborted
		GroundTruthMatchingResult negativeResult;
		negativeResult.nbPointsTriangulated = -1;
		return negativeResult;
	}

	const auto rays = computeRays(cameras, points2D);
	// Check projection of points on rays (only works with no occlusion)
	// checkUnProject(points3D, rays);
	
	// Matching and triangulation of points
	std::vector<glm::vec3> triangulatedPoints3D;
	std::vector<std::vector<std::pair<int, int>>> setsOfRays;
	const auto startTime = std::chrono::steady_clock::now();
	std::tie(triangulatedPoints3D, setsOfRays) = matchRaysAndTriangulate(cameras, points2D, rays, parameters.thresholdNoPair);
	const auto endTime = std::chrono::steady_clock::now();

	// Draw the scene in OBJ for Debugging
	// exportSceneAsOBJ(points3D, rays, "scene.obj");
	// exportSplitSceneAsOBJ(rays, setsOfRays, triangulatedPoints3D);

	// If some of the points are triangulated with only one ray, we remove them because it's probably a failed match
	removePointsFromSingleRays(triangulatedPoints3D, setsOfRays);

	// Sort the set of rays to make it uniquely identifiable even if it has been permuted
	sortSetsOfRays(setsOfRays);
	// Match the two sets of points and check the distance
	auto result = matchingTriangulatedPointsWithGroundTruth(triangulatedPoints3D, setsOfRays, points3D, trueCorrespondences);
	// Update run time
	result.runtime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
	
	return result;
}

void testRuntimeWithMorePoints()
{
	std::cout << "Runtime in ms with varying number of points" << std::endl;

	const std::vector<int> numberPoints = {
		10,
		20,
		50,
		100,
		200,
		500,
		1000,
		2000,
		5000,
		10000
	};
	
	Parameters parameters;

	parameters.numberCameras = 3;
	// No noise so that it is the worst case scenario for the number of points to match
	parameters.noiseStd = 0.0f;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();

	for (const auto& numberPoint : numberPoints)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.numberPoints3D = numberPoint;

		for (int s = 0; s < 100; s++)
		{
			parameters.seed = s;

			const auto result = testWithSyntheticData(parameters);
			results.push_back(result);
		}

		const auto aggregation = aggregateResults(results);

		std::cout
			// Number of points to reconstruct
			<< parameters.numberPoints3D << "\t"
			// Number of points 100% successfully reconstructed (hopefully equal to numberPoints3D)
			<< aggregation.nbRightPointsCorrespondence << "\t"
			// Average runtime
			<< aggregation.runtime << std::endl;
	}
}

void testRuntimeWithMoreCameras()
{
	std::cout << "Runtime in ms with varying number of cameras" << std::endl;

	Parameters parameters;

	parameters.numberPoints3D = 20;
	// No noise so that it is the worst case scenario for the number of points to match
	parameters.noiseStd = 0.0f;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();

	for (int numberCameras = 2; numberCameras <= 10; numberCameras++)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.numberCameras = numberCameras;

		for (int s = 0; s < 100; s++)
		{
			parameters.seed = s;

			const auto result = testWithSyntheticData(parameters);
			results.push_back(result);
		}

		const auto aggregation = aggregateResults(results);

		std::cout
		    << parameters.numberCameras << "\t"
			// Number of points to reconstruct
			<< parameters.numberPoints3D << "\t"
			// Number of points 100% successfully reconstructed (hopefully equal to numberPoints3D)
			<< aggregation.nbRightPointsCorrespondence << "\t"
			// Average runtime
			<< aggregation.runtime << std::endl;
	}
}

void testAccuracyWithMoreCameras(float noiseStd)
{
	std::cout << "Accuracy with varying number of cameras and noise " << noiseStd << std::endl;
	
	Parameters parameters;
	
	parameters.numberPoints3D = 10;
	parameters.noiseStd = noiseStd;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();
	
	for (int numberCameras = 3; numberCameras <= 6; numberCameras++)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.numberCameras = numberCameras;

		#pragma omp parallel for default(none) firstprivate(parameters) shared(results)
		for (int s = 0; s < 2000; s++)
		{
			parameters.seed = s;

			const auto result = testWithSyntheticData(parameters);

			#pragma omp critical(resultupdate)
			{
				results.push_back(result);
			}
		}

		const auto aggregation = aggregateResults(results);

		std::cout
			<< parameters.numberCameras << "\t"
			// Number of points to reconstruct
			<< parameters.numberPoints3D << "\t"
			// Number of points 100% successfully reconstructed (hopefully equal to numberPoints3D)
			<< aggregation.nbRightPointsCorrespondence << "\t"
			<< aggregation.minimumDistance << "\t"
			<< aggregation.firstQuartileDistance << "\t"
			<< aggregation.medianDistance << "\t"
			<< aggregation.thirdQuartileDistance << "\t"
			<< aggregation.maximumDistance << "\t"
			<< aggregation.meanDistance << "\t" << std::endl;
	}
}

void testCorrespondenceWithMoreCameras()
{
	std::cout << "Correspondence precision/recall with varying noise " << std::endl;

	Parameters parameters;

	parameters.numberPoints3D = 10;
	parameters.numberCameras = 6;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();

	for (int noiseStdInt = 0; noiseStdInt <= 20; noiseStdInt++)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.noiseStd = float(noiseStdInt) / 10.f;

#pragma omp parallel for default(none) firstprivate(parameters) shared(results)
		for (int s = 0; s < 50; s++) // TODO: set to 2000
		{
			parameters.seed = s;

			const auto result = testWithSyntheticData(parameters);

#pragma omp critical(resultupdate)
			{
				results.push_back(result);
			}
		}

		const auto aggregation = aggregateResults(results);

		std::cout
			<< parameters.noiseStd << "\t"
			// Number of points to reconstruct
			<< parameters.numberPoints3D << "\t"
			// Number of points 100% successfully reconstructed (hopefully equal to numberPoints3D)
			<< aggregation.nbRightPointsCorrespondence << "\t"
			<< aggregation.precisionCorrespondence << "\t"
			<< aggregation.recallCorrespondence << "\t"
			<< aggregation.fMeasureCorrespondence << "\t" << std::endl;
	}
}

void testCorrespondenceWithThreshold(float noiseStd)
{
	std::cout << "Correspondence precision/recall with varying threshold and noise " << noiseStd << std::endl;

	Parameters parameters;

	parameters.numberPoints3D = 20;
	parameters.numberCameras = 6;
	parameters.noiseStd = noiseStd;
	parameters.probabilityKeep = 0.5f;

	for (int thresholdNoPairInt = 1; thresholdNoPairInt <= 40; thresholdNoPairInt++)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.thresholdNoPair = float(thresholdNoPairInt);

#pragma omp parallel for default(none) firstprivate(parameters) shared(results)
		for (int s = 0; s < 500; s++) // TODO: set to 2000
		{
			parameters.seed = s;

			const auto result = testWithSyntheticData(parameters);

#pragma omp critical(resultupdate)
			{
				results.push_back(result);
			}
		}

		const auto aggregation = aggregateResults(results);

		std::cout
			<< parameters.noiseStd << "\t"
			<< parameters.probabilityKeep << "\t"
			<< parameters.thresholdNoPair << "\t"
			<< aggregation.nbRuns << "\t"
			// Number of points to reconstruct
			<< parameters.numberPoints3D << "\t"
			// Number of points 100% successfully reconstructed (hopefully equal to numberPoints3D)
			<< aggregation.nbRightPointsCorrespondence << "\t"
			<< aggregation.precisionCorrespondence << "\t"
			<< aggregation.recallCorrespondence << "\t"
			<< aggregation.fMeasureCorrespondence << "\t" << std::endl;
	}
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
	// Runtime (single core) vs number of points with 3 cameras
	// testRuntimeWithMorePoints();

	// Runtime (single core) vs number of cameras
	// testRuntimeWithMoreCameras();
	
	// Accuracy vs number of cameras with noise 0.1
	// testAccuracyWithMoreCameras(0.1f);
	// testAccuracyWithMoreCameras(0.2f);
	// testAccuracyWithMoreCameras(0.5f);
	// testAccuracyWithMoreCameras(1.0f);

	// F-measure of correspondence vs noise with 6 cameras
	// testCorrespondenceWithMoreCameras();

	// ROC curve of correspondence with vs threshold with 6 cameras
	// testCorrespondenceWithThreshold(0.1f);
	// testCorrespondenceWithThreshold(0.2f);
	// testCorrespondenceWithThreshold(0.5f);
	// testCorrespondenceWithThreshold(1.0f);
	// testCorrespondenceWithThreshold(1.2f);
	// testCorrespondenceWithThreshold(1.5f);
	// testCorrespondenceWithThreshold(2.0f);

	// TODO: Accuracy vs noise with 6 cameras


	// TODO: example with a plant in a phenotyping facility
	
	// TODO: example with a real object and a ChaRuCo calibration pattern
	
    return 0;
}
