#include <chrono>
#include <iostream>
#include <tuple>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <utils/warnon.h>

#include "Camera.h"
#include "ExportScene.h"
#include "Phenotyping.h"
#include "RayMatching.h"
#include "Reconstruction.h"
#include "SyntheticData.h"

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

	const bool verboseRemovePoints = false;
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
	std::tie(points2D, trueCorrespondences) = removePoints(noisyPoints2D, parameters.probabilityKeep, !verboseRemovePoints);

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
	auto result = matchingTriangulatedPointsWithGroundTruth(cameras,
	                                                        points2D,
	                                                        points3D,
	                                                        trueCorrespondences,
	                                                        triangulatedPoints3D,
	                                                        setsOfRays);
	// Update run time
	result.runtime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
	
	return result;
}

void testRuntimeWithMorePoints()
{
	std::cout << "# Runtime in ms with varying number of points" << std::endl;

	// Number of points and number of runs to compute the average runtime
	// Longer instances are ran less times than smaller ones to reduce the total
	const std::vector<std::pair<int, int>> testHyperParameters = {
		{ 10, 1000 },
		{ 20, 1000 },
		{ 50, 1000 },
		{ 100, 500 },
		{ 200, 200 },
		{ 500, 100 },
		{ 1000, 50 },
		{ 2000, 20 },
		{ 5000, 10 },
		{ 10000, 5 }
	};
	
	Parameters parameters;

	parameters.numberCameras = 3;
	// No noise so that it is the worst case scenario for the number of points to match
	parameters.noiseStd = 0.0f;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();

	for (const auto& testHyperParameter : testHyperParameters)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.numberPoints3D = testHyperParameter.first;
		const int nbRuns = testHyperParameter.second;

		for (int s = 0; s < nbRuns; s++)
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
			// Number of runs to compute the average
			<< aggregation.nbRuns << "\t"
			// Average runtime
			<< aggregation.meanRuntime << "\t"
			<< aggregation.stdRuntime << std::endl;
	}
}

void testRuntimeWithMoreCameras()
{
	std::cout << "# Runtime in ms with varying number of cameras" << std::endl;

	// Number of cameras and number of runs to compute the average runtime
	// Longer instances are ran less times than smaller ones to reduce the total
	const std::vector<std::pair<int, int>> testHyperParameters = {
		{ 2, 100 },
		{ 3, 100 },
		{ 4, 100 },
		{ 5, 100 },
		{ 6, 50 },
		{ 7, 30 },
		{ 8, 20 },
		{ 9, 10 },
		{ 10, 3 },
		{ 11, 3 },
		{ 12, 3 },
		{ 13, 3 },
		{ 14, 3 },
		{ 15, 3 },
	};

	Parameters parameters;

	parameters.numberPoints3D = 20;
	// No noise so that it is the worst case scenario for the number of points to match
	parameters.noiseStd = 0.0f;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();

	for (const auto& testHyperParameter : testHyperParameters)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.numberCameras = testHyperParameter.first;
		const int nbRuns = testHyperParameter.second;

		for (int s = 0; s < nbRuns; s++)
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
			// Number of runs to compute the average
			<< aggregation.nbRuns << "\t"
			// Average runtime
			<< aggregation.meanRuntime << "\t"
			<< aggregation.stdRuntime << std::endl;
	}
}

void testAccuracyWithMoreCameras(float noiseStd)
{
	std::cout << "# Accuracy with varying number of cameras and noise " << noiseStd << std::endl;
	
	Parameters parameters;
	
	parameters.numberPoints3D = 10;
	parameters.noiseStd = noiseStd;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();
	
	for (int numberCameras = 2; numberCameras <= 10; numberCameras++)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.numberCameras = numberCameras;

		#pragma omp parallel for default(none) firstprivate(parameters) shared(results)
		for (int s = 0; s < 10000; s++)
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
			<< parameters.noiseStd << "\t"
			// Number of points to reconstruct
			<< parameters.numberPoints3D << "\t"
			// Number of points 100% successfully reconstructed (hopefully equal to numberPoints3D)
			<< aggregation.nbRightPointsCorrespondence << "\t"
			<< aggregation.minimumDistance << "\t"
			<< aggregation.firstDecileDistance << "\t"
			<< aggregation.firstQuartileDistance << "\t"
			<< aggregation.medianDistance << "\t"
			<< aggregation.thirdQuartileDistance << "\t"
			<< aggregation.lastDecileDistance << "\t"
			<< aggregation.maximumDistance << "\t"
			<< aggregation.meanDistance << "\t"
			<< aggregation.stdDistance << std::endl;
	}
}

void testCorrespondenceWithMoreNoise()
{
	std::cout << "# Correspondence precision/recall with varying noise " << std::endl;

	Parameters parameters;

	parameters.numberPoints3D = 10;
	parameters.numberCameras = 6;
	parameters.probabilityKeep = 1.0f;
	parameters.thresholdNoPair = std::numeric_limits<float>::max();

	for (int noiseStdInt = 0; noiseStdInt <= 100; noiseStdInt += 5)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.noiseStd = float(noiseStdInt) / 10.f;

#pragma omp parallel for default(none) firstprivate(parameters) shared(results)
		for (int s = 0; s < 10000; s++)
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
			<< aggregation.fMeasureCorrespondence << "\t"
			<< aggregation.proportionOfBetterReprojectionError << "\t"
			<< aggregation.averageDifferenceInReprojectionError << std::endl;
	}
}

void testCorrespondenceWithThreshold(float noiseStd)
{
	std::cout << "# Correspondence precision/recall with varying threshold and noise " << noiseStd << std::endl;

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
		for (int s = 0; s < 10000; s++)
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

void runPlantPhenotyping()
{
	const auto setup = loadPhenotypingSetup("cameras/");

	// X axis is from left to right
	// Y axis is from bottom to top
	// Plant: 4-9-18_Schnable_49-281-JS39-65_2018-04-11_12-09-35_9968800
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
	const auto rays = computeRays(setup.cameras(), points2D);

	auto [triangulatedPoints3D, setsOfRays] = matchRaysAndTriangulate(setup.cameras(), points2D, rays);

	// Export the scene
	exportSplitSceneAsOBJ(rays, setsOfRays, triangulatedPoints3D);
}

void runLeafCounting(const std::string& folder)
{
	// Convert the output of the calibration script to a CSV file that can be read by readAndApplyTranslationsFromCsv()
	// Use the following line (and replace the name of files)
	// convertCalibrationOutputToCsv(folder + "calibration_output.txt", folder + "calibration.csv");

	const auto setup = loadPhenotypingSetup(folder + "/cameras/");
	auto plants = readLeafTipsFromCSV(folder + "/leaf_tips.csv");
	keepOnlyPlantsWithMultipleViews(plants);
	// Apply the transformation from the image-based calibration
	readAndApplyTranslationsFromCsv(folder + "/calibration.csv", plants);
	// Only for top views there is a 90 degrees clockwise rotation
	// TODO: apply the exact rotation from the SorghumReconstruction app (89.33f clockwise) with 2018 data set
	apply90DegreesRotationToViews("TV_90", setup, plants);
	// Flip Y axis because our camera model origin is on the bottom left
	flipYAxisOnAllPlants(setup, plants);

	std::vector<std::pair<std::string, int>> numberLeafTips(plants.size());

	#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plants.size()); i++)
	{
		const auto triangulatedPoints3D = triangulateLeafTips(setup, plants[i]);
		numberLeafTips[i].first = plants[i].plantName();
		numberLeafTips[i].second = static_cast<int>(triangulatedPoints3D.size());
	}

	for (const auto& [plantName, number] : numberLeafTips)
	{
		std::cout << plantName << "\t" << number << std::endl;
	}
}

void runCrocodileMeasurement()
{
	// X axis is from left to right
	// Y axis is from bottom to top
	const std::vector<std::vector<glm::vec2>> points2D = {
		// Camera 0
		{
			{1877, 1237},
			{2185, 1056},
		},
		// Camera 1
		{
			{1687, 1164},
			{2108, 1147},
		},
		// Camera 2
		{
			{1352, 1078},
			{1701, 1154},
		},
		// Camera 3
		{
			{1714, 821},
			{1944, 1067},
		},
		// Camera 4
		{
			{2198, 1334},
			{2300, 1102},
		},
	};

	constexpr int imageWidth = 4032;
	constexpr int imageHeight = 3024;

	// Camera calibration
	cv::Mat1d cameraMatrix = (cv::Mat1d(3, 3) <<
		3153.273013526734, 0.0, 2005.9916092061376, // fx, 0, cx
		0.0, 3105.260406995229, 1540.0981727685203, // 0, fy, cy
		0.0, 0.0, 1.0);

	cv::Mat1d distCoeffs = (cv::Mat1d(5, 1) <<
		0.280605688854058, // k1
		-1.6945809491702621, // k2
		0.0024575553828127643, // p1
		-0.0021983284657425975, // p2
		3.2574845170325277); // k3

	// Cameras poses
	const std::vector<cv::Mat1d> rvecs = {
		(cv::Mat1d(3, 1) << 2.40327, -0.457555, 0.335426),
		(cv::Mat1d(3, 1) << 2.40059, 0.350462, -0.116547),
		(cv::Mat1d(3, 1) << 2.2446, 0.816225, -0.373654),
		(cv::Mat1d(3, 1) << 2.16444, 1.58991, -0.220916),
		(cv::Mat1d(3, 1) << 2.21499, -1.05024, 0.374562),
	};

	const std::vector<cv::Mat1d> tvecs = {
		(cv::Mat1d(3, 1) << -0.0581547, 0.0542219, 0.163637),
		(cv::Mat1d(3, 1) << -0.119149, 0.0101526, 0.220561),
		(cv::Mat1d(3, 1) << -0.156187, -0.0288829, 0.281005),
		(cv::Mat1d(3, 1) << -0.112906, -0.111428, 0.307856),
		(cv::Mat1d(3, 1) << 0.0137574, 0.0700162, 0.207419),
	};

	const auto pointsDistance = measureTwoPointsCharuco(imageWidth,
	                                                    imageHeight,
	                                                    cameraMatrix,
	                                                    distCoeffs,
	                                                    rvecs,
	                                                    tvecs,
	                                                    points2D);

	std::cout << "Length: " << pointsDistance << " m" << std::endl;
}

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		std::cout << "Give the benchmark to execute as an argument." << std::endl;
		return 1;
	}

	const std::string command = argv[1];

	if (command == "runtime_points")
	{
		// Runtime (single core) vs number of points with 3 cameras
		testRuntimeWithMorePoints();
	}
	else if (command == "runtime_cameras")
	{
		// Runtime (single core) vs number of cameras
		testRuntimeWithMoreCameras();
	}
	else if (command == "accuracy_cameras")
	{
		// Accuracy vs number of cameras
		testAccuracyWithMoreCameras(0.1f);
	    testAccuracyWithMoreCameras(0.5f);
		testAccuracyWithMoreCameras(2.0f);
	}
	else if (command == "correspondence_noise")
	{
		// F-measure of correspondence vs noise with 6 cameras
		testCorrespondenceWithMoreNoise();
	}
	else if (command == "correspondence_threshold")
	{
		// ROC curve of correspondence with vs threshold with 6 cameras
		testCorrespondenceWithThreshold(0.1f);
		testCorrespondenceWithThreshold(0.5f);
		testCorrespondenceWithThreshold(2.0f);
	}
	else if (command == "plant_phenotyping")
	{
		// Example with a plant in a phenotyping facility
		runPlantPhenotyping();
	}
	else if (command == "leaf_counting")
	{
		// Example with counting leaves for a set of manually annotated plants
		runLeafCounting("sorghum_2018");
	}
	else if (command == "measure_crocodile")
	{
		// Example of measuring the length between two points in a LEGO set
		// Reference images are in the folder /Images/crocodile/*.jpg
		runCrocodileMeasurement();
	}
	
	// TODO: misdetection and false positive versus threshold
	
    return 0;
}
