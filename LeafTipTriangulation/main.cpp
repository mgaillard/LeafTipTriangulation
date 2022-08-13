#include <chrono>
#include <filesystem>
#include <iostream>
#include <random>
#include <tuple>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <spdlog/spdlog.h>
#include <utils/warnon.h>

#include "Camera.h"
#include "ExportScene.h"
#include "Phenotyping.h"
#include "RayMatching.h"
#include "Reconstruction.h"
#include "SyntheticData.h"
#include "Triangulation.h"

namespace fs = std::filesystem;

struct Parameters
{
	unsigned int seed;

	int numberPoints3D;
	int numberCameras;
	
	double noiseStd;
	double probabilityKeep;

	double thresholdNoPair;
};

GroundTruthMatchingResult testWithSyntheticData(const Parameters& parameters)
{
	// Set the random seed
	srand(parameters.seed);

	const bool verboseRemovePoints = false;
	const double spherePointsRadius = 1.0;
	const double sphereCamerasRadius = 3.0;
	// TODO: give the threshold for not pairing a ray and a point as a parameter (and make it a function of the noise)

	const auto points3d = generatePointsInSphere(parameters.numberPoints3D, spherePointsRadius);
	const auto cameras = generateCamerasOnSphere(parameters.numberCameras, sphereCamerasRadius);
	const auto projectedPoints2d = projectPoints(points3d, cameras);
	
	// Add noise and occlusion and shuffle points
	const auto noisyPoints2d = addNoise(projectedPoints2d, cameras, parameters.noiseStd);
	// Check that the problem can be solved
	// One point must be visible from at least two cameras
	const auto [points2d, trueCorrespondences] = removePoints(noisyPoints2d, parameters.probabilityKeep, verboseRemovePoints);

	// TODO: maybe let the single ray points and check that they are not matched with something else
	
	// If too many points have been removed, cancel the reconstruction
	if (points2d.empty() || trueCorrespondences.empty())
	{
		// Return a negative number of point triangulated to say that it has been aborted
		GroundTruthMatchingResult negativeResult;
		negativeResult.nbPointsTriangulated = -1;
		return negativeResult;
	}

	const auto rays = computeRays(cameras, points2d);
	// Check projection of points on rays (only works with no occlusion)
	// checkUnProject(points3D, rays);
	
	// Matching and triangulation of points
	const auto startTime = std::chrono::steady_clock::now();
	auto [triangulatedPoints3D, setsOfCorrespondences, viewOrder] = matchRaysAndTriangulate(cameras,
		                                                                                    points2d,
		                                                                                    rays,
		                                                                                    parameters.thresholdNoPair);
	const auto endTime = std::chrono::steady_clock::now();

	// Draw the scene in OBJ for Debugging
	// exportSceneAsOBJ(points3D, rays, "scene.obj");
	// exportSplitSceneAsOBJ(rays, setsOfCorrespondences, triangulatedPoints3D);

	// If some of the points are triangulated with only one ray, we remove them because it's probably a failed match
	removePointsFromSingleRays(triangulatedPoints3D, setsOfCorrespondences);

	// Sort the set of rays to make it uniquely identifiable even if it has been permuted
	sortSetsOfCorrespondences(setsOfCorrespondences);
	// Match the two sets of points and check the distance
	auto result = matchingTriangulatedPointsWithGroundTruth(cameras,
	                                                        points2d,
	                                                        points3d,
	                                                        trueCorrespondences,
	                                                        triangulatedPoints3D,
	                                                        setsOfCorrespondences);
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
	parameters.noiseStd = 0.0;
	parameters.probabilityKeep = 1.0;
	parameters.thresholdNoPair = std::numeric_limits<double>::max();

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
	parameters.noiseStd = 0.0;
	parameters.probabilityKeep = 1.0;
	parameters.thresholdNoPair = std::numeric_limits<double>::max();

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

void testAccuracyWithMoreCameras(double noiseStd)
{
	std::cout << "# Accuracy with varying number of cameras and noise " << noiseStd << std::endl;
	
	Parameters parameters;
	
	parameters.numberPoints3D = 10;
	parameters.noiseStd = noiseStd;
	parameters.probabilityKeep = 1.0;
	parameters.thresholdNoPair = std::numeric_limits<double>::max();
	
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
	parameters.probabilityKeep = 1.0;
	parameters.thresholdNoPair = std::numeric_limits<double>::max();

	for (int noiseStdInt = 0; noiseStdInt <= 100; noiseStdInt += 5)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.noiseStd = static_cast<double>(noiseStdInt) / 10.0;

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

void testCorrespondenceWithThreshold(double noiseStd)
{
	std::cout << "# Correspondence precision/recall with varying threshold and noise " << noiseStd << std::endl;

	Parameters parameters;

	parameters.numberPoints3D = 20;
	parameters.numberCameras = 6;
	parameters.noiseStd = noiseStd;
	parameters.probabilityKeep = 0.5;

	for (int thresholdNoPairInt = 1; thresholdNoPairInt <= 80; thresholdNoPairInt += 2)
	{
		std::vector<GroundTruthMatchingResult> results;

		parameters.thresholdNoPair = static_cast<double>(thresholdNoPairInt);

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

void runPlantPhenotypingExample2018(const std::string& folderCameras2018)
{
	auto setup = loadPhenotypingSetup(folderCameras2018);

	// X axis is from left to right
	// Y axis is from bottom to top
	// Plant: 4-9-18_Schnable_49-281-JS39-65_2018-04-11_12-09-35_9968800
	const SetsOfVec2 points2d = {
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

	// List of annotated views
	const std::vector<std::string> availableViews = {
		ViewSv0,
		ViewSv72,
		ViewSv144,
		ViewSv216,
		ViewSv288,
		ViewTv90
	};

	// List of all views available in the camera setup
	auto views = setup.views();

	for (const auto& v : availableViews)
	{
		// Remove views that are available
		views.erase(std::remove_if(views.begin(),
		                           views.end(),
		                           [&v](const auto& x) { return x == v; }),
		            views.end());
	}

	// Now remove unavailable views from the camera setup
	for (const auto& v : views)
	{
		setup.removeView(v);
	}

	// Compute rays in 3D from camera matrices and 2D points
	const auto rays = computeRays(setup.cameras(), points2d);

	auto [triangulatedPoints3D, setsOfCorrespondences, viewOrder] = matchRaysAndTriangulate(setup.cameras(), points2d, rays);

	// Export the scene
	exportSplitSceneAsOBJ(rays, setsOfCorrespondences, triangulatedPoints3D);
}

void runPlantPhenotypingExample2022(const std::string& folderCameras2022)
{
	const auto setup = loadPhenotypingSetup(folderCameras2022);
	
	PlantPhenotypePoints plant("3-10-22-Schnable-Sorghum_310-219-42-3-BTx623_2022-03-11_15-17-51.394_5209300");

	// 4 points are visible from 3 cameras
	// Occlusions are simulated by discarding points
	// None of the points are visible by all cameras
	// No camera can see all points

	// Camera 1
	plant.addPointsFromView(ViewSv36, {
		// { 1629.0, 6035.0 }, // Leaf 1
		{ 1404.0, 5674.0 }, // Leaf 2
		{ 3127.0, 4843.0 }, // Leaf 3
		{ 2709.0, 4551.0 }  // Leaf 4
	});

	// Camera 2
	plant.addPointsFromView(ViewSv72, {
		{ 1455.0, 5913.0 }, // Leaf 1
		// { 1022.0, 5520.0 }, // Leaf 2
		{ 3248.0, 4977.0 }, // Leaf 3
		{ 1869.0, 4541.0 }  // Leaf 4
	});

	// Camera 3
	plant.addPointsFromView(ViewSv108, {
		{ 1597.0, 5790.0 }, // Leaf 1
		{ 1163.0, 5338.0 }, // Leaf 2
		// { 2957.0, 5083.0 }, // Leaf 3
		{ 1248.0, 4509.0 }  // Leaf 4
	});

	std::vector<PlantPhenotypePoints> plants = {std::move(plant)};

	// Apply the transformation from the image-based calibration
	PlantImageTranslations translations;
	translations.loadFromCsv(folderCameras2022 + "/calibration.csv");
	translations.applyTranslationToPlants(plants);

	// Flip Y axis because our camera model origin is on the bottom left
	flipYAxisOnAllPlants(setup, plants);

	spdlog::info("Triangulating leaves of plant {}", plants.front().plantName());

	const auto viewNames = plants.front().getAllViews();
	const auto points = plants.front().pointsFromViews(viewNames);
	const auto cameras = setup.camerasFromViews(viewNames);
	const auto rays = computeRays(cameras, points);
	const auto [triangulatedPoints3D, setsOfCorrespondences, viewOrder] = matchRaysAndTriangulate(cameras, points, rays, 100.0);

	computeDistributionOfSimilarities("distribution_similarities.txt", cameras, points, rays, setsOfCorrespondences);

	// Export the scene
	spdlog::debug("Found {} leaves", triangulatedPoints3D.size());
	exportSplitSceneAsOBJ(rays, setsOfCorrespondences, triangulatedPoints3D);
}

void runPlantPhenotypingSyntheticData(const fs::path& folder)
{
	constexpr unsigned int seed = 0;
	constexpr int numberPlants = 100;
	constexpr int numberLeavesMin = 5;
	constexpr int numberLeavesMax = 15;
	constexpr double radius = 0.5;

	// Set the random seed for reproducibility
	srand(seed);
	std::minstd_rand randomGenerator(seed);
	std::uniform_int_distribution<int> leafDistribution(numberLeavesMin, numberLeavesMax);

	const auto setup = loadPhenotypingSetup(folder);

	SetsOfVec3 points3d(numberPlants);
	std::vector<PlantPhenotypePoints> plants;
	plants.reserve(numberPlants);

	for (int i = 0; i < numberPlants; i++)
	{
		// Generate the number of leaves
		const auto numberLeaves = leafDistribution(randomGenerator);
		// Generate the leaf tips
		points3d[i] = generatePointsInSphere(numberLeaves, radius);

		// Project points to cameras
		const auto& viewNames = setup.views();
		const auto& cameras = setup.cameras();
		const auto points2d = projectPoints(points3d[i], cameras);

		assert(viewNames.size() == points2d.size());

		// Convert to a PlantPhenotypePoints object
		PlantPhenotypePoints plant("synthetic_plant_" + std::to_string(i));
		for (unsigned int j = 0; j < points2d.size(); j++)
		{
			plant.addPointsFromView(viewNames[j], points2d[j]);
		}

		plants.push_back(plant);
	}

	// Flip axis for all views to get the coordinates with the Y axis origin on the top of the image
	flipYAxisOnAllPlants(setup, plants);

	// Apply the rotation 3 times, because it is equivalent as applying it once the other way
	for (int i = 0; i < 3; i++)
	{
		apply90DegreesRotationToViews(ViewTv90, setup, plants);
	}

	// Export the 2D points to CSV
	const auto leafTipsCsvPath = folder / "leaf_tips.csv";
	exportPlantPointsToCsv(leafTipsCsvPath.string(), plants);

	// Export the ground truth number of leaves to CSV
	std::vector<std::pair<std::string, int>> numberLeaves(numberPlants);
	for (int i = 0; i < numberPlants; i++)
	{
		numberLeaves[i].first = plants[i].plantName();
		numberLeaves[i].second = static_cast<int>(points3d[i].size());
	}
	const auto groundTruthCsvPath = folder / "annotations" / "ground_truth.csv";
	exportNumberLeavesToCsv(groundTruthCsvPath.string(), numberLeaves);

	// Export the ground truth 3D position of points
	const auto groundTruthTxtPath = folder / "annotations" / "ground_truth_position.txt";
	exportPositionLeavesToTxt(groundTruthTxtPath.string(), plants, points3d);
}

std::tuple<PhenotypingSetup, std::vector<PlantPhenotypePoints>>
loadPhenotypingSetupAndPhenotypePoints(const std::string& folder, PlantPhenotypePointType type)
{
	// Convert the output of the calibration script to a CSV file that can be read by readAndApplyTranslationsFromCsv()
	// Use the following line (and replace the name of files)
	// convertCalibrationOutputToCsv(folder + "calibration_output.txt", folder + "calibration.csv");

	auto setup = loadPhenotypingSetup(folder);
	auto plants = readPhenotypePointsFromCsv(folder + "/leaf_tips.csv", type);
	keepOnlyPlantsWithMultipleViews(plants);
	// Apply the transformation from the image-based calibration
	PlantImageTranslations translations;
	translations.loadFromCsv(folder + "/calibration.csv");
	translations.applyTranslationToPlants(plants);
	// Only for top views there is a 90 degrees rotation
	// For the 2018 data set, the rotation is clockwise
	// For the 2022 data set, the rotation is counterclockwise
	apply90DegreesRotationToViews(ViewTv90, setup, plants);
	// Flip Y axis because our camera model origin is on the bottom left
	flipYAxisOnAllPlants(setup, plants);

	return { setup, plants };
}

void runPlantPhenotyping(const std::string& folder, const std::string& phenotype, const std::string& plantName)
{
	const auto type = readPhenotypingPointTypeFromString(phenotype);
	auto [setup, plants] = loadPhenotypingSetupAndPhenotypePoints(folder, type);

	// Select only the plant whose name is plantName
	for (auto itPlant = plants.begin(); itPlant != plants.end();)
	{
		if (itPlant->plantName() != plantName)
		{
			itPlant = plants.erase(itPlant);
		}
		else
		{
			++itPlant;
		}
	}

	if (plants.empty())
	{
		spdlog::error("Could not find the plant: {}", plantName);
		return;
	}

	spdlog::info("Triangulating leaves of plant {}", plantName);

	const auto [triangulatedPoints3D, setsOfCorrespondences] = triangulatePhenotypePoints(setup, plants.front(), 1500.0);

	// Export the scene
	// Re-compute the rays because it is needed for exporting some of the information
	const auto viewNames = plants.front().getAllViews();
	const auto points = plants.front().pointsFromViews(viewNames);
	const auto cameras = setup.camerasFromViews(viewNames);
	auto rays = computeRays(cameras, points);
	clampRaysWithPhenotypingSetup(setup, viewNames, rays);
	spdlog::debug("Found {} leaves", triangulatedPoints3D.size());
	exportSplitSceneAsOBJ(rays, setsOfCorrespondences, triangulatedPoints3D);

	// Compute and save the distribution of similarities
	computeDistributionOfSimilarities("distribution_similarities.txt", cameras, points, rays, setsOfCorrespondences);
}

void runLeafCounting(
	const std::string& folder,
	const std::string& phenotype,
	double thresholdNoPair,
	unsigned int seed,
	double probabilityDiscard,
	const std::string& outputFileNumberLeaves,
	const std::string& outputFileAnnotationsPerView)
{
	const auto type = readPhenotypingPointTypeFromString(phenotype);
	auto [setup, plants] = loadPhenotypingSetupAndPhenotypePoints(folder, type);

	spdlog::info("Triangulating leaves for {} plants in the folder {}", plants.size(), folder);

	constexpr auto eps = 1e-4;
	if (probabilityDiscard > eps)
	{
		spdlog::debug("Discarding points with probability {:.2f} % (seed = {:d})", 100.0 * probabilityDiscard, seed);
		discardPointsRandomly(seed, probabilityDiscard, plants);
	}
	else if (probabilityDiscard < -eps)
	{
		spdlog::debug("Adding points with probability {:.2f} % (seed = {:d})", -100.0 * probabilityDiscard, seed);
		addPointsRandomly(setup, seed, -probabilityDiscard, plants);
	}

	if (thresholdNoPair < std::numeric_limits<double>::max())
	{
		spdlog::debug("Association threshold is {:.2f}", thresholdNoPair);
	}

	std::vector<std::pair<std::string, int>> numberLeafTips(plants.size());

	#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plants.size()); i++)
	{
		const auto [triangulatedPoints3D, setsOfCorrespondences] = triangulatePhenotypePoints(setup, plants[i], thresholdNoPair);
		numberLeafTips[i].first = plants[i].plantName();
		numberLeafTips[i].second = static_cast<int>(triangulatedPoints3D.size());
	}

	spdlog::debug("Saving plant annotation statistics in {}", outputFileAnnotationsPerView);
	exportPlantStatsToCsv(outputFileAnnotationsPerView, setup, plants);
	spdlog::debug("Saving number of leaves in {}", outputFileNumberLeaves);
	exportNumberLeavesToCsv(outputFileNumberLeaves, numberLeafTips);
}

void runExportAnnotations(
	const std::string& annotationFolderStr,
	const std::string& phenotype,
	double thresholdNoPair,
	const std::string& imageFolderStr,
	const std::string& outputFolderStr)
{
	const fs::path annotationFolder(annotationFolderStr);
	const fs::path imageFolder(imageFolderStr);
	const fs::path outputFolder(outputFolderStr);

	if (!fs::exists(annotationFolder) || !fs::is_directory(annotationFolder))
	{
		spdlog::error("The annotation folder {} does not exist.", annotationFolderStr);
		return;
	}

	if (!fs::exists(imageFolder) || !fs::is_directory(imageFolder))
	{
		spdlog::error("The image folder {} does not exist.", imageFolderStr);
		return;
	}

	if (!fs::exists(outputFolder) || !fs::is_directory(outputFolder))
	{
		spdlog::error("The output folder {} does not exist.", outputFolderStr);
		return;
	}

	if (thresholdNoPair < std::numeric_limits<double>::max())
	{
		spdlog::debug("Association threshold is {:.2f}", thresholdNoPair);
	}

	const auto type = readPhenotypingPointTypeFromString(phenotype);
	// Load list of plants for running the triangulation
	const auto [setup, plants_calibrated] = loadPhenotypingSetupAndPhenotypePoints(annotationFolderStr, type);
	// Load again the list of plants without calibration
	// Note that annotations are not calibrated to reflect exactly what was clicked by the annotators
	auto plants = readPhenotypePointsFromCsv(annotationFolderStr + "/leaf_tips.csv", type);
	// Flip Y axis of annotations because our camera model origin is on the bottom left
	flipYAxisOnAllPlants(setup, plants);
	// Load the image translations to translate projected points back to the original images
	PlantImageTranslations translations;
	translations.loadFromCsv(annotationFolderStr + "/calibration.csv");

	// Triangulate plants for displaying the results reprojected with the annotations
	std::vector<SetsOfVec2> triangulatedPoints(plants_calibrated.size());
	std::vector<std::vector<std::vector<int>>> triangulatedPointsMatches(plants_calibrated.size());
	#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(plants_calibrated.size()); i++)
	{
		// Triangulate points of the plant
		const auto [triangulatedPoints3D, setsOfCorrespondences] = triangulatePhenotypePoints(setup, plants_calibrated[i], thresholdNoPair);
		// Re-project points of the plant on views, and retain correspondences with 2D to draw matches in views
		auto [plantPoints, plantMatches] = projectPhenotypePointsAndRetainMatches(setup,
			                                                                      plants_calibrated[i],
			                                                                      triangulatedPoints3D,
			                                                                      setsOfCorrespondences);

		// Apply the inverse translation that was applied during imaged based calibration to projected points
		applyInverseTranslationsOnPhenotypePoints(translations,
		                                          plants_calibrated[i].plantName(),
		                                          plants_calibrated[i].getAllViews(),
		                                          plantPoints);

		triangulatedPoints[i] = std::move(plantPoints);
		triangulatedPointsMatches[i] = std::move(plantMatches);
	}

	spdlog::info("Exporting annotations for {} plants in the folder {}", plants.size(), annotationFolderStr);

	for (unsigned int p = 0; p < plants.size(); p++)
	{
		const auto& plant = plants[p];

		spdlog::debug("Exporting annotations for plant {}", plant.plantName());

		const auto outputPlantFolder = outputFolder / plant.plantName();
		auto plantFolder = imageFolder / plant.plantName();

		// If the folder with original images of the plant does not exist, raise warning and move on
		if (!fs::exists(plantFolder))
		{
			spdlog::warn("Could not open the plant folder for {}. Trying to find matches...", plant.plantName());

			// Try to look for similar directory names
			bool alternativeFound = false;
			for (const auto& entry : fs::directory_iterator(imageFolder))
			{
				if (entry.is_directory())
				{
					// If the name of the directory contains the name of the current plant, we try to use it
					const auto directoryName = entry.path().filename().string();
					if (directoryName.find(plant.plantName()) != std::string::npos)
					{
						spdlog::warn("Using plant folder {} instead", directoryName);
						plantFolder = entry.path();
						alternativeFound = true;
						break;
					}
				}
			}

			if (!alternativeFound)
			{
				spdlog::warn("Could not find any alternative folder, skipping plant.");
				continue;
			}
		}

		// If the output folder does not exist, create it
		fs::create_directory(outputPlantFolder);

		const auto viewNames = plant.getAllViews();
		#pragma omp parallel for 
		for (int i = 0; i < static_cast<int>(viewNames.size()); i++)
		{
			const auto& viewName = viewNames[i];
			const auto filename = translateViewNameToFilename(viewName, "png");
			const auto viewFile = plantFolder / filename;
			const auto outputViewFile = outputPlantFolder / filename;

			if (!fs::exists(viewFile))
			{
				spdlog::warn("Could not open the view file {} for plant {}.", filename, plant.plantName());
				continue;
			}

			const auto points = plant.pointsFromView(viewName);
			drawPointsInImageWithMatches(outputViewFile.string(),
			                             viewFile.string(),
			                             points,
			                             triangulatedPoints[p][i],
			                             triangulatedPointsMatches[p][i]);
		}
	}
}

void runCrocodileMeasurement()
{
	// X axis is from left to right
	// Y axis is from bottom to top
	const SetsOfVec2 points2d = {
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
	                                                    points2d);

	spdlog::info("Length: {} m", pointsDistance);
}

int main(int argc, char *argv[])
{
	spdlog::set_level(spdlog::level::trace);

	if (argc < 2)
	{
		spdlog::error("Give the command to execute as an argument.");
		return 1;
	}

	// The command to execute
	const std::string command = argv[1];

	// Add any remaining argument to an array
	std::vector<std::string> args;
	for (int i = 2; i < argc; i++)
	{
		args.emplace_back(argv[i]);
	}

	// Execute the command
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
		testAccuracyWithMoreCameras(0.1);
	    testAccuracyWithMoreCameras(0.5);
		testAccuracyWithMoreCameras(2.0);
	}
	else if (command == "correspondence_noise")
	{
		// F-measure of correspondence vs noise with 6 cameras
		testCorrespondenceWithMoreNoise();
	}
	else if (command == "correspondence_threshold")
	{
		// ROC curve of correspondence with vs threshold with 6 cameras
		testCorrespondenceWithThreshold(0.1);
		testCorrespondenceWithThreshold(0.5);
		testCorrespondenceWithThreshold(2.0);
	}
	else if (command == "plant_phenotyping_example_2018")
	{
		if (args.empty())
		{
			spdlog::error("Missing arguments for plant phenotyping.");
			spdlog::error("Argument 1: Path to the folder for the 2018 phenotyping setup.");
			return 1;
		}

		// Example with a plant in a phenotyping facility
		runPlantPhenotypingExample2018(args[0]);
	}
	else if (command == "plant_phenotyping_example_2022")
	{
		if (args.empty())
		{
			spdlog::error("Missing arguments for plant phenotyping.");
			spdlog::error("Argument 1: Path to the folder for the 2022 phenotyping setup.");
			return 1;
		}

		// Example with a plant in a phenotyping facility
		runPlantPhenotypingExample2022(args[0]);
	}
	else if (command == "plant_phenotyping_synthetic_data")
	{
		if (args.empty())
		{
			spdlog::error("Missing arguments for plant phenotyping.");
			spdlog::error("Argument 1: Path to the folder for the phenotyping setup.");
			return 1;
		}

		// Example with a plant in a phenotyping facility
		runPlantPhenotypingSyntheticData(args[0]);
	}
	else if (command == "plant_phenotyping")
	{
		if (args.size() < 3)
		{
			spdlog::error("Missing arguments for plant phenotyping");
			spdlog::error("Argument 1: Path to the folder for the phenotyping setup.");
			spdlog::error("Argument 2: Phenotype to triangulate and count (tips, junctions).");
			spdlog::error("Argument 3: Name of the plant to inspect.");
			return 1;
		}

		// Triangulating leaf tips for a set of manually annotated plants
		runPlantPhenotyping(args[0], args[1], args[2]);
	}
	else if (command == "leaf_counting")
	{
		if (args.size() < 7)
		{
			spdlog::error("Missing arguments for leaf counting");
			spdlog::error("Argument 1: Path to the folder for the phenotyping setup.");
			spdlog::error("Argument 2: Phenotype to triangulate and count (tips, junctions).");
			spdlog::error("Argument 3: Theta parameter in px (threshold above which two points cannot be matched).");
			spdlog::error("Argument 4: Random seed (unsigned integer).");
			spdlog::error("Argument 5: Percentage of points to discard between 0.0 and 1.0.");
			spdlog::error("Argument 6: Output CSV file for the number of leaves.");
			spdlog::error("Argument 7: Output CSV file for statistics about plants.");
			return 1;
		}

		// If theta is zero, set to to infinity
		double thresholdNoPair = std::stod(args[2]);
		if (thresholdNoPair <= 0.0)
		{
			thresholdNoPair = std::numeric_limits<double>::max();
		}

		// Counting leaves for a set of manually annotated plants
		runLeafCounting(args[0],
						args[1],
				        thresholdNoPair,
						std::stoul(args[3]),
		                std::stod(args[4]),
		                args[5],
		                args[6]);
	}
	else if (command == "export_annotations")
	{
		if (args.size() < 5)
		{
			spdlog::error("Missing arguments for leaf counting");
			spdlog::error("Argument 1: Path to the folder for the phenotyping setup.");
			spdlog::error("Argument 2: Phenotype to triangulate and count (tips, junctions).");
			spdlog::error("Argument 3: Theta parameter in px (threshold above which two points cannot be matched).");
			spdlog::error("Argument 4: Path to the folder with images.");
			spdlog::error("Argument 5: Path to the output folder.");

			return 1;
		}

		// If theta is zero, set to to infinity
		double thresholdNoPair = std::stod(args[2]);
		if (thresholdNoPair <= 0.0)
		{
			thresholdNoPair = std::numeric_limits<double>::max();
		}

		runExportAnnotations(args[0], args[1], thresholdNoPair, args[3], args[4]);
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
