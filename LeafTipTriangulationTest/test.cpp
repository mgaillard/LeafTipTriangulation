#define  _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch.hpp>

#include "Camera.h"
#include "Triangulation.h"

/**
 * \brief Test the triangulation from multiple views:
 *         - Project the 3D point on all views
 *         - Check that the re-projection on views is correct
 *         - Check that the re-projection error is close to zero
 *         - Check that the point is accurately triangulated from all views
 * \param cameras A set of cameras
 * \param point A 3D point
 */
void testTriangulationOnePoint(const std::vector<Camera>& cameras, const glm::vec3& point)
{
	// Matrices to project a 3D point to 2D point in viewport coordinates
	std::vector<glm::mat4> projectionMatrices;
	std::vector<glm::vec2> points2d;

	for (const auto& camera : cameras)
	{
		const auto& projectionMatrix = camera.mat();
		const auto projectedPoint = camera.windowToViewport(camera.project(point));

		// Check that re-projecting the point with the homography gives the right result
		const glm::vec4 homogeneousPoint(point[0], point[1], point[2], 1.0f);
		const auto reProjectedPoint = projectPoint(projectionMatrix, homogeneousPoint);
		REQUIRE(reProjectedPoint[0] == Approx(projectedPoint[0]));
		REQUIRE(reProjectedPoint[1] == Approx(projectedPoint[1]));

		// Projection matrix
		projectionMatrices.push_back(projectionMatrix);
		// Projected point in viewport coordinates
		points2d.push_back(projectedPoint);
	}

	// Error when re-projecting the ground truth should almost 0
	const auto groundTruthError = reprojectionErrorFromMultipleViews(projectionMatrices, points2d, point);
	REQUIRE(groundTruthError == Approx(0.0).margin(1e-12));

	float error;
	glm::vec3 triangulatedPoint;
	std::tie(error, triangulatedPoint) = triangulatePointFromMultipleViews(projectionMatrices, points2d);

	REQUIRE(triangulatedPoint[0] == Approx(point.x));
	REQUIRE(triangulatedPoint[1] == Approx(point.y));
	REQUIRE(triangulatedPoint[2] == Approx(point.z));
}

std::vector<Camera> generateCamerasAroundOrigin(int nbCameras)
{
	std::vector<Camera> cameras;

	for (int i = 0; i < nbCameras; i++)
	{
		const auto angle = 2 * M_PI * (static_cast<double>(i) / static_cast<double>(nbCameras));

		cameras.emplace_back(
			glm::vec3(2.0 * std::cos(angle), 2.0 * std::sin(angle), 0.0),
			glm::vec3(0.0, 0.0, 0.0),
			glm::vec3(0.0, 0.0, 1.0)
		);
	}

	return cameras;
}

TEST_CASE("Triangulation of one point from 2 views", "[triangulation]")
{
	// Define two cameras
	const Camera camera1(
		glm::vec3(0.0, 2.0, 0.0),
		glm::vec3(0.0, 0.0, 0.0),
		glm::vec3(0.0, 0.0, 1.0));

	const Camera camera2(
		glm::vec3(2.0, 0.0, 0.0),
		glm::vec3(0.0, 0.0, 0.0),
		glm::vec3(0.0, 0.0, 1.0));

	const std::vector<Camera> cameras = {
		camera1,
		camera2
	};

	// Test the triangulation with several points
	testTriangulationOnePoint(cameras, glm::vec3(0.1, -0.2, 0.15));
	testTriangulationOnePoint(cameras, glm::vec3(0.298, -0.767, -0.881));
	testTriangulationOnePoint(cameras, glm::vec3(0.154, 0.715, -0.733));
	testTriangulationOnePoint(cameras, glm::vec3(0.339, 0.957, -0.331));
	testTriangulationOnePoint(cameras, glm::vec3(-0.346, 0.887, -0.869));
}

TEST_CASE("Triangulation of one point from multiple views", "[triangulation]")
{
	// Define twelve cameras in a circle around the origin
	const int nbCameras = 12;
	const auto cameras = generateCamerasAroundOrigin(nbCameras);

	// Test the triangulation with several points
	testTriangulationOnePoint(cameras, glm::vec3(0.1, -0.2, 0.15));
	testTriangulationOnePoint(cameras, glm::vec3(0.298, -0.767, -0.881));
	testTriangulationOnePoint(cameras, glm::vec3(0.154, 0.715, -0.733));
	testTriangulationOnePoint(cameras, glm::vec3(0.339, 0.957, -0.331));
	testTriangulationOnePoint(cameras, glm::vec3(-0.346, 0.887, -0.869));
}

#ifdef CATCH_CONFIG_ENABLE_BENCHMARKING
TEST_CASE("Benchmark triangulation of one point", "[triangulation][benchmark]")
{
	// Define twelve cameras in a circle around the origin
	const int nbCameras = 12;
	const auto cameras = generateCamerasAroundOrigin(nbCameras);

	// Get the first N cameras
	const std::vector<Camera> cameras2(cameras.begin(), cameras.begin() + 2);
	const std::vector<Camera> cameras3(cameras.begin(), cameras.begin() + 3);
	const std::vector<Camera> cameras6(cameras.begin(), cameras.begin() + 6);
	
	BENCHMARK("Number of cameras: 2")
	{
		testTriangulationOnePoint(cameras2, glm::vec3(0.1, -0.2, 0.15));
	};
	
	BENCHMARK("Number of cameras: 3")
	{
		testTriangulationOnePoint(cameras3, glm::vec3(0.1, -0.2, 0.15));
	};
	
	BENCHMARK("Number of cameras: 6")
	{
		testTriangulationOnePoint(cameras6, glm::vec3(0.1, -0.2, 0.15));
	};

	BENCHMARK("Number of cameras: 12")
	{
		testTriangulationOnePoint(cameras, glm::vec3(0.1, -0.2, 0.15));
	};
}
#endif
