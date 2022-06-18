#include <catch2/catch.hpp>

#include "Constants.h"
#include "Camera.h"
#include "Triangulation.h"

// Hold the test data for rays
struct RayTestData
{
	Ray ray0;
	Ray ray1;
	bool success;
	glm::vec3 intersection;
};

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
	std::vector<glm::dmat4> projectionMatrices;
	std::vector<glm::dvec2> points2d;

	for (const auto& camera : cameras)
	{
		const auto& projectionMatrix = camera.mat();
		const auto projectedPoint = camera.windowToViewport(camera.project(point));

		// Check that re-projecting the point with the homography gives the right result
		const glm::dvec4 homogeneousPoint(point[0], point[1], point[2], 1.0);
		const auto reProjectedPoint = projectPoint(projectionMatrix, homogeneousPoint);
		REQUIRE(reProjectedPoint[0] == Approx(projectedPoint[0]));
		REQUIRE(reProjectedPoint[1] == Approx(projectedPoint[1]));

		// Projection matrix
		projectionMatrices.emplace_back(projectionMatrix);
		// Projected point in viewport coordinates
		points2d.emplace_back(projectedPoint);
	}

	// Error when re-projecting the ground truth should almost 0
	const auto groundTruthError = reprojectionErrorFromMultipleViews(projectionMatrices, points2d, point);
	REQUIRE(groundTruthError == Approx(0.0).margin(1e-12));
	
	const auto [error, triangulatedPoint] = triangulatePointFromMultipleViews(projectionMatrices, points2d);

	REQUIRE(triangulatedPoint[0] == Approx(point.x));
	REQUIRE(triangulatedPoint[1] == Approx(point.y));
	REQUIRE(triangulatedPoint[2] == Approx(point.z));
}

std::vector<Camera> generateCamerasAroundOrigin(int nbCameras)
{
	std::vector<Camera> cameras;

	for (int i = 0; i < nbCameras; i++)
	{
		const auto angle = 2 * constants::pi * (static_cast<double>(i) / static_cast<double>(nbCameras));

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

TEST_CASE("Triangulation of many points from multiple views", "[triangulation]")
{
	// Define twelve cameras in a circle around the origin
	const int nbCameras = 12;
	const auto cameras = generateCamerasAroundOrigin(nbCameras);

	// Some 3D points that will be triangulated (randomly generated)
	const std::vector<glm::vec3> points3d = {
		glm::vec3(0.401, 0.075, -0.28),
		glm::vec3(0.095, 0.275, 0.687),
		glm::vec3(-0.739, 0.592, -0.952),
		glm::vec3(-0.89, 0.265, 0.336),
		glm::vec3(0.933, 0.107, 0.822),
		glm::vec3(-0.828, -0.315, 0.782),
		glm::vec3(-0.941, -0.587, -0.63),
		glm::vec3(-0.425, 0.698, -0.346),
		glm::vec3(0.415, -0.715, 0.12),
		glm::vec3(-0.237, 0.476, 0.935),
		glm::vec3(0.764, -0.107, 0.714),
		glm::vec3(0.017, 0.346, -0.991),
		glm::vec3(0.839, -0.932, -0.45),
		glm::vec3(-0.36, 0.813, -0.975),
		glm::vec3(-0.344, 0.228, 0.314),
		glm::vec3(0.286, 0.881, -0.261),
		glm::vec3(-0.027, -0.432, -0.678),
		glm::vec3(0.264, -0.023, 0.775),
		glm::vec3(0.408, -0.905, -0.018),
		glm::vec3(-0.009, -0.938, 0.223)
	};

	// For each camera, a list of 2D points seen from it
	std::vector<std::vector<glm::vec2>> points2d(cameras.size());
	// For each 3D point, the list of cameras and points seeing it
	std::vector<std::vector<std::pair<int, int>>> correspondences(points3d.size());

	// Project 3D points and keep track of correspondences
	for (int c = 0; c < static_cast<int>(cameras.size()); c++)
	{
		for (int i = 0; i < static_cast<int>(points3d.size()); i++)
		{
			const auto& point3d = points3d[i];
			const auto& camera = cameras[c];
			
			const auto projectedPoint = camera.project(point3d);

			points2d[c].emplace_back(projectedPoint);
			correspondences[i].emplace_back(c, i);
		}
	}

	// Check that the ground-truth reprojection error is zero
	const auto groundTruthError = reprojectionErrorManyPointsFromMultipleViews(cameras, points2d, correspondences, points3d);
	REQUIRE(groundTruthError == Approx(0.0).margin(1e-12));

	// Run the triangulation
	const auto [triangulationError, triangulatedPoints] = triangulateManyPointsFromMultipleViews(cameras, points2d, correspondences);

	// Check that points are triangulated correctly
	REQUIRE(points3d.size() == triangulatedPoints.size());
	REQUIRE(triangulationError == Approx(0.0).margin(1e-12));
	for (unsigned int i = 0; i < points3d.size(); i++)
	{
		REQUIRE(points3d[i].x == Approx(triangulatedPoints[i].x));
		REQUIRE(points3d[i].y == Approx(triangulatedPoints[i].y));
		REQUIRE(points3d[i].z == Approx(triangulatedPoints[i].z));
	}

	// Test that the reprojection error is the same as the one returned by the triangulation function
	const auto error = reprojectionErrorManyPointsFromMultipleViews(cameras, points2d, correspondences, triangulatedPoints);
	REQUIRE(error == Approx(triangulationError));
}

TEST_CASE("Rays pseudo intersections parallel (lines)", "[rays]")
{
	// Rays that are parallel
	const Ray parallelR1{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
	const Ray parallelR2{ {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0} };
	glm::vec3 intersection;

	const auto success = raysPseudoIntersection(parallelR1, parallelR2, intersection);
	REQUIRE_FALSE(success);
}

TEST_CASE("Rays pseudo intersections parallel (line segments)", "[rays]")
{
	const std::vector<RayTestData> tests = {
		// Line segment 0 on top, line segment 1 at the bottom
		{
			{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 2.0, 3.0 },
			{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.5, 1.5, 0.0 }
		},
		// Line segment 0 on top, line segment 1 at the bottom (exact match)
		{
			{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 1.0, 2.0 },
			{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.5, 1.0, 0.0 }
		},
		// Line segment 0 on top, line segment 1 at the bottom (overlap A1-A0-B1-B0)
		{
			{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 1.0, 3.0 },
			{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 2.0 },
			false,
			{ 0.0, 0.0, 0.0 }
		},
		// Line segment 0 on the left, line segment 1 the right (overlap A0-A1-B1-B0)
		{
			{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 3.0 },
			{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 1.0, 2.0 },
			false,
			{ 0.0, 0.0, 0.0 }
		}
	};

	for (const auto& test : tests)
	{
		glm::vec3 intersection;
		bool success;
		// Run the test one way
		success = raysPseudoIntersection(test.ray0, test.ray1, intersection);

		REQUIRE(success == test.success);

		// If the test case has a unique pseudo-intersection, check its coordinates
		if (test.success)
		{
			REQUIRE(intersection.x == Approx(test.intersection.x));
			REQUIRE(intersection.y == Approx(test.intersection.y));
			REQUIRE(intersection.z == Approx(test.intersection.z));
		}

		// Run the test the other way (swap ray0 and ray1)
		// Run the test one way
		success = raysPseudoIntersection(test.ray1, test.ray0, intersection);

		REQUIRE(success == test.success);

		// If the test case has a unique pseudo-intersection, check its coordinates
		if (test.success)
		{
			REQUIRE(intersection.x == Approx(test.intersection.x));
			REQUIRE(intersection.y == Approx(test.intersection.y));
			REQUIRE(intersection.z == Approx(test.intersection.z));
		}
	}
}

TEST_CASE("Rays pseudo intersections parallel (mixed line and line segments)", "[rays]")
{
	// Rays that are parallel
	const Ray parallelR1{ {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0} };
	const Ray parallelR2{ {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0, 1.0 };
	glm::vec3 intersection;

	const auto success = raysPseudoIntersection(parallelR1, parallelR2, intersection);
	REQUIRE_FALSE(success);
}

TEST_CASE("Rays pseudo intersections (lines)", "[rays]")
{
	// Very simple hard coded test
	const Ray r1{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0} };
	const Ray r2{ {1.0, 1.0, 1.0}, {-1.0, 0.0, 0.0} };
	glm::vec3 intersection;
	bool success;

	success = raysPseudoIntersection(r1, r2, intersection);
	REQUIRE(success);
	REQUIRE(intersection.x == Approx(0.0));
	REQUIRE(intersection.y == Approx(1.0));
	REQUIRE(intersection.z == Approx(0.5));

	// Test against the current implementation on random input rays
	const std::array<Ray, 4> r =
	{ {
		{ {0.067, 0.310, 0.721}, {0.196, 0.362, 0.911} },
		{ {0.867, 0.158, 0.293}, {0.762, 0.620, 0.189} },
		{ {0.343, 0.805, 0.849}, {0.073, 0.021, 0.997} },
		{ {0.808, 0.951, 0.391}, {0.111, 0.798, 0.592} }
	} };

	const std::array<std::array<glm::vec3, 4>, 4> intersections = { {
		{{
			{ 0.0, 0.0, 0.0},
			{ 0.07263205945491791, -0.1626813411712646, 0.08639559149742126 },
			{ 0.3989423215389252, 0.844562828540802, 2.097469806671143 },
			{ 0.2778298556804657, -0.02141415514051914, -0.2319426983594894 },
		}},
		{{
			{ 0.07263205945491791, -0.1626813411712646, 0.08639559149742126 },
			{ 0.0, 0.0, 0.0},
			{ 0.5734415054321289, 0.4661479294300079, 0.2999313771724701 },
			{ 0.7931162714958191, 0.2724314630031586, 0.1348105072975159 },
		}},
		{{
			{ 0.3989423215389252, 0.844562828540802, 2.097469806671143 },
			{ 0.5734415054321289, 0.4661479294300079, 0.2999313771724701 },
			{ 0.0, 0.0, 0.0},
			{ 0.5407107472419739, 0.772326648235321, 0.2602503001689911 },
		}},
		{{
			{ 0.2778298556804657, -0.02141415514051914, -0.2319426983594894 },
			{ 0.7931162714958191, 0.2724314630031586, 0.1348105072975159 },
			{ 0.5407107472419739, 0.772326648235321, 0.2602503001689911 },
			{ 0.0, 0.0, 0.0}
		}}
	} };

	for (unsigned int i = 0; i < r.size(); i++)
	{
		for (unsigned int j = 0; j < r.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			
			success = raysPseudoIntersection(r[i], r[j], intersection);
			REQUIRE(success);
			REQUIRE(intersection.x == Approx(intersections[i][j].x));
			REQUIRE(intersection.y == Approx(intersections[i][j].y));
			REQUIRE(intersection.z == Approx(intersections[i][j].z));
		}
	}
}

TEST_CASE("Rays pseudo intersections (line segments)", "[rays]")
{
	const std::vector<RayTestData> tests = {
		// Intersection in the middle of the two line segments, equivalent to lines
		{
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 2.0 },
			{ {1.0, 1.0, 1.0}, {-1.0, 0.0, 0.0}, 0.0, 2.0 },
			true,
			{ 0.0, 1.0, 0.5 }
		},
		// Intersection at A of line segment 0 and middle of line segment 1
		{
			{ {1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 1.0 },
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 2.0 },
			true,
			{ 0.5, 1.0, 0.5 }
		},
		// Intersection at B of line segment 0 and middle of line segment 1
		{
			{ {-1.0, 1.0, 1.0}, {-1.0, 0.0, 0.0}, 0.0, 1.0 },
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 2.0 },
			true,
			{ -0.5, 1.0, 0.5 }
		},
		// Intersection at A of line segment 1 and middle of line segment 0
		{
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 2.0 },
			{ {1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.5, 1.0, 0.5 }
		},
		// Intersection at B of line segment 1 and middle of line segment 0
		{
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 2.0 },
			{ {-1.0, 1.0, 1.0}, {-1.0, 0.0, 0.0}, 0.0, 1.0 },
			true,
			{ -0.5, 1.0, 0.5 }
		},
		// Intersection at both line segments in A
		{
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 1.0 },
			{ {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.0, 0.0, 0.5 }
		},
		// Intersection at A of line segments 0 and at B of line segment 1
		{
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 1.0 },
			{ {-1.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.0, 0.0, 0.5 }
		},
		// Intersection at B of line segments 0 and at A of line segment 1
		{
			{ {-1.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 1.0 },
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.0, 0.0, 0.5 }
		},
		// Intersection at both line segments in B
		{
			{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, 1.0 },
			{ {-1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 1.0 },
			true,
			{ 0.0, 1.0, 0.5 }
		},
	};

	for (const auto& test: tests)
	{
		// Run the test
		glm::vec3 intersection;
		const auto success = raysPseudoIntersection(test.ray0, test.ray1, intersection);

		REQUIRE(success == test.success);

		// If the test case has a unique pseudo-intersection, check its coordinates
		if (test.success)
		{
			REQUIRE(intersection.x == Approx(test.intersection.x));
			REQUIRE(intersection.y == Approx(test.intersection.y));
			REQUIRE(intersection.z == Approx(test.intersection.z));
		}
	}
}

TEST_CASE("Rays pseudo intersections (mixed line and line segments)", "[rays]")
{
	// Line ray (infinite)
	const Ray r1{ {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0} };
	// Line segment ray (finite)
	const Ray r2{ {1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}, 0.0, 2.0 };
	// Line segment ray (finite)
	const Ray r3{ {1.0, 1.0, 1.0}, {-1.0, 0.0, 0.0}, 0.0, 2.0 };
	glm::vec3 intersection;
	bool success;

	success = raysPseudoIntersection(r1, r2, intersection);
	REQUIRE(success);

	REQUIRE(intersection.x == Approx(0.5));
	REQUIRE(intersection.y == Approx(1.0));
	REQUIRE(intersection.z == Approx(0.5));

	success = raysPseudoIntersection(r1, r3, intersection);
	REQUIRE(success);

	REQUIRE(intersection.x == Approx(0.0));
	REQUIRE(intersection.y == Approx(1.0));
	REQUIRE(intersection.z == Approx(0.5));
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
