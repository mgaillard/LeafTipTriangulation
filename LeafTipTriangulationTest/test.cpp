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
	std::vector<cv::Mat1f> homographies;
	std::vector<cv::Vec2f> points;

	for (const auto& camera : cameras)
	{
		const auto homography = convertToOpenCV(removeZRow(camera.mat()));
		const auto projectedPoint = camera.project(point);
		const auto projectedPointOpenCV = convertToOpenCV(camera.windowToViewport(projectedPoint));
		
		// Check that re-projecting the point with the homography gives the right result
		const cv::Vec4f homogeneousPoint(point[0], point[1], point[2], 1.0f);
		const auto reProjectedPoint = projectPoint(homography, homogeneousPoint);
		REQUIRE(reProjectedPoint[0] == Approx(projectedPointOpenCV[0]));
		REQUIRE(reProjectedPoint[1] == Approx(projectedPointOpenCV[1]));

		// Projection matrix
		homographies.push_back(homography);
		// Projected point in viewport coordinates
		points.push_back(projectedPointOpenCV);
	}

	// Error when re-projecting the ground truth should almost 0
	const auto pointOpenCV = convertToOpenCV(point);
	const auto groundTruthError = reprojectionError(homographies, points, pointOpenCV);
	REQUIRE(groundTruthError == Approx(0.0).margin(1e-12));

	float error;
	cv::Vec3f triangulatedPoint;

	std::tie(error, triangulatedPoint) = reconstructPointFromViews(homographies, points);

	REQUIRE(triangulatedPoint[0] == Approx(point.x));
	REQUIRE(triangulatedPoint[1] == Approx(point.y));
	REQUIRE(triangulatedPoint[2] == Approx(point.z));
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

	// Test the triangulation with several points
	testTriangulationOnePoint(cameras, glm::vec3(0.1, -0.2, 0.15));
	testTriangulationOnePoint(cameras, glm::vec3(0.298, -0.767, -0.881));
	testTriangulationOnePoint(cameras, glm::vec3(0.154, 0.715, -0.733));
	testTriangulationOnePoint(cameras, glm::vec3(0.339, 0.957, -0.331));
	testTriangulationOnePoint(cameras, glm::vec3(-0.346, 0.887, -0.869));
}
