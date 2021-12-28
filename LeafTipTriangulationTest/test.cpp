#include <catch2/catch.hpp>

#include "Camera.h"
#include "Triangulation.h"

TEST_CASE("Triangulation from 2 views", "[triangulation]")
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

	// Define a point to project on all views
	const glm::vec3 point(0.1, -0.2, 0.15);
	const auto pointOpenCV = convertToOpenCV(point);

	// Project a point on the two views
	const glm::vec2 projectedPoint1(camera1.project(point));
	const glm::vec2 projectedPoint2(camera2.project(point));

	// Matrices to project a 3D point to 2D point in viewport coordinates
	const std::vector<cv::Mat1f> homographies = {
		convertToOpenCV(removeZRow(camera1.mat())),
		convertToOpenCV(removeZRow(camera2.mat()))
	};

	// Projections of the point in 2D in viewport coordinates
	const std::vector<cv::Vec2f> points = {
		convertToOpenCV(camera1.windowToViewport(projectedPoint1)),
		convertToOpenCV(camera1.windowToViewport(projectedPoint2))
	};

	// Check that re-projecting the point with the homographies gives the right result
	const cv::Vec4f homogeneousPoint(point[0], point[1], point[2], 1.0f);
	const auto reProjectedPoint1 = projectPoint(homographies[0], homogeneousPoint);
	const auto reProjectedPoint2 = projectPoint(homographies[1], homogeneousPoint);
	REQUIRE(reProjectedPoint1[0] == Approx(points[0][0]));
	REQUIRE(reProjectedPoint1[1] == Approx(points[0][1]));
	REQUIRE(reProjectedPoint2[0] == Approx(points[1][0]));
	REQUIRE(reProjectedPoint2[1] == Approx(points[1][1]));

	// Error when re-projecting the ground truth should almost 0
	const auto groundTruthError = reprojectionError(homographies, points, pointOpenCV);
	REQUIRE(groundTruthError == Approx(0.0).margin(1e-12));

	float error;
	cv::Vec3f triangulatedPoint;

	std::tie(error, triangulatedPoint) = reconstructPointFromViews(homographies, points);

	REQUIRE(triangulatedPoint[0] == Approx(point.x));
	REQUIRE(triangulatedPoint[1] == Approx(point.y));
	REQUIRE(triangulatedPoint[2] == Approx(point.z));
}
