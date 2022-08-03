#pragma once

#include <string>
#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <opencv2/core/core.hpp>
#include <utils/warnon.h>

#include "Types.h"

/**
 * \brief Measure the length between two points triangulated without correspondences
 * \param imageWidth Resolution of width of images
 * \param imageHeight Resolution of height of images
 * \param cameraMatrix Camera matrix of the camera used to take images
 * \param distCoeffs Distortion coefficients for the camera
 * \param rvecs Per-view rotation in Rodrigues angles 
 * \param tvecs Per-view translation
 * \param inputPoints2d Per-view 2D coordinates of the two points
 * \return The distance between the two points
 */
double measureTwoPointsCharuco(int imageWidth,
                               int imageHeight,
                               const cv::Mat1d& cameraMatrix,
                               const cv::Mat1d& distCoeffs,
                               const std::vector<cv::Mat1d>& rvecs,
                               const std::vector<cv::Mat1d>& tvecs,
                               const SetsOfVec2& inputPoints2d);
