#include "Camera.h"

#include <fstream>
#include <iostream>

#include <utils/warnoff.h>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/calib3d.hpp>
#include <utils/warnon.h>

#include "Constants.h"
#include "ConvertUtils.h"

// ------------------------ Private functions ------------------------

double radiansToDegrees(double radians)
{
    return radians * (180.0 / constants::pi);
}

std::pair<double, double> focalLengthInMm(const cv::Mat1d& cameraMatrix, const cv::Size& imageSize, const cv::Size2d& sensorSize)
{
    return {
        cameraMatrix.at<double>(0, 0) * sensorSize.width / imageSize.width,
        cameraMatrix.at<double>(1, 1) * sensorSize.height / imageSize.height
    };
}

cv::Mat rotationX180(const cv::Mat1d& matrix)
{
    // 180 degrees rotation matrix
    cv::Mat1d R(3, 3, 0.0);

    R.at<double>(0, 0) = 1.0;
    R.at<double>(1, 1) = -1.0;
    R.at<double>(2, 2) = -1.0;

    const cv::Mat1d rotationResult = R * matrix;

    return rotationResult.clone();
}

cv::Mat1d removeZProjectionMatrix(const cv::Mat& projectionMatrix)
{
    cv::Mat1d H(3, 3, 0.0);

    H.at<double>(0, 0) = projectionMatrix.at<double>(0, 0);
    H.at<double>(1, 0) = projectionMatrix.at<double>(1, 0);
    H.at<double>(2, 0) = projectionMatrix.at<double>(2, 0);

    H.at<double>(0, 1) = projectionMatrix.at<double>(0, 1);
    H.at<double>(1, 1) = projectionMatrix.at<double>(1, 1);
    H.at<double>(2, 1) = projectionMatrix.at<double>(2, 1);

    H.at<double>(0, 2) = projectionMatrix.at<double>(0, 3);
    H.at<double>(1, 2) = projectionMatrix.at<double>(1, 3);
    H.at<double>(2, 2) = projectionMatrix.at<double>(2, 3);

    H /= H.at<double>(2, 2);

    return H;
}


cv::Mat1d cameraPose(const cv::Mat1d& rvec, const cv::Mat1d& tvec)
{
    cv::Mat1d R;
    cv::Rodrigues(rvec, R); // R is 3x3

    auto invTvec = -R.t() * tvec; // translation of inverse
    R = rotationX180(R.t());  // rotation of inverse

    cv::Mat1d T = cv::Mat1d::eye(4, 4); // T is 4x4
    T(cv::Range(0, 3), cv::Range(0, 3)) = R.t() * 1; // copies R into T
    T(cv::Range(0, 3), cv::Range(3, 4)) = invTvec * 1; // copies tvec into T

    // To get the Euler angles (XYZ)
    // Go to https://www.andre-gaschler.com/rotationconverter/
    // Copy the rotation matrix R
    // Input the angles (in degrees) in blender

    return T;
}

cv::Vec3d cameraPoseVectorX(const cv::Mat1d& rvec)
{
    cv::Mat1d R;
    cv::Rodrigues(rvec, R); // R is 3x3

    // X is the first column of the R transposed matrix
    cv::Mat1d T = R.t() * 1.0; // copies R into T

    return {
        T.at<double>(0, 0),
        T.at<double>(1, 0),
        T.at<double>(2, 0)
    };
}

// Source: https://answers.opencv.org/question/162932/create-a-stereo-projection-matrix-using-rvec-and-tvec/
cv::Mat1d computeProjectionMatrix(const cv::Mat1d& cameraMatrix, const cv::Mat1d& rvec, const cv::Mat1d& tvec)
{
    cv::Mat1d rotMat(3, 3), rotTransMat(3, 4);
    // Convert rotation vector into rotation matrix 
    cv::Rodrigues(rvec, rotMat);
    // Append translation vector to rotation matrix
    cv::hconcat(rotMat, tvec, rotTransMat);
    // Compute projection matrix by multiplying intrinsic parameter 
    // matrix (A) with 3 x 4 rotation and translation pose matrix (RT).
    // Formula: Projection Matrix = A * RT;
    return cameraMatrix * rotTransMat;
}

cv::Vec3d lookAtPoint(const cv::Mat1d& homography, const cv::Mat1d& cameraMatrix)
{
    assert(homography.rows == 3);
    assert(homography.cols == 4);

    // For the linear system Ax=b to solve
    cv::Mat1d A(3, 3, 0.0);
    A.at<double>(0, 0) = homography.at<double>(0, 0);
    A.at<double>(0, 1) = homography.at<double>(0, 1);
    A.at<double>(1, 0) = homography.at<double>(1, 0);
    A.at<double>(1, 1) = homography.at<double>(1, 1);
    A.at<double>(2, 0) = homography.at<double>(2, 0);
    A.at<double>(2, 1) = homography.at<double>(2, 1);

    A.at<double>(0, 2) = -cameraMatrix.at<double>(0, 2);
    A.at<double>(1, 2) = -cameraMatrix.at<double>(1, 2);
    A.at<double>(2, 2) = -1.0;

    cv::Mat1d b(3, 1, 0.0);
    b.at<double>(0) = -homography.at<double>(0, 3);
    b.at<double>(1) = -homography.at<double>(1, 3);
    b.at<double>(2) = -homography.at<double>(2, 3);

    // Solve for linear least squares
    cv::Mat1d x;
    cv::solve(A, b, x, cv::DECOMP_SVD);

    // Convert to vector
    return {
        x.at<double>(0),
        x.at<double>(1),
        0.0
    };
}

std::tuple<glm::dvec3, glm::dvec3, glm::dvec3> cameraEyeAtUpFromPose(
    const cv::Mat1d& cameraMatrix,
    const cv::Mat1d& rvec,
    const cv::Mat1d& tvec
)
{
    const auto homography = computeProjectionMatrix(cameraMatrix, rvec, tvec);
    const auto pose = cameraPose(rvec, tvec);

    // Find position of the camera in world coordinates
    const glm::dvec3 eye(
        pose.at<double>(0, 3),
        pose.at<double>(1, 3),
        pose.at<double>(2, 3)
    );

    // Find the look at point (intersection between optical center and plane z=0)
    const auto at = convertToGlm(lookAtPoint(homography, cameraMatrix));

    // Get the X vector of the camera in world coordinates
    const auto x = convertToGlm(cameraPoseVectorX(rvec));
    // From the X vector, we can get the up vector
    const auto eyeToAt = glm::normalize(at - eye);
    const auto up = glm::normalize(glm::cross(x, eyeToAt));

    return std::make_tuple(eye, at, up);
}

// ------------------------ Public functions ------------------------

Camera::Camera(const glm::dvec3& eye, const glm::dvec3& at, const glm::dvec3& up):
    m_eye(eye),
    m_at(at),
    m_up(up),
    m_matV(glm::lookAt(eye, at, up)), // Generate the view matrix
    m_matP(glm::perspective(45.0, 1.0, 0.1, 10.0)), // Generate the projection matrix
    m_mat(m_matP * m_matV),
    m_viewport(0.0, 0.0, 1000.0, 1000.0)
{
    
}

Camera::Camera(const glm::dvec3& eye,
               const glm::dvec3& at,
               const glm::dvec3& up,
               double fovy,
               double aspectRatio,
               const glm::dvec2& viewportSize) :
    m_eye(eye),
    m_at(at),
    m_up(up),
    m_matV(glm::lookAt(eye, at, up)), // Generate the view matrix
    m_matP(glm::perspective(fovy, aspectRatio, 0.001, 10.0)), // Generate the projection matrix
    m_mat(m_matP * m_matV),
    m_viewport(0.0, 0.0, viewportSize.x, viewportSize.y)
{
    
}

Camera::Camera(const glm::dvec3& eye,
               const glm::dvec3& at,
               const glm::dvec3& up,
               const glm::dmat4& matV,
               const glm::dmat4& matP,
               const glm::dvec2& viewportSize) :
    m_eye(eye),
    m_at(at),
    m_up(up),
    m_matV(matV),
    m_matP(matP),
    m_mat(m_matP * m_matV),
    m_viewport(0.0, 0.0, viewportSize.x, viewportSize.y)
{
}

glm::dvec3 Camera::project(const glm::dvec3& point) const
{
    return glm::projectNO(point,
                          glm::identity<glm::dmat4>(),
                          m_mat,
                          m_viewport);
}

glm::dvec3 Camera::unProject(const glm::dvec3& point) const
{
    return glm::unProjectNO(point,
                            glm::identity<glm::dmat4>(),
                            m_mat,
                            m_viewport);
}

glm::dvec2 Camera::windowToViewport(const glm::dvec2& point) const
{
    // Remap the viewport coordinates in pixels to [-1, 1]
    return {
        -1.0 + 2.0 * ((point.x - m_viewport[0]) / (m_viewport[2] - m_viewport[0])),
        -1.0 + 2.0 * ((point.y - m_viewport[1]) / (m_viewport[3] - m_viewport[1])),
    };
}

double computeMaximumCameraResolution(const Camera& camera)
{
    return std::max(
        camera.viewport().z - camera.viewport().x,
        camera.viewport().w - camera.viewport().y
    );
}

double computeMaximumCameraResolution(const std::vector<Camera>& cameras)
{
    double maximumResolution = 0.0;

    for (const auto& camera : cameras)
    {
        maximumResolution = std::max(
            maximumResolution,
            computeMaximumCameraResolution(camera)
        );
    }

    return maximumResolution;
}

std::vector<Camera> loadCamerasFromFiles(const std::vector<std::string>& files)
{
    std::vector<Camera> cameras;

    cameras.reserve(files.size());

    for (const auto& filename : files)
    {
        std::ifstream file(filename);

        if (file.is_open())
        {
            int imageWidth, imageHeight;
            glm::dvec3 eye, at, up;
            // These are not used by our camera model, but are still part of the file
            double fovy, aspectRatio, nearPlane, farPlane;
            glm::dmat4 matV, matP;

            file >> imageWidth >> imageHeight;

            file >> eye.x >> eye.y >> eye.z;
            file >> at.x >> at.y >> at.z;
            file >> up.x >> up.y >> up.z;

            file >> fovy;
            file >> aspectRatio;
            file >> nearPlane;
            file >> farPlane;

            file >> matV[0][0] >> matV[1][0] >> matV[2][0] >> matV[3][0];
            file >> matV[0][1] >> matV[1][1] >> matV[2][1] >> matV[3][1];
            file >> matV[0][2] >> matV[1][2] >> matV[2][2] >> matV[3][2];
            file >> matV[0][3] >> matV[1][3] >> matV[2][3] >> matV[3][3];

            file >> matP[0][0] >> matP[1][0] >> matP[2][0] >> matP[3][0];
            file >> matP[0][1] >> matP[1][1] >> matP[2][1] >> matP[3][1];
            file >> matP[0][2] >> matP[1][2] >> matP[2][2] >> matP[3][2];
            file >> matP[0][3] >> matP[1][3] >> matP[2][3] >> matP[3][3];

            file.close();

            cameras.emplace_back(eye, at, up, matV, matP, glm::dvec2(imageWidth, imageHeight));
        }
        else
        {
            std::cerr << "Could not open file: " << filename << std::endl;
        }
    }

    return cameras;
}

SetsOfVec2 undistortAndFlipYAxis(
    const cv::Mat1d& cameraMatrix,
    const cv::Mat1d& distCoeffs,
    int imageHeight,
    const SetsOfVec2& points2d)
{
    // Undistort 2D points and change the frame of reference
    // Copy points
    auto outputPoints2d = points2d;
    for (auto& cameraPoints2D : outputPoints2d)
    {
        for (auto& point2d : cameraPoints2D)
        {
            // Distorted point
            std::vector<cv::Point2d> inputDistortedPoints = { cv::Point2d(point2d.x, point2d.y) };
            std::vector<cv::Point2d> outputUndistortedPoints;

            cv::undistortPoints(inputDistortedPoints, outputUndistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

            // Save undistorted point
            point2d.x = outputUndistortedPoints.front().x;
            point2d.y = static_cast<double>(imageHeight) - outputUndistortedPoints.front().y;
        }
    }

    return outputPoints2d;
}

std::vector<Camera> generateCamerasFromOpenCV(
    int imageWidth,
    int imageHeight,
    const cv::Size2d sensorSize,
    const cv::Mat1d& cameraMatrix,
    const std::vector<cv::Mat1d>& rvecs,
    const std::vector<cv::Mat1d>& tvecs)
{
    std::vector<Camera> cameras;

    const int nbImages = static_cast<int>(std::min(rvecs.size(), tvecs.size()));
    const cv::Size imageSize(imageWidth, imageHeight);

    // Compute aspect ratio from images
    const auto aspectRatio = static_cast<double>(imageWidth) / imageHeight;
    const auto focalLength = focalLengthInMm(cameraMatrix, imageSize, sensorSize);
    const auto fovy = 2.0 * std::atan(sensorSize.height / (2.0 * focalLength.second));

    for (int i = 0; i < nbImages; i++)
    {
        glm::dvec3 eye, at, up;
        std::tie(eye, at, up) = cameraEyeAtUpFromPose(cameraMatrix, rvecs[i], tvecs[i]);

        cameras.emplace_back(eye, at, up,
                             fovy,
                             aspectRatio,
                             glm::dvec2(imageWidth, imageHeight));
    }

    return cameras;
}

SetsOfVec2 projectPoints(const SetOfVec3& points3d, const std::vector<Camera>& cameras)
{
    SetsOfVec2 projected(cameras.size());

    for (unsigned int c = 0; c < cameras.size(); c++)
    {
        for (const auto& point : points3d)
        {
            const auto point2d = cameras[c].project(point);

            // Check depth: the point must be visible from the camera and not behind it
            assert(point2d.z > 0.0);

            projected[c].emplace_back(point2d);
        }
    }

    return projected;
}

SetsOfRays computeRays(
    const std::vector<Camera>& cameras,
    const SetsOfVec2& points
)
{
    assert(points.size() == cameras.size());

    SetsOfRays rays(cameras.size());

    for (unsigned int c = 0; c < cameras.size(); c++)
    {
        for (const auto& point : points[c])
        {
            const glm::dvec3 points3d(point.x, point.y, 1.0);

            const auto unProjected = cameras[c].unProject(points3d);

            const auto origin = cameras[c].eye();
            const auto direction = glm::normalize(unProjected - origin);

            rays[c].emplace_back(origin, direction);
        }
    }

    return rays;
}
