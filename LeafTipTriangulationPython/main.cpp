#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <utils/warnoff.h>
#include <opencv2/core.hpp>
#include <utils/warnon.h>

#include "Camera.h"
#include "Ray.h"
#include "RayMatching.h"

namespace py = pybind11;

// Bindings to use Numpy types instead of glm
glm::dvec3 convertNumpyToGlmVec3(const py::array_t<double>& input)
{
    const auto buf = input.request();

    if (buf.ndim != 1)
    {
        throw std::runtime_error("Number of dimensions must be one");
    }

    if (buf.size != 3)
    {
        throw std::runtime_error("Input shape must be of size 3");
    }

    const auto* ptr = static_cast<const double*>(buf.ptr);

    const glm::dvec3 vec(ptr[0], ptr[1], ptr[2]);

    return vec;
}

py::array_t<double> convertGlmToNumpyVec3(const glm::dvec3& vec)
{
    /* No pointer is passed, so NumPy will allocate the buffer */
    auto result = py::array_t<double>(3);

    py::buffer_info buf = result.request();
    
    auto* ptr = static_cast<double*>(buf.ptr);

    ptr[0] = vec.x;
    ptr[1] = vec.y;
    ptr[2] = vec.z;
    
    return result;
}

glm::dvec2 convertNumpyToGlmVec2(const py::array_t<double>& input)
{
    const auto buf = input.request();

    if (buf.ndim != 1)
    {
        throw std::runtime_error("Number of dimensions must be one");
    }

    if (buf.size != 2)
    {
        throw std::runtime_error("Input shape must be of size 2");
    }

    const auto* ptr = static_cast<const double*>(buf.ptr);

    const glm::dvec2 vec(ptr[0], ptr[1]);

    return vec;
}

py::array_t<double> convertGlmToNumpyVec2(const glm::dvec2& vec)
{
    /* No pointer is passed, so NumPy will allocate the buffer */
    auto result = py::array_t<double>(2);

    py::buffer_info buf = result.request();

    auto* ptr = static_cast<double*>(buf.ptr);

    ptr[0] = vec.x;
    ptr[1] = vec.y;

    return result;
}

SetsOfVec2 convertNumpyToGlmSetsOfVec2(const std::vector<std::vector<py::array_t<double>>>& input)
{
    SetsOfVec2 points2d(input.size());

    for (unsigned int i = 0; i < input.size(); i++)
    {
        for (unsigned int j = 0; j < input[i].size(); j++)
        {
            points2d[i].push_back(convertNumpyToGlmVec2(input[i][j]));
        }
    }

    return points2d;
}

std::vector<std::vector<py::array_t<double>>> convertGlmToNumpySetsOfVec2(const SetsOfVec2& points2d)
{
    std::vector<std::vector<py::array_t<double>>> result(points2d.size());

    for (unsigned int i = 0; i < points2d.size(); i++)
    {
        for (unsigned int j = 0; j < points2d[i].size(); j++)
        {
            result[i].push_back(convertGlmToNumpyVec2(points2d[i][j]));
        }
    }

    return result;
}

std::vector<py::array_t<double>> convertNumpyToGlmSetOfVec3(const SetOfVec3& points3d)
{
    std::vector<py::array_t<double>> result;

    for (const auto& i : points3d)
    {
        result.push_back(convertGlmToNumpyVec3(i));
    }

    return result;
}

cv::Mat1d convertNumpyToOpenCvMat(const py::array_t<double>& input)
{
    const auto buf = input.request();

    if (buf.ndim != 2)
    {
        throw std::runtime_error("Number of dimensions must be 2");
    }

    if (buf.shape[0] < 1 || buf.shape[1] < 1)
    {
        throw std::runtime_error("The shape ");
    }

    const auto rows = static_cast<int>(buf.shape[0]);
    const auto cols = static_cast<int>(buf.shape[1]);

    const auto* ptr = static_cast<const double*>(buf.ptr);
    cv::Mat1d matrix(rows, cols);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            const auto index = j + cols * i;
            matrix.at<double>(i, j) = ptr[index];
        }
    }

    return matrix;
}

std::vector<cv::Mat1d> convertNumpyToVectorOfOpenCvMat(const std::vector<py::array_t<double>>& input)
{
    std::vector<cv::Mat1d> result;

    for (const auto& i : input)
    {
        result.push_back(convertNumpyToOpenCvMat(i));
    }

    return result;
}

PYBIND11_MODULE(LeafTipTriangulationPython, m)
{
    m.doc() = "pybind11 LeafTipTriangulation module plugin";

    // Ray class binding
    py::class_<Ray>(m, "Ray")
        .def(py::init([](const py::array_t<double>& originArray,
                         const py::array_t<double>& directionArray) -> Ray
        {
            const auto origin = convertNumpyToGlmVec3(originArray);
            const auto direction = convertNumpyToGlmVec3(directionArray);

            return {origin, direction};
        }))
        .def("clampRay", &Ray::clampRay)
        .def("isClamped", &Ray::isClamped)
        .def("start", &Ray::start)
        .def("end", &Ray::end)
        .def("at", [](const Ray& ray, double t) -> py::array_t<double>
        {
            return convertGlmToNumpyVec3(ray.at(t));
        });

    // Camera class binding
    py::class_<Camera>(m, "Camera")
        .def(py::init([](const py::array_t<double>& eyeArray,
                           const py::array_t<double>& atArray,
                           const py::array_t<double>& upArray,
                           double fovy,
                           double aspectRatio,
                           const py::array_t<double>& viewportSizeArray) -> Camera
        {
            const auto eye = convertNumpyToGlmVec3(eyeArray);
            const auto at = convertNumpyToGlmVec3(atArray);
            const auto up = convertNumpyToGlmVec3(upArray);
            const auto viewportSize = convertNumpyToGlmVec2(viewportSizeArray);

            return {eye, at, up, fovy, aspectRatio, viewportSize};
        }))
        .def("eye", [](const Camera& camera) -> py::array_t<double>
        {
            return convertGlmToNumpyVec3(camera.eye());
        })
        .def("at", [](const Camera& camera) -> py::array_t<double>
        {
            return convertGlmToNumpyVec3(camera.at());
        })
        .def("up", [](const Camera& camera) -> py::array_t<double>
        {
            return convertGlmToNumpyVec3(camera.up());
        })
        .def("project", [](const Camera& camera, const py::array_t<double>& pointArray) -> py::array_t<double>
        {
            const auto point = convertNumpyToGlmVec3(pointArray);
            return convertGlmToNumpyVec2(camera.project(point));
        });

    // Load cameras from a file
    m.def("loadCamerasFromFiles", &loadCamerasFromFiles);

    m.def("undistortCenterAndFlipYAxis", [](int imageWidth,
                                            int imageHeight,
                                            const py::array_t<double>& cameraMatrixArray,
                                            const py::array_t<double>& distCoeffsArray,
                                            const std::vector<std::vector<py::array_t<double>>>& points2dArray)
                                         -> std::vector<std::vector<py::array_t<double>>>
    {
        const auto cameraMatrix = convertNumpyToOpenCvMat(cameraMatrixArray);
        const auto distCoeffs = convertNumpyToOpenCvMat(distCoeffsArray);
        const auto points2d = convertNumpyToGlmSetsOfVec2(points2dArray);

        const auto outputPoints2d = undistortCenterAndFlipYAxis(imageWidth, imageHeight, cameraMatrix, distCoeffs, points2d);

        return convertGlmToNumpySetsOfVec2(outputPoints2d);
    });

    m.def("generateCamerasFromOpenCV", [](int imageWidth,
                                          int imageHeight,
                                          double sensorWidthInMm,
                                          double sensorHeightInMm,
                                          const py::array_t<double>& cameraMatrixArray,
                                          const std::vector<py::array_t<double>>& rvecsArray,
                                          const std::vector<py::array_t<double>>& tvecsArray)
                                       -> std::vector<Camera>
    {
        const auto cameraMatrix = convertNumpyToOpenCvMat(cameraMatrixArray);
        const auto rvecs = convertNumpyToVectorOfOpenCvMat(rvecsArray);
        const auto tvecs = convertNumpyToVectorOfOpenCvMat(tvecsArray);

        return generateCamerasFromOpenCV(imageWidth,
                                         imageHeight,
                                         {sensorWidthInMm, sensorHeightInMm},
                                         cameraMatrix,
                                         rvecs,
                                         tvecs);
    });

    // Get the set of rays from 2D points
    m.def("computeRays", [](const std::vector<Camera>& cameras,
                            const std::vector<std::vector<py::array_t<double>>>& points2dArray) -> SetsOfRays
    {
        const auto points2d = convertNumpyToGlmSetsOfVec2(points2dArray);
        return computeRays(cameras, points2d);
    });

    m.def("matchRaysAndTriangulate", [](const std::vector<Camera>& cameras,
                                               const std::vector<std::vector<py::array_t<double>>>& points2dArray,
                                               const SetsOfRays& rays,
                                               double thresholdNoPair) -> std::vector<py::array_t<double>>
    {
        const auto points2d = convertNumpyToGlmSetsOfVec2(points2dArray);
        const auto [points3d, setsOfCorrespondences] = matchRaysAndTriangulate(cameras,
                                                                               points2d,
                                                                               rays,
                                                                               thresholdNoPair);
        return convertNumpyToGlmSetOfVec3(points3d);
    });
}
