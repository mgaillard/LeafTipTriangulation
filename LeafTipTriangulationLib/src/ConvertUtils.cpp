#include "ConvertUtils.h"

glm::vec3 convertToGlm(const cv::Vec3f& v)
{
    return { v[0], v[1], v[2] };
}

glm::dvec3 convertToGlm(const cv::Vec3d& v)
{
    return { v[0], v[1], v[2] };
}

glm::vec4 convertToGlm(const cv::Vec4f& v)
{
    return { v[0], v[1], v[2], v[3] };
}
