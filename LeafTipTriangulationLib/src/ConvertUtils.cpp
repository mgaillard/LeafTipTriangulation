#include "ConvertUtils.h"

cv::Vec2f convertToOpenCV(const glm::vec2& v)
{
	cv::Vec2f output;

	output[0] = v.x;
	output[1] = v.y;

	return output;
}

cv::Vec3f convertToOpenCV(const glm::vec3& v)
{
	cv::Vec3f output;

	output[0] = v.x;
	output[1] = v.y;
	output[2] = v.z;

	return output;
}

glm::vec3 convertToGlm(const cv::Vec3f& v)
{
	return { v[0], v[1], v[2] };
}

glm::vec3 convertToGlm(const cv::Vec3d& v)
{
	return { v[0], v[1], v[2] };
}

glm::vec4 convertToGlm(const cv::Vec4f& v)
{
	return { v[0], v[1], v[2], v[3] };
}
