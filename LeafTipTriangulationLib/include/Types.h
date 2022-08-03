#pragma once

#include <vector>

#include <utils/warnoff.h>
#include <glm/glm.hpp>
#include <utils/warnon.h>

#include "Ray.h"

using SetOfVec2 = std::vector<glm::dvec2>;
using SetsOfVec2 = std::vector<SetOfVec2>;

using SetOfVec3 = std::vector<glm::dvec3>;
using SetsOfVec3 = std::vector<SetOfVec3>;

using SetOfRays = std::vector<Ray>;
using SetsOfRays = std::vector<SetOfRays>;

using SetOfCorrespondences = std::vector<std::pair<int, int>>;
using SetsOfCorrespondences = std::vector<SetOfCorrespondences>;
