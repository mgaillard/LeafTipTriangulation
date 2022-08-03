#pragma once

#include <string>
#include <vector>

#include "Ray.h"
#include "Types.h"

/**
 * \brief Export the whole scene in an OBJ file.
 *	      Points are added as vertices
 *	      Rays are segments (length of 6 meters)
 * \param points A list of triangulated 3D points
 * \param rays Rays A list of 3D Rays
 * \param filename The path to the output file
 * \return True if the file was successfully saved, false if an error occured
 */
bool exportSceneAsOBJ(const SetOfVec3& points,
	                  const SetsOfRays& rays,
	                  const std::string& filename);

/**
 * \brief Save the scene in different files.
 *	      points.obj contains the triangulated 3D points
 *	      camera_{i}.obj contains all rays from camera i
 *	      rays_{j}.obj contains all rays from 3D point j
 * \param rays A list of 3D rays 
 * \param setsOfCorrespondences The mapping between triangulated 3D points and 3D rays
 * \param triangulatedPoints3D A list of triangulated 3D points
 */
void exportSplitSceneAsOBJ(const SetsOfRays& rays,
	                       const SetsOfCorrespondences& setsOfCorrespondences,
	                       const SetOfVec3& triangulatedPoints3D);
