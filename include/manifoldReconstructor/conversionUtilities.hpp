//  Copyright 2014 Andrea Romanoni
//
//  This file is part of edgePointSpaceCarver.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

#ifndef CONVERSIONUTILITIES_HPP_
#define CONVERSIONUTILITIES_HPP_

// This header provides the functions that converts a Matrix from the
// FreespaceDelaunayAlgorithm matrix convention to the Eigen ones
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <glm.hpp>

namespace conversionUtilities {
Eigen::Matrix4f toEigenMatrix(glm::mat4 mat);
cv::Mat toOpenCVMatrix(glm::mat4 mat, bool cameraMat4x4 = true);
}  // namespace conversionUtilities



#endif /* CONVERSIONUTILITIES_HPP_ */
