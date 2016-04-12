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

#ifndef KITTICAMPARSER_H_
#define KITTICAMPARSER_H_

// This class read the camera calibrations files generated from the
// KITTI dataset ground truth
#include "Parser.h"

#include <iostream>
#include <fstream>
#include <vector>


#include <Eigen/Core>

struct Camera {
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    //camera center
    Eigen::Vector3f center;

    std::vector<int> viewingPointsIndices;
};
class KittiCamParser {
  public:
    KittiCamParser(std::string fileInput);
    virtual ~KittiCamParser();
    bool parseFile();

    const std::vector<Camera>& getCamerasList() const {
      return camerasList_;
    }

    int getNumCameras() const {
      return numCameras_;
    }

    int getNumPoints() const {
      return numPoints_;
    }

    const std::vector<PointParser>& getPointsList() const {
      return pointsList_;
    }
protected:
    std::string fileName_;
    std::ifstream fileStream_;
    int numCameras_;
    int numPoints_;
    std::vector<Camera> camerasList_;
    std::vector<PointParser> pointsList_;
};
#endif /* KITTICAMPARSER_H_ */
