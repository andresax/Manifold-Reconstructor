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


// This class read the rectified camera calibrations files generated from the
// KITTI dataset ground truth
#ifndef CAMPARSER_H_
#define CAMPARSER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <types_reconstructor.hpp>
#include <types_config.hpp>

#include <glm.hpp>

class CamParser {
  public:
    CamParser(std::string fileInput);
    virtual ~CamParser();
    bool parseFile(const int &downsample = 1);

    const std::vector<CameraType>& getCamerasList() const {
      return camerasList_;
    }

    int getNumCameras() const {
      return numCameras_;
    }

    int getNumPoints() const {
      return numPoints_;
    }

  const std::vector<std::string>& getCamerasPaths() const {
    return camerasPaths_;
  }

  protected:
    std::string fileName_;
    std::ifstream fileStream_;
    int numCameras_;
    int numPoints_;
    std::vector<CameraType> camerasList_;
    std::vector<std::string> camerasPaths_;
};
#endif /* CAMPARSER_H_ */
