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
#ifndef KITTIRECTIFIEDCAMPARSER_H_
#define KITTIRECTIFIEDCAMPARSER_H_

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <types_reconstructor.hpp>
#include <types_config.hpp>


class KittiRectifiedCamParser {
  public:
    KittiRectifiedCamParser(std::string fileInput);
    virtual ~KittiRectifiedCamParser();
    bool parseFile();

    const std::vector<CameraRect>& getCamerasList() const {
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
    std::vector<CameraRect> camerasList_;
    std::vector<PointParser> pointsList_;
};
#endif /* KITTIRECTIFIEDCAMPARSER_H_ */
