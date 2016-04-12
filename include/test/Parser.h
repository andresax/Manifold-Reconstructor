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

#ifndef PARSER_H_
#define PARSER_H_

// This class has to be extended with a class implementing the parsing function.
// It provides camera calibration parameters and point positions

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <vector>
#include <types_reconstructor.hpp>
#include <types_config.hpp>


class Parser {
  public:
    Parser(std::string fileInput) : fileName_(fileInput), numCameras_(0), numPoints_(0){}
    virtual ~Parser() {};
    virtual bool parseFile() = 0;

    const std::vector<SensorParser>& getCamerasList() const {
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

    std::vector<SensorParser> camerasList_;
    std::vector<PointParser> pointsList_;
};

#endif /* PARSER_H_ */
