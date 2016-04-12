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

#include "KittiRectifiedCamParser.h"


KittiRectifiedCamParser::KittiRectifiedCamParser(std::string fileName) : fileName_(fileName), numCameras_(0), numPoints_(0) {
  fileStream_.open(fileName_.c_str(),std::ios::in);

}

KittiRectifiedCamParser::~KittiRectifiedCamParser() {
}

bool KittiRectifiedCamParser::parseFile() {

  std::string line;
  //read and discard the first line (# KittiCamParser)
  std::getline(fileStream_, line);

  //read num cam
  std::getline(fileStream_, line);
  std::istringstream iss(line);
  iss >> numCameras_;

  for (int curCam = 0; curCam < numCameras_; ++curCam) {
    CameraRect tempCamera;

    //rotation matrix <R>
    std::getline(fileStream_, line);
    iss.str(line);
    sscanf(iss.str().c_str(), "%f %f %f %f", & tempCamera.P(0,0),&tempCamera.P(0,1),&tempCamera.P(0,2),&tempCamera.P(0,3));
    //iss >> tempCamera.P(0,0) >> tempCamera.P(0,1) >>  tempCamera.P(0,2) >> tempCamera.P(0,3);
    std::getline(fileStream_, line);
    iss.str(line);
    sscanf(iss.str().c_str(), "%f %f %f %f", & tempCamera.P(1,0),&tempCamera.P(1,1),&tempCamera.P(1,2),&tempCamera.P(1,3));
//    iss >> tempCamera.P(1,0) >> tempCamera.P(1,1) >>  tempCamera.P(1,2) >> tempCamera.P(1,3);
    std::getline(fileStream_, line);
    iss.str(line);
    sscanf(iss.str().c_str(), "%f %f %f %f", & tempCamera.P(2,0),&tempCamera.P(2,1),&tempCamera.P(2,2),&tempCamera.P(2,3));
//    iss >> tempCamera.P(2,0) >> tempCamera.P(2,1) >>  tempCamera.P(2,2) >> tempCamera.P(2,3);


    //translation vector <t>
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempCamera.center(0) >> tempCamera.center(1) >>  tempCamera.center(2);

    camerasList_.push_back(tempCamera);
  }

  return true;
}
