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

#include "KittiCamParser.h"


KittiCamParser::KittiCamParser(std::string fileName) : fileName_(fileName), numCameras_(0), numPoints_(0) {
  fileStream_.open(fileName_.c_str(),std::ios::in);

}

KittiCamParser::~KittiCamParser() {
}

bool KittiCamParser::parseFile() {

  std::string line;
  //read and discard the first line (# KittiCamParser)
  std::getline(fileStream_, line);

  //read num cam
  std::getline(fileStream_, line);
  std::istringstream iss(line);
  iss >> numCameras_ >> numPoints_;

  for (int curCam = 0; curCam < numCameras_; ++curCam) {
    Camera tempCamera;

    //first line <f, k1, k2>
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempCamera.fx >> tempCamera.fy >> tempCamera.cx >> tempCamera.cy >> tempCamera.k1 >> tempCamera.k2 >> tempCamera.k3 >> tempCamera.k4 >> tempCamera.k5;
    sscanf(iss.str().c_str(), "%f %f %f %f %f %f %f %f %f", &tempCamera.fx,&tempCamera.fy,&tempCamera.cx,&tempCamera.cy,&tempCamera.k1,
        &tempCamera.k2,&tempCamera.k3,&tempCamera.k4,&tempCamera.k5);
//    std::cout<<iss.str()<<std::endl;

    //rotation matrix <R>
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempCamera.R(0,0) >> tempCamera.R(0,1) >>  tempCamera.R(0,2);
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempCamera.R(1,0) >> tempCamera.R(1,1) >>  tempCamera.R(1,2);
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempCamera.R(2,0) >> tempCamera.R(2,1) >>  tempCamera.R(2,2);

    //translation vector <t>
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempCamera.t(0) >> tempCamera.t(1) >>  tempCamera.t(2);

    //camera center calc
    tempCamera.center = -1 * tempCamera.R.transpose() * tempCamera.t;
    camerasList_.push_back(tempCamera);
  }

  for (int curPoint = 0; curPoint < numPoints_; ++curPoint) {
    PointParser tempPoint;

    //point's position <x, y, z>
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempPoint.x >> tempPoint.y >> tempPoint.z;

//    std::cout<<"TXT Line "<<iss.str();
//    std::cout<<"values read x= "<< tempPoint.x <<" y = "<< tempPoint.y <<" z= "<<tempPoint.z<<std::endl;
    //std::cout<<iss.str()<<endl;
    //poin'st color
    std::getline(fileStream_, line);
    iss.str(line);
    iss >> tempPoint.R >> tempPoint.G >> tempPoint.B;

    //view list
    std::getline(fileStream_, line);
    iss.str(line);

    int lengthList,  tempKey, tempIdx;
    float tempX, tempY;
    iss >> lengthList;
    for (int curViewingCamera = 0; curViewingCamera < lengthList; ++curViewingCamera) {
      iss >> tempIdx;
      tempPoint.viewingCamerasIndices.push_back(tempIdx);
      camerasList_[tempIdx].viewingPointsIndices.push_back(curPoint);
      iss >> tempKey;
      iss >> tempX;
      tempPoint.viewingCamerasX.push_back(tempX);
      iss >> tempY;
      tempPoint.viewingCamerasY.push_back(tempY);
    }

    pointsList_.push_back(tempPoint);

  }

  return true;
}

